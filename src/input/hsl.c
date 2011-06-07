/* OpenBSC Abis input driver for HSL Femto */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by On-Waves
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* HSL uses a much more primitive/simplified version of the IPA multiplex.
 *
 * They have taken out the nice parts like the ID_GET / ID_RESP for resolving
 * the UNIT ID, as well as the keepalive ping/pong messages.  Furthermore, the
 * Stream Identifiers are fixed on the BTS side (RSL always 0, OML always 0xff)
 * and both OML+RSL share a single TCP connection.
 *
 * Other oddities include the encapsulation of BSSGP messages in the L3_INFO IE
 * of RSL
 */

#include "internal.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <osmocom/core/select.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/core/msgb.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/core/logging.h>
#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/core/socket.h>
#include <talloc.h>

#define HSL_TCP_PORT	2500
#define HSL_PROTO_DEBUG	0xdd

#define PRIV_OML 1
#define PRIV_RSL 2

static void *tall_hsl_ctx;

/* data structure for one E1 interface with A-bis */
struct hsl_e1_handle {
	struct osmo_fd listen_fd;
	struct gsm_network *gsmnet;
};

static struct hsl_e1_handle *e1h;


#define TS1_ALLOC_SIZE	900

static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *link;
	struct ipaccess_head *hh;
	struct msgb *msg;
	int ret = 0, error;

	msg = ipaccess_read_msg(bfd, &error);
	if (!msg) {
		if (e1i_ts->line->ops.error)
			e1i_ts->line->ops.error(NULL, error);
		if (error == 0) {
			osmo_fd_unregister(bfd);
			close(bfd->fd);
			bfd->fd = -1;
		}
		return error;
	}
	DEBUGP(DMI, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	hh = (struct ipaccess_head *) msg->data;
	if (hh->proto == HSL_PROTO_DEBUG) {
		LOGP(DINP, LOGL_NOTICE, "HSL debug: %s\n", msg->data + sizeof(*hh));
		msgb_free(msg);
		return ret;
	}

	/* HSL proprietary RSL extension */
	if (hh->proto == 0 && (msg->l2h[0] == 0x81 || msg->l2h[0] == 0x80)) {
		if (!line->ops.sign_link_up) {
			LOGP(DINP, LOGL_ERROR, "Fix your application, no "
				"action set if the signalling link "
				"becomes ready.\n");
			return -EINVAL;
		}
		ret = line->ops.sign_link_up(msg, line);
		if (ret < 0) {
			/* FIXME: close connection */
			if (line->ops.error)
				line->ops.error(msg, -EBADMSG);
			return ret;
		} else if (ret == 1)
			return 0;
		/* else: continue... */
	}

#ifdef HSL_SR_1_0
	/* HSL for whatever reason chose to use 0x81 instead of 0x80 for FOM */
	if (hh->proto == 255 && msg->l2h[0] == (ABIS_OM_MDISC_FOM | 0x01))
		msg->l2h[0] = ABIS_OM_MDISC_FOM;
#endif
	link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (!link) {
		LOGP(DINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		msgb_free(msg);
		return -EIO;
	}
	msg->dst = link;

	/* XXX: better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops.sign_link) {
		LOGP(DINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		return -ENOENT;
	}
	e1i_ts->line->ops.sign_link(msg, link);

	return ret;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	e1i_ts->driver.ipaccess.fd.when |= BSC_FD_WRITE;

	return 0;
}

static void timeout_ts1_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	ts_want_write(e1i_ts);
}

static int handle_ts1_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *sign_link;
	struct msgb *msg;
	uint8_t proto;
	int ret;

	bfd->when &= ~BSC_FD_WRITE;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		return 0;
	}

	switch (sign_link->type) {
	case E1INP_SIGN_OML:
		proto = IPAC_PROTO_OML;
#ifdef HSL_SR_1_0
		/* HSL uses 0x81 for FOM for some reason */
		if (msg->data[0] == ABIS_OM_MDISC_FOM)
			msg->data[0] = ABIS_OM_MDISC_FOM | 0x01;
#endif
		break;
	case E1INP_SIGN_RSL:
		proto = IPAC_PROTO_RSL;
		break;
	default:
		msgb_free(msg);
		bfd->when |= BSC_FD_WRITE; /* come back for more msg */
		return -EINVAL;
	}

	msg->l2h = msg->data;
	ipaccess_prepend_header(msg, sign_link->tei);

	DEBUGP(DMI, "TX %u: %s\n", ts_nr, osmo_hexdump(msg->l2h, msgb_l2len(msg)));

	ret = send(bfd->fd, msg->data, msg->len, 0);
	msgb_free(msg);

	/* set tx delay timer for next event */
	e1i_ts->sign.tx_timer.cb = timeout_ts1_write;
	e1i_ts->sign.tx_timer.data = e1i_ts;

	/* Reducing this might break the nanoBTS 900 init. */
	osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, e1i_ts->sign.delay);

	return ret;
}

/* callback from select.c in case one of the fd's can be read/written */
static int hsl_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	unsigned int idx = ts_nr-1;
	struct e1inp_ts *e1i_ts;
	int rc = 0;

	/* In case of early RSL we might not yet have a line */

	if (line)
		e1i_ts = &line->ts[idx];

	if (!line || e1i_ts->type == E1INP_TS_TYPE_SIGN) {
		if (what & BSC_FD_READ)
			rc = handle_ts1_read(bfd);
		if (what & BSC_FD_WRITE)
			rc = handle_ts1_write(bfd);
	} else
		LOGP(DINP, LOGL_ERROR, "unknown E1 TS type %u\n", e1i_ts->type);

	return rc;
}

static int hsl_line_update(struct e1inp_line *line, enum e1inp_line_role role);

struct e1inp_driver hsl_driver = {
	.name = "hsl",
	.want_write = ts_want_write,
	.line_update = hsl_line_update,
	.default_delay = 0,
};

/* callback of the OML listening filedescriptor */
static int listen_fd_cb(struct osmo_fd *listen_bfd, unsigned int what)
{
	int ret;
	int idx = 0;
	int i;
	struct e1inp_line *line = listen_bfd->data;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);

	if (!(what & BSC_FD_READ))
		return 0;

	ret = accept(listen_bfd->fd, (struct sockaddr *) &sa, &sa_len);
	if (ret < 0) {
		perror("accept");
		return ret;
	}
	LOGP(DINP, LOGL_NOTICE, "accept()ed new HSL link from %s\n",
		inet_ntoa(sa.sin_addr));

	/* create virrtual E1 timeslots for signalling */
	e1inp_ts_config_sign(&line->ts[1-1], line);

	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	e1i_ts = &line->ts[idx];

	bfd = &e1i_ts->driver.ipaccess.fd;
	bfd->fd = ret;
	bfd->data = line;
	bfd->priv_nr = PRIV_OML;
	bfd->cb = hsl_fd_cb;
	bfd->when = BSC_FD_READ;
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "could not register FD\n");
		close(bfd->fd);
		talloc_free(line);
		return ret;
	}

        return ret;
}

static int
hsl_line_update(struct e1inp_line *line, enum e1inp_line_role role)
{
	int ret = -ENOENT;

	switch(role) {
	case E1INP_LINE_R_BSC:
		ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
					 "0.0.0.0", HSL_TCP_PORT, 1);
		if (ret < 0)
			return ret;

		e1h->listen_fd.fd = ret;
		e1h->listen_fd.when |= BSC_FD_READ;
		e1h->listen_fd.cb = listen_fd_cb;
		e1h->listen_fd.data = line;

		if (osmo_fd_register(&e1h->listen_fd) < 0) {
			close(ret);
			return ret;
		}
		break;
	case E1INP_LINE_R_BTS:
		/* XXX: not implemented yet. */
		break;
	default:
		break;
	}
	return ret;
}

int hsl_setup(struct gsm_network *gsmnet)
{
	e1h->gsmnet = gsmnet;
	return 0;
}

void e1inp_hsl_init(void)
{
	tall_hsl_ctx = talloc_named_const(libosmo_abis_ctx, 1, "hsl");

	e1h = talloc_zero(tall_hsl_ctx, struct hsl_e1_handle);
	if (!e1h)
		return;

	e1inp_driver_register(&hsl_driver);
}

/* OpenBSC Abis input driver for ip.access */

/* (C) 2009 by Harald Welte <laforge@gnumonks.org>
 * (C) 2010 by Holger Hans Peter Freyther
 * (C) 2010 by On-Waves
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
#include <osmocom/core/logging.h>
#include <talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>
#include <osmocom/abis/logging.h>
#include <osmocom/abis/ipa.h>

static void *tall_ipa_ctx;

#define TS1_ALLOC_SIZE	900

/*
 * Common propietary IPA messages:
 *      - PONG: in reply to PING.
 *      - ID_REQUEST: first messages once OML has been established.
 *      - ID_ACK: in reply to ID_ACK.
 */
const uint8_t ipa_pong_msg[] = {
	0, 1, IPAC_PROTO_IPACCESS, IPAC_MSGT_PONG
};

const uint8_t ipa_id_ack_msg[] = {
	0, 1, IPAC_PROTO_IPACCESS, IPAC_MSGT_ID_ACK
};

const uint8_t ipa_id_req_msg[] = {
	0, 17, IPAC_PROTO_IPACCESS, IPAC_MSGT_ID_GET,
	0x01, IPAC_IDTAG_UNIT,
	0x01, IPAC_IDTAG_MACADDR,
	0x01, IPAC_IDTAG_LOCATION1,
	0x01, IPAC_IDTAG_LOCATION2,
	0x01, IPAC_IDTAG_EQUIPVERS,
	0x01, IPAC_IDTAG_SWVERSION,
	0x01, IPAC_IDTAG_UNITNAME,
	0x01, IPAC_IDTAG_SERNR,
};

static int ipaccess_send(int fd, const void *msg, size_t msglen)
{
	int ret;

	ret = write(fd, msg, msglen);
	if (ret < 0)
		return ret;
	if (ret < msglen) {
		LOGP(DINP, LOGL_ERROR, "ipaccess_send: short write\n");
		return -EIO;
	}
	return ret;
}

int ipaccess_send_pong(int fd)
{
	return ipaccess_send(fd, ipa_pong_msg, sizeof(ipa_pong_msg));
}

int ipaccess_send_id_ack(int fd)
{
	return ipaccess_send(fd, ipa_id_ack_msg, sizeof(ipa_id_ack_msg));
}

int ipaccess_send_id_req(int fd)
{
	return ipaccess_send(fd, ipa_id_req_msg, sizeof(ipa_id_req_msg));
}

/* base handling of the ip.access protocol */
int ipaccess_rcvmsg_base(struct msgb *msg,
			 struct osmo_fd *bfd)
{
	uint8_t msg_type = *(msg->l2h);
	int ret = 0;

	switch (msg_type) {
	case IPAC_MSGT_PING:
		ret = ipaccess_send_pong(bfd->fd);
		break;
	case IPAC_MSGT_PONG:
		DEBUGP(DMI, "PONG!\n");
		break;
	case IPAC_MSGT_ID_ACK:
		DEBUGP(DMI, "ID_ACK? -> ACK!\n");
		ret = ipaccess_send_id_ack(bfd->fd);
		break;
	}
	return 0;
}

static int ipaccess_rcvmsg(struct e1inp_line *line, struct msgb *msg,
			   struct osmo_fd *bfd)
{
	uint8_t msg_type = *(msg->l2h);

	/* handle base messages */
	ipaccess_rcvmsg_base(msg, bfd);

	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		DEBUGP(DMI, "ID_RESP\n");
		if (!line->ops->sign_link_up) {
			LOGP(DINP, LOGL_ERROR, "Fix your application, "
				"no action set if the signalling link "
				"becomes ready\n");
			return -EINVAL;
		}
		line->ops->sign_link_up(msg, line, bfd->priv_nr);
		break;
	}
	return 0;
}

static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = NULL;
	struct e1inp_sign_link *link;
	struct ipaccess_head *hh;
	struct msgb *msg;
	int ret = 0, error;

	error = ipa_msg_recv(bfd->fd, &msg);
	if (error <= 0) {
		/* skip if RSL line is not set yet. */
		if (e1i_ts && e1i_ts->line->ops->error)
			e1i_ts->line->ops->error(NULL, line, ts_nr, error);
		if (error == 0) {
		        osmo_fd_unregister(bfd);
		        close(bfd->fd);
		        bfd->fd = -1;
			talloc_free(bfd);
		}
		return error;
	}
	DEBUGP(DMI, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	/* Now that we're sure that we've got a line for socket. */
	e1i_ts = &line->ts[ts_nr-1];

	hh = (struct ipaccess_head *) msg->data;
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		ipaccess_rcvmsg(line, msg, bfd);
		msgb_free(msg);
		return ret;
	}
	/* BIG FAT WARNING: bfd might no longer exist here, since ipaccess_rcvmsg()
	 * might have free'd it !!! */

	link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (!link) {
		LOGP(DINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		msgb_free(msg);
		return -EIO;
	}
	msg->dst = link;
	msg->trx = link->trx;

	/* XXX better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops->sign_link) {
		LOGP(DINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		return -ENOENT;
	}
	e1i_ts->line->ops->sign_link(msg, line, link);

	return ret;
}

void ipaccess_prepend_header_ext(struct msgb *msg, int proto)
{
	struct ipaccess_head_ext *hh_ext;

	/* prepend the osmo ip.access header extension */
	hh_ext = (struct ipaccess_head_ext *) msgb_push(msg, sizeof(*hh_ext));
	hh_ext->proto = proto;
}

void ipaccess_prepend_header(struct msgb *msg, int proto)
{
	struct ipaccess_head *hh;

	/* prepend the ip.access header */
	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh));
	hh->len = htons(msg->len - sizeof(*hh));
	hh->proto = proto;
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
static int ipaccess_fd_cb(struct osmo_fd *bfd, unsigned int what)
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

static int ipaccess_line_update(struct e1inp_line *line,
				enum e1inp_line_role role, const char *addr);

struct e1inp_driver ipaccess_driver = {
	.name = "ipa",
	.want_write = ts_want_write,
	.line_update = ipaccess_line_update,
	.default_delay = 0,
};

/* callback of the OML listening filedescriptor */
static int ipaccess_bsc_oml_cb(struct ipa_server_link *link, int fd)
{
	int ret;
	int idx = 0;
	int i;
	struct e1inp_line *line = link->line;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;

	/* create virrtual E1 timeslots for signalling */
	e1inp_ts_config_sign(&line->ts[1-1], line);

	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	e1i_ts = &line->ts[idx];

	bfd = &e1i_ts->driver.ipaccess.fd;
	bfd->fd = fd;
	bfd->data = line;
	bfd->priv_nr = E1INP_SIGN_OML;
	bfd->cb = ipaccess_fd_cb;
	bfd->when = BSC_FD_READ;
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "could not register FD\n");
		close(bfd->fd);
		return ret;
	}

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipaccess_send_id_req(bfd->fd);

        return ret;
}

static int ipaccess_bsc_rsl_cb(struct ipa_server_link *link, int fd)
{
	struct osmo_fd *bfd;
	struct e1inp_line *line = link->line;
	int ret;

	/* create virtual E1 timeslots for signalling */
	e1inp_ts_config_sign(&line->ts[E1INP_SIGN_RSL-1], line);

	bfd = talloc_zero(tall_ipa_ctx, struct osmo_fd);
	if (!bfd)
		return -ENOMEM;

	bfd->fd = fd;
	bfd->data = line;
	bfd->priv_nr = E1INP_SIGN_RSL;
	bfd->cb = ipaccess_fd_cb;
	bfd->when = BSC_FD_READ;
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "could not register FD\n");
		close(bfd->fd);
		talloc_free(bfd);
		return ret;
	}
	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipaccess_send_id_req(bfd->fd);

	return 0;
}

static int ipaccess_bts_cb(struct ipa_client_link *link, struct msgb *msg)
{
	struct ipaccess_head *hh = (struct ipaccess_head *) msg->data;
	struct e1inp_ts *e1i_ts = NULL;
	struct e1inp_sign_link *sign_link;

	/* special handling for IPA CCM. */
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		uint8_t msg_type = *(msg->l2h);

		/* ping, pong and acknowledgment cases. */
		ipaccess_rcvmsg_base(msg, &link->ofd);

		/* this is a request for identification from the BSC. */
		if (msg_type == IPAC_MSGT_ID_GET) {
			LOGP(DINP, LOGL_NOTICE, "received ID get\n");
			if (!link->line->ops->sign_link_up) {
				LOGP(DINP, LOGL_ERROR, "Fix your application, "
					"no action set if the signalling link "
					"becomes ready\n");
				return -EINVAL;
			}
			link->line->ops->sign_link_up(msg, link->line,
				     link->port == IPA_TCP_PORT_OML ?
					E1INP_SIGN_OML : E1INP_SIGN_RSL);
		}
		return 0;
	} else if (link->port == IPA_TCP_PORT_OML)
		e1i_ts = &link->line->ts[0];
	else if (link->port == IPA_TCP_PORT_RSL)
		e1i_ts = &link->line->ts[1];

	/* look up for some existing signaling link. */
	sign_link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (sign_link == NULL) {
		LOGP(DINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		msgb_free(msg);
		return -EIO;
	}
	msg->dst = sign_link;
	msg->trx = sign_link->trx;

	/* XXX better use e1inp_ts_rx? */
	if (!link->line->ops->sign_link) {
		LOGP(DINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		return -ENOENT;
	}
	link->line->ops->sign_link(msg, link->line, sign_link);
	return 0;
}

static int ipaccess_line_update(struct e1inp_line *line,
				enum e1inp_line_role role, const char *addr)
{
	int ret = -ENOENT;

	switch(role) {
	case E1INP_LINE_R_BSC: {
		struct ipa_server_link *oml_link, *rsl_link;

		LOGP(DINP, LOGL_NOTICE, "enabling ipaccess BSC mode\n");

		oml_link = ipa_server_link_create(tall_ipa_ctx, line,
					          "0.0.0.0", IPA_TCP_PORT_OML,
						  ipaccess_bsc_oml_cb, NULL);
		if (oml_link == NULL) {
			LOGP(DINP, LOGL_ERROR, "cannot create OML "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_server_link_open(oml_link) < 0) {
			LOGP(DINP, LOGL_ERROR, "cannot open OML BSC link: %s\n",
				strerror(errno));
			ipa_server_link_close(oml_link);
			ipa_server_link_destroy(oml_link);
			return -EIO;
		}
		rsl_link = ipa_server_link_create(tall_ipa_ctx, line,
						  "0.0.0.0", IPA_TCP_PORT_RSL,
						  ipaccess_bsc_rsl_cb, NULL);
		if (rsl_link == NULL) {
			LOGP(DINP, LOGL_ERROR, "cannot create RSL "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_server_link_open(rsl_link) < 0) {
			LOGP(DINP, LOGL_ERROR, "cannot open RSL BSC link: %s\n",
				strerror(errno));
			ipa_server_link_close(rsl_link);
			ipa_server_link_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	case E1INP_LINE_R_BTS: {
		struct ipa_client_link *link, *rsl_link;

		LOGP(DINP, LOGL_NOTICE, "enabling ipaccess BTS mode\n");

		link = ipa_client_link_create(tall_ipa_ctx, line,
					      addr, IPA_TCP_PORT_OML,
					      ipaccess_bts_cb, NULL);
		if (link == NULL) {
			LOGP(DINP, LOGL_ERROR, "cannot create OML "
				"BTS link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_client_link_open(link) < 0) {
			LOGP(DINP, LOGL_ERROR, "cannot open OML BTS link: %s\n",
				strerror(errno));
			ipa_client_link_close(link);
			ipa_client_link_destroy(link);
			return -EIO;
		}
		rsl_link = ipa_client_link_create(tall_ipa_ctx, line,
						  addr, IPA_TCP_PORT_RSL,
						  ipaccess_bts_cb, NULL);
		if (rsl_link == NULL) {
			LOGP(DINP, LOGL_ERROR, "cannot create RSL "
				"BTS link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_client_link_open(rsl_link) < 0) {
			LOGP(DINP, LOGL_ERROR, "cannot open RSL BTS link: %s\n",
				strerror(errno));
			ipa_client_link_close(rsl_link);
			ipa_client_link_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	default:
		break;
	}
	return ret;
}

void e1inp_ipaccess_init(void)
{
	tall_ipa_ctx = talloc_named_const(libosmo_abis_ctx, 1, "ipa");
	e1inp_driver_register(&ipaccess_driver);
}

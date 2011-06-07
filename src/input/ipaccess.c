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

#define PRIV_OML 1
#define PRIV_RSL 2

static void *tall_ipa_ctx;

/* data structure for one E1 interface with A-bis */
struct ia_e1_handle {
	struct osmo_fd listen_fd;
	struct osmo_fd rsl_listen_fd;
	struct gsm_network *gsmnet;
};

static struct ia_e1_handle *e1h;


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

/*
 * read one ipa message from the socket
 * return NULL in case of error
 */
struct msgb *ipaccess_read_msg(struct osmo_fd *bfd, int *error)
{
	struct msgb *msg = msgb_alloc(TS1_ALLOC_SIZE, "Abis/IP");
	struct ipaccess_head *hh;
	int len, ret = 0;

	if (!msg) {
		*error = -ENOMEM;
		return NULL;
	}

	/* first read our 3-byte header */
	hh = (struct ipaccess_head *) msg->data;
	ret = recv(bfd->fd, msg->data, sizeof(*hh), 0);
	if (ret == 0) {
		msgb_free(msg);
		*error = ret;
		return NULL;
	} else if (ret != sizeof(*hh)) {
		if (errno != EAGAIN)
			LOGP(DINP, LOGL_ERROR, "recv error %d %s\n", ret, strerror(errno));
		msgb_free(msg);
		*error = ret;
		return NULL;
	}

	msgb_put(msg, ret);

	/* then read te length as specified in header */
	msg->l2h = msg->data + sizeof(*hh);
	len = ntohs(hh->len);

	if (len < 0 || TS1_ALLOC_SIZE < len + sizeof(*hh)) {
		LOGP(DINP, LOGL_ERROR, "Can not read this packet. %d avail\n", len);
		msgb_free(msg);
		*error = -EIO;
		return NULL;
	}

	ret = recv(bfd->fd, msg->l2h, len, 0);
	if (ret < len) {
		LOGP(DINP, LOGL_ERROR, "short read! Got %d from %d\n", ret, len);
		msgb_free(msg);
		*error = -EIO;
		return NULL;
	}
	msgb_put(msg, ret);

	return msg;
}

static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	int ret = 0, error;

	msg = ipaccess_read_msg(bfd, &error);
	if (!msg) {
		if (e1i_ts->line->rx_err)
			e1i_ts->line->rx_err(error);
		if (error == 0) {
		        osmo_fd_unregister(bfd);
		        close(bfd->fd);
		        bfd->fd = -1;
		}
		return error;
	}
	DEBUGP(DMI, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	/* XXX better use e1inp_ts_rx. */

	if (e1i_ts->line->rx)
		e1i_ts->line->rx(msg, e1i_ts);
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

static int
ipaccess_line_update(struct e1inp_line *line, enum e1inp_line_role role);

struct e1inp_driver ipaccess_driver = {
	.name = "ipa",
	.want_write = ts_want_write,
	.line_update = ipaccess_line_update,
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
	LOGP(DINP, LOGL_NOTICE, "accept()ed new OML link from %s\n",
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

static int rsl_listen_fd_cb(struct osmo_fd *listen_bfd, unsigned int what)
{
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);
	struct osmo_fd *bfd;
	int ret;

	if (!(what & BSC_FD_READ))
		return 0;

	bfd = talloc_zero(tall_ipa_ctx, struct osmo_fd);
	if (!bfd)
		return -ENOMEM;

	/* Some BTS has connected to us, but we don't know yet which line
	 * (as created by the OML link) to associate it with.  Thus, we
	 * allocate a temporary bfd until we have received ID from BTS */

	bfd->fd = accept(listen_bfd->fd, (struct sockaddr *) &sa, &sa_len);
	if (bfd->fd < 0) {
		perror("accept");
		return bfd->fd;
	}
	LOGP(DINP, LOGL_NOTICE, "accept()ed new RSL link from %s\n", inet_ntoa(sa.sin_addr));
	bfd->priv_nr = PRIV_RSL;
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

static int
ipaccess_line_update(struct e1inp_line *line, enum e1inp_line_role role)
{
	int ret = -ENOENT;

	switch(role) {
	case E1INP_LINE_R_BSC:
		/* Listen for OML connections */
		ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
					"0.0.0.0", IPA_TCP_PORT_OML, 1);
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
		/* Listen for RSL connections */
		ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
					"0.0.0.0", IPA_TCP_PORT_RSL, 1);
		if (ret < 0)
			return ret;

		e1h->rsl_listen_fd.fd = ret;
		e1h->rsl_listen_fd.when |= BSC_FD_READ;
		e1h->rsl_listen_fd.cb = rsl_listen_fd_cb;
		e1h->rsl_listen_fd.data = NULL;

		if (osmo_fd_register(&e1h->rsl_listen_fd) < 0) {
			close(ret);
			return ret;
		}
		break;
	case E1INP_LINE_R_BTS:
		/* XXX: no implemented yet. */
		break;
	default:
		break;
	}
	return ret;
}

/* Actively connect to a BTS.  Currently used by ipaccess-config.c */
int ipaccess_connect(struct e1inp_line *line, struct sockaddr_in *sa)
{
	struct e1inp_ts *e1i_ts = &line->ts[0];
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	int ret, on = 1;

	bfd->fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	bfd->cb = ipaccess_fd_cb;
	bfd->when = BSC_FD_READ | BSC_FD_WRITE;
	bfd->data = line;
	bfd->priv_nr = PRIV_OML;

	if (bfd->fd < 0) {
		LOGP(DINP, LOGL_ERROR, "could not create TCP socket.\n");
		return -EIO;
	}

	setsockopt(bfd->fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	ret = connect(bfd->fd, (struct sockaddr *) sa, sizeof(*sa));
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "could not connect socket\n");
		close(bfd->fd);
		return ret;
	}

	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		close(bfd->fd);
		return ret;
	}

	line->driver = &ipaccess_driver;

        return ret;
	//return e1inp_line_register(line);
}

int ipaccess_setup(struct gsm_network *gsmnet)
{
	e1h->gsmnet = gsmnet;
	return 0;
}

void e1inp_ipaccess_init(void)
{
	tall_ipa_ctx = talloc_named_const(libosmo_abis_ctx, 1, "ipa");

	e1h = talloc_zero(tall_ipa_ctx, struct ia_e1_handle);
	if (!e1h)
		return;

	e1inp_driver_register(&ipaccess_driver);
}

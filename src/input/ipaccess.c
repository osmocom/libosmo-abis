/* OpenBSC Abis input driver for ip.access */

/* (C) 2024 by sysmocom - s.f.m.c. GmbH <info@sysmocom.de>
 * (C) 2009-2021 by Harald Welte <laforge@gnumonks.org>
 * (C) 2010 by Holger Hans Peter Freyther
 * (C) 2010 by On-Waves
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
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
#include <stdbool.h>
#include <errno.h>
#include <netinet/tcp.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <osmocom/core/select.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/macaddr.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/backtrace.h>
#include <osmocom/core/stats_tcp.h>
#include <osmocom/core/fsm.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/gsm/protocol/ipaccess.h>
#include <osmocom/gsm/ipa.h>
#include <osmocom/netif/stream.h>
#include <osmocom/netif/ipa.h>
#include <osmocom/abis/e1_input.h>

/* global parameters of IPA input driver */
struct ipa_pars g_e1inp_ipaccess_pars;

static void *tall_ipa_ctx;

#define DEFAULT_TCP_KEEPALIVE_IDLE_TIMEOUT 30
#define DEFAULT_TCP_KEEPALIVE_INTERVAL     3
#define DEFAULT_TCP_KEEPALIVE_RETRY_COUNT  10

struct ipaccess_line {
	bool line_already_initialized;
	struct osmo_stream_cli *ipa_cli[NUM_E1_TS]; /* 0=OML, 1+N=TRX_N */
};

static inline struct e1inp_ts *ipaccess_line_ts(struct osmo_fd *bfd, struct e1inp_line *line)
{
	if (bfd->priv_nr == E1INP_SIGN_OML)
		return e1inp_line_ipa_oml_ts(line);
	else
		return e1inp_line_ipa_rsl_ts(line, bfd->priv_nr - E1INP_SIGN_RSL);
}

static inline void ipaccess_keepalive_fsm_cleanup(struct e1inp_ts *e1i_ts)
{
	struct osmo_ipa_ka_fsm_inst *ka_fsm;

	ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	if (ka_fsm) {
		e1i_ts->driver.ipaccess.ka_fsm = NULL;
		osmo_ipa_ka_fsm_stop(ka_fsm);
		osmo_ipa_ka_fsm_free(ka_fsm);
	}
}

static int ipa_bsc_keepalive_send_cb(struct osmo_ipa_ka_fsm_inst *ka_fi, struct msgb *msg, void *data)
{
	struct osmo_stream_srv *srv = data;
	osmo_stream_srv_send(srv, msg);
	return 0;
}

static int ipa_bsc_keepalive_timeout_cb(struct osmo_ipa_ka_fsm_inst *ka_fi, void *data)
{
	struct osmo_stream_srv *srv = data;
	osmo_stream_srv_destroy(srv);
	return 0;
}

static void ipaccess_bsc_keepalive_fsm_alloc(struct e1inp_ts *e1i_ts, struct osmo_stream_srv *conn, const char *id)
{
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_ipa_ka_fsm_inst *ka_fsm;

	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	if (!line->ipa_kap)
		return;

	ka_fsm = osmo_ipa_ka_fsm_alloc(conn, id);
	e1i_ts->driver.ipaccess.ka_fsm = ka_fsm;
	if (!ka_fsm) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Failed to allocate IPA keepalive FSM\n");
		return;
	}
	osmo_ipa_ka_fsm_set_ping_interval(ka_fsm, line->ipa_kap->interval);
	osmo_ipa_ka_fsm_set_pong_timeout(ka_fsm, line->ipa_kap->wait_for_resp);
	osmo_ipa_ka_fsm_set_data(ka_fsm, conn);
	osmo_ipa_ka_fsm_set_send_cb(ka_fsm, ipa_bsc_keepalive_send_cb);
	osmo_ipa_ka_fsm_set_timeout_cb(ka_fsm, ipa_bsc_keepalive_timeout_cb);
	osmo_ipa_ka_fsm_start(ka_fsm);
}

static int ipa_bts_keepalive_send_cb(struct osmo_ipa_ka_fsm_inst *ka_fi, struct msgb *msg, void *data)
{
	struct osmo_stream_cli *cli = data;
	osmo_stream_cli_send(cli, msg);
	return 0;
}

static void _ipaccess_bts_down_cb(struct osmo_stream_cli *cli)
{
	struct e1inp_ts *e1i_ts = osmo_stream_cli_get_data(cli);
	struct e1inp_line *line = e1i_ts->line;

	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	if (line->ops->sign_link_down)
		line->ops->sign_link_down(line);
}

static int ipa_bts_keepalive_timeout_cb(struct osmo_ipa_ka_fsm_inst *ka_fi, void *data)
{
	struct osmo_stream_cli *cli = data;
	_ipaccess_bts_down_cb(cli);
	return 0;
}

static void ipaccess_bts_keepalive_fsm_alloc(struct e1inp_ts *e1i_ts, struct osmo_stream_cli *client, const char *id)
{
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_ipa_ka_fsm_inst *ka_fsm;

	OSMO_ASSERT(e1i_ts->driver.ipaccess.ka_fsm == NULL);
	if (!line->ipa_kap)
		return;

	ka_fsm = osmo_ipa_ka_fsm_alloc(client, id);
	e1i_ts->driver.ipaccess.ka_fsm = ka_fsm;
	if (!ka_fsm) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Failed to allocate IPA keepalive FSM\n");
		return;
	}
	osmo_ipa_ka_fsm_set_ping_interval(ka_fsm, line->ipa_kap->interval);
	osmo_ipa_ka_fsm_set_pong_timeout(ka_fsm, line->ipa_kap->wait_for_resp);
	osmo_ipa_ka_fsm_set_data(ka_fsm, client);
	osmo_ipa_ka_fsm_set_send_cb(ka_fsm, ipa_bts_keepalive_send_cb);
	osmo_ipa_ka_fsm_set_timeout_cb(ka_fsm, ipa_bts_keepalive_timeout_cb);
}

/* See how ts->num is assigned in e1inp_line_create: line->ts[i].num = i+1;
* As per e1inp_line_ipa_oml_ts(), first TS in line (ts->num=1) is OML.
* As per e1inp_line_ipa_rsl_ts(), second TS in line (ts->num>=2) is RSL.
*/
static inline enum e1inp_sign_type ipaccess_e1i_ts_sign_type(const struct e1inp_ts *e1i_ts)
{
	OSMO_ASSERT(e1i_ts->num != 0);
	if (e1i_ts->num == 1)
		return E1INP_SIGN_OML;
	return E1INP_SIGN_RSL;
}

static inline unsigned int ipaccess_e1i_ts_trx_nr(const struct e1inp_ts *e1i_ts)
{
	enum e1inp_sign_type sign_type = ipaccess_e1i_ts_sign_type(e1i_ts);
	if (sign_type == E1INP_SIGN_OML)
		return 0; /* OML uses trx_nr=0 */
	OSMO_ASSERT(sign_type == E1INP_SIGN_RSL);
	/* e1i_ts->num >= 2: */
	return e1i_ts->num - 2;
}

static inline struct osmo_stream_cli *ipaccess_bts_e1i_ts_stream_cli(const struct e1inp_ts *e1i_ts)
{
	OSMO_ASSERT(e1i_ts);
	struct ipaccess_line *il = e1i_ts->line->driver_data;
	OSMO_ASSERT(il);
	struct osmo_stream_cli *cli = il->ipa_cli[e1i_ts->num - 1];
	OSMO_ASSERT(cli);
	return cli;
}

static inline struct osmo_stream_srv *ipaccess_bts_e1i_ts_stream_srv(const struct e1inp_ts *e1i_ts)
{
	OSMO_ASSERT(e1i_ts);
	struct osmo_stream_srv *conn = e1i_ts->driver.ipaccess.fd.data;
	/* conn may be NULL here, if we are called by user during sign_link_up()
	 * and hence before we finished to internally move the ipaccess conn under
	 * the proper final line/ts object.
	 */
	return conn;
}

static int ipaccess_bsc_write_cb(struct e1inp_ts *e1i_ts);

/* Returns -1 on error, and 0 or 1 on success. If -1 or 1 is returned, line has
 * been released and should not be used anymore by the caller.
 * msg is always freed upon return.
 */
static int ipaccess_bsc_rcvmsg(struct osmo_stream_srv *conn, struct msgb *msg)
{
	struct e1inp_ts *e1i_ts = osmo_stream_srv_get_data(conn);
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	struct tlv_parsed tlvp;
	uint8_t msg_type = *(msg->l2h);
	struct ipaccess_unit unit_data = {};
	struct e1inp_sign_link *sign_link;
	char *unitid;
	int len, ret;
	struct osmo_ipa_ka_fsm_inst *ka_fsm;

	/* peek the pong for our keepalive fsm */
	ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	if (ka_fsm && msg_type == IPAC_MSGT_PONG)
		osmo_ipa_ka_fsm_pong_received(ka_fsm);

	/* Handle IPA PING, PONG and ID_ACK messages. */
	ret = ipa_ccm_rcvmsg_base(msg, bfd);
	switch(ret) {
	case -1:
		/* error in IPA control message handling */
		goto err;
	case 1:
		/* this is an IPA control message, skip further processing */
		goto ret_skip;
	case 0:
		/* this is not an IPA control message, continue */
		break;
	default:
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Unexpected return from ipa_ccm_rcvmsg_base (ret=%d)\n", ret);
		goto err;
	}

	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "ID_RESP ");
		/* parse tags, search for Unit ID */
		ret = ipa_ccm_id_resp_parse(&tlvp, (const uint8_t *)msg->l2h+1, msgb_l2len(msg)-1);
		DEBUGPC(DLMI, "\n");
		if (ret < 0) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "IPA response message with malformed TLVs\n");
			goto err;
		}
		if (!TLVP_PRESENT(&tlvp, IPAC_IDTAG_UNIT)) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "IPA response message without unit ID\n");
			goto err;

		}
		len = TLVP_LEN(&tlvp, IPAC_IDTAG_UNIT);
		if (len < 1) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "IPA response message with too small unit ID\n");
			goto err;
		}
		unitid = (char *) TLVP_VAL(&tlvp, IPAC_IDTAG_UNIT);
		unitid[len - 1] = '\0';
		ret = ipa_parse_unitid(unitid, &unit_data);
		if (ret) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Failed to parse unit ID '%s'\n", unitid);
			goto err;
		}

		if (!line->ops->sign_link_up) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Unable to set signal link, closing socket.\n");
			goto err;
		}
		/* the BSC creates the new sign links at this stage. */
		if (bfd->priv_nr == E1INP_SIGN_OML) {
			sign_link =
				line->ops->sign_link_up(&unit_data, line,
							E1INP_SIGN_OML);
			if (sign_link == NULL) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Unable to set signal link, closing socket.\n");
				goto err;
			}

			ipaccess_bsc_keepalive_fsm_alloc(e1i_ts, conn, "oml_bsc_to_bts");

		} else if (bfd->priv_nr == E1INP_SIGN_RSL) {
			struct e1inp_ts *new_ts;
			struct osmo_fd *newbfd;
			struct e1inp_line *new_line;
			char tcp_stat_name[64];

			sign_link =
				line->ops->sign_link_up(&unit_data, line, E1INP_SIGN_RSL);
			if (sign_link == NULL) {
				LOGPIL(line, DLINP, LOGL_ERROR, "Unable to set signal link, closing socket.\n");
				goto err;
			}
			/* Finally, we know which OML link is associated with
			 * this RSL link, attach it to this socket. */
			new_line = sign_link->ts->line;
			/* this is a bugtrap, the BSC should be using the
			 * virtual E1 line used by OML for this RSL link. */
			if (new_line == line) {
				LOGPIL(line, DLINP, LOGL_ERROR, "Fix your BSC, you should use the "
					"E1 line used by the OML link for your RSL link.\n");
				goto ret_skip;
			}

			/* Attach srv_conn to new line+ts: */
			e1inp_line_get2(new_line, "ipa_bfd");
			new_ts = e1inp_line_ipa_rsl_ts(new_line, unit_data.trx_id);
			newbfd = &new_ts->driver.ipaccess.fd;
			OSMO_ASSERT(newbfd != bfd);
			osmo_fd_setup(newbfd, bfd->fd, 0, NULL, conn, E1INP_SIGN_RSL + unit_data.trx_id);
			osmo_stream_srv_set_data(conn, new_ts);

			/* Now we can release the old temporary dummy RSL line+ts: */
			osmo_stats_tcp_osmo_fd_unregister(bfd);
			bfd->fd = -1;
			/* conn holds a reference to old line, drop it */
			OSMO_ASSERT(bfd->data == conn);
			bfd->data = NULL;
			e1inp_line_put2(line, "ipa_bfd");

			snprintf(tcp_stat_name, sizeof(tcp_stat_name), "site.%u.bts.%u.ipa-rsl.%u",
				unit_data.site_id, unit_data.bts_id, unit_data.trx_id);
			osmo_stats_tcp_osmo_fd_register(newbfd, tcp_stat_name);

			ipaccess_bsc_keepalive_fsm_alloc(new_ts, conn, "rsl_bsc_to_bts");

			/* Now that we attached the srv_conn to the e1ts, transmit packets which may been enqueued
			 * by user during sign_link_up() and which were delayed due to no srv_conn found: */
			ipaccess_bsc_write_cb(new_ts);
			/* Conn is not freed here, so we need to free msgb. */
			msgb_free(msg);
			return 1;
		}
		break;
	default:
		LOGPIL(line, DLINP, LOGL_ERROR, "Unknown IPA message type\n");
		goto err;
	}

ret_skip:
	msgb_free(msg);
	return 0;
err:
	msgb_free(msg);
	osmo_stream_srv_destroy(conn);
	return -1;
}

static void ipaccess_close(struct e1inp_sign_link *sign_link)
{
	struct e1inp_ts *e1i_ts = sign_link->ts;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_ipa_ka_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	struct osmo_stream_cli *cli;
	struct osmo_stream_srv *conn;

	/* depending on caller the fsm might be dead */
	if (ka_fsm)
		osmo_ipa_ka_fsm_stop(ka_fsm);

	e1inp_int_snd_event(e1i_ts, sign_link, S_L_INP_TEI_DN);
	/* the first e1inp_sign_link_destroy call closes the socket. */

	OSMO_ASSERT(line->ops);
	switch (line->ops->cfg.ipa.role) {
	case E1INP_LINE_R_BTS:
		cli = ipaccess_bts_e1i_ts_stream_cli(e1i_ts);
		osmo_stream_cli_close(cli);
		bfd->fd = -1; /* Compatibility with older implementations */
		break;
	case E1INP_LINE_R_BSC:
		/* conn may be NULL here, if we are called by user during sign_link_up()
		 * callback and hence before we finished to internally move the ipaccess conn
		 * under the proper final line/ts object:
		 */
		conn = ipaccess_bts_e1i_ts_stream_srv(e1i_ts);
		if (conn) {
			osmo_stream_srv_destroy(conn);
		} else {
			LOGPITS(e1i_ts, DLINP, LOGL_DEBUG,
				"ipaccess_close() on ts with no srv_conn, "
				"probably called during sign_link_up() or sign_link_down() user cb.\n");
		}
		break;
	default:
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR,
			"ipaccess_close() for line with unknown role %d\n",
			line->ops->cfg.ipa.role);
		break;
	}
}

static bool e1i_ts_has_pending_tx_msgs(struct e1inp_ts *e1i_ts)
{
	struct e1inp_sign_link *link;
	llist_for_each_entry(link, &e1i_ts->sign.sign_links, list) {
		if (!llist_empty(&link->tx_list)) {
			return true;
		}
	}
	return false;
}

static int ipaccess_bts_send_msg(struct e1inp_ts *e1i_ts,
				 struct e1inp_sign_link *sign_link,
				 struct osmo_stream_cli *cli,
				 struct msgb *msg)
{
	switch (sign_link->type) {
	case E1INP_SIGN_OML:
	case E1INP_SIGN_RSL:
	case E1INP_SIGN_OSMO:
		break;
	default:
		msgb_free(msg);
		return -EINVAL;
	}

	msg->l2h = msg->data;
	ipa_prepend_header(msg, sign_link->tei);

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "TX: %s\n", osmo_hexdump(msg->l2h, msgb_l2len(msg)));
	osmo_stream_cli_send(cli, msg);
	return 0;
}

/* msg was enqueued in sign_link->tx_list.
 * Pop it from that list, submit it to osmo_stream_cli. */
static int ipaccess_bts_write_cb(struct e1inp_ts *e1i_ts)
{
	int rc = 0;
	struct osmo_stream_cli *cli = ipaccess_bts_e1i_ts_stream_cli(e1i_ts);

	/* get the next msg for this timeslot */
	while (e1i_ts_has_pending_tx_msgs(e1i_ts)) {
		struct e1inp_sign_link *sign_link = NULL;
		struct msgb *msg;
		msg = e1inp_tx_ts(e1i_ts, &sign_link);
		OSMO_ASSERT(msg);
		OSMO_ASSERT(sign_link);
		rc |= ipaccess_bts_send_msg(e1i_ts, sign_link, cli, msg);
	}
	return rc;
}

static int ipaccess_bsc_send_msg(struct e1inp_ts *e1i_ts,
				 struct e1inp_sign_link *sign_link,
				 struct osmo_stream_srv *conn,
				 struct msgb *msg)
{
	switch (sign_link->type) {
	case E1INP_SIGN_OML:
	case E1INP_SIGN_RSL:
	case E1INP_SIGN_OSMO:
		break;
	default:
		LOGPITS(e1i_ts, DLMI, LOGL_NOTICE, "Drop Tx msg for unknown sign_link type %d: %s\n",
			sign_link->type, osmo_hexdump(msg->data, msgb_length(msg)));
		msgb_free(msg);
		return -EINVAL;
	}

	msg->l2h = msg->data;
	ipa_prepend_header(msg, sign_link->tei);

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "TX: %s\n", osmo_hexdump(msg->l2h, msgb_l2len(msg)));
	osmo_stream_srv_send(conn, msg);
	return 0;
}

/* msg was enqueued in sign_link->tx_list.
 * Pop it from that list, submit it to osmo_stream_srv. */
static int ipaccess_bsc_write_cb(struct e1inp_ts *e1i_ts)
{
	int rc = 0;
	struct osmo_stream_srv *srv = ipaccess_bts_e1i_ts_stream_srv(e1i_ts);

	/* conn may be NULL here, if we are called by user during sign_link_up()
	 * ballback and hence before we finished to internally move the ipaccess conn
	 * under the proper final line/ts object:
	 */
	if (!srv) {
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "Delaying Tx on ts with no srv_conn\n");
		return -ENODEV;
	}

	/* get the next msg for this timeslot */
	while (e1i_ts_has_pending_tx_msgs(e1i_ts)) {
		struct e1inp_sign_link *sign_link = NULL;
		struct msgb *msg;
		msg = e1inp_tx_ts(e1i_ts, &sign_link);
		OSMO_ASSERT(msg);
		OSMO_ASSERT(sign_link);
		rc |= ipaccess_bsc_send_msg(e1i_ts, sign_link, srv, msg);
	}
	return rc;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	enum e1inp_line_role role = E1INP_LINE_R_NONE;
	/* osmo-bts handover_test crashes here because has ops = NULL (doesn't
	 * call e1inp_line_bind_ops())... keep old behavior compatible. */
	OSMO_ASSERT(e1i_ts->line);
	if (e1i_ts->line->ops)
		role = e1i_ts->line->ops->cfg.ipa.role;

	switch (role) {
	case E1INP_LINE_R_BTS:
		/* msg was enqueued in sign_link->tx_list.
		 * Pop it from that list, submit it to osmo_stream_cli: */
		return ipaccess_bts_write_cb(e1i_ts);
	case E1INP_LINE_R_BSC:
		/* msg was enqueued in sign_link->tx_list.
		 * Pop it from that list, submit it to osmo_stream_cli: */
		return ipaccess_bsc_write_cb(e1i_ts);
	case E1INP_LINE_R_NONE:
	default:
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "want_write for ts with unknown role %d!\n", role);
		return 0;
	}
}

static int ipaccess_bsc_conn_read_cb(struct osmo_stream_srv *conn, int res, struct msgb *msg)
{
	enum ipaccess_proto ipa_proto = osmo_ipa_msgb_cb_proto(msg);
	struct e1inp_ts *e1i_ts = osmo_stream_srv_get_data(conn);
	struct e1inp_sign_link *link;
	int ret, rc;

	if (res <= 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "failed reading from socket: %d\n", res);
		goto err;
	}

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "RX: %s\n", osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	if (ipa_proto == IPAC_PROTO_IPACCESS) {
		ret = ipaccess_bsc_rcvmsg(conn, msg);
		/* BIG FAT WARNING: bfd might no longer exist here (ret != 0),
		 * since ipaccess_rcvmsg() might have free'd it !!! */
		return ret != 0 ? -EBADF : 0;
	}
	if (e1i_ts->type == E1INP_TS_TYPE_NONE) {
		/* this sign link is not know yet.. complain. */
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Timeslot is not configured.\n");
		goto err;
	}

	link = e1inp_lookup_sign_link(e1i_ts, ipa_proto, 0);
	if (!link) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "no matching signalling link for ipa_proto=0x%02x\n", ipa_proto);
		goto err;
	}
	msg->dst = link;

	/* XXX better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops->sign_link) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Fix your application, no action set for signalling messages.\n");
		goto err;
	}
	rc = e1i_ts->line->ops->sign_link(msg);
	if (rc < 0) {
		/* Don't close the signalling link if the upper layers report
		 * an error, that's too strict. BTW, the signalling layer is
		 * resposible for releasing the message.
		 */
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Bad signalling message, sign_link returned error: %s.\n",
			strerror(-rc));
	}

	return rc;
err:
	msgb_free(msg);
	osmo_stream_srv_destroy(conn);
	return 0;
}

static int ipaccess_bsc_conn_closed_cb(struct osmo_stream_srv *conn)
{
	struct e1inp_ts *e1i_ts = osmo_stream_srv_get_data(conn);
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	e1inp_line_get2(line, __func__);
	e1inp_line_put2(line, "ipa_bfd");

	osmo_stats_tcp_osmo_fd_unregister(bfd);
	ipaccess_keepalive_fsm_cleanup(e1i_ts);

	osmo_stream_srv_set_data(conn, NULL);
	bfd->fd = -1;
	bfd->data = NULL;

	if (line->ops->sign_link_down)
		line->ops->sign_link_down(line);
	e1inp_line_put2(line, __func__);
	return 0;
}

static void update_fd_settings(struct e1inp_line *line, int fd)
{
	int ret;
	int val, idle_val, interval_val, retry_count_val, user_timeout_val;

	if (line->keepalive_num_probes) {
		/* Enable TCP keepalive to find out if the connection is gone */
		val = 1;
		ret = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
		if (ret < 0)
			LOGPIL(line, DLINP, LOGL_ERROR, "Failed to enable TCP keepalive: %s\n", strerror(errno));
		else
			LOGPIL(line, DLINP, LOGL_NOTICE, "TCP Keepalive is enabled\n");

		idle_val = line->keepalive_idle_timeout > 0 ?
				line->keepalive_idle_timeout :
				DEFAULT_TCP_KEEPALIVE_IDLE_TIMEOUT;
		interval_val = line->keepalive_probe_interval > -1 ?
				line->keepalive_probe_interval :
				DEFAULT_TCP_KEEPALIVE_INTERVAL;
		retry_count_val = line->keepalive_num_probes > 0 ?
				line->keepalive_num_probes :
				DEFAULT_TCP_KEEPALIVE_RETRY_COUNT;
		user_timeout_val = 1000 * retry_count_val * (interval_val + idle_val);
		LOGPIL(line, DLINP, LOGL_NOTICE, "TCP keepalive idle_timeout=%us, interval=%us, retry_count=%u "
			"user_timeout=%ums\n", idle_val, interval_val, retry_count_val, user_timeout_val);
		/* The following options are not portable! */
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_val, sizeof(idle_val));
		if (ret < 0) {
			LOGPIL(line, DLINP, LOGL_ERROR, "Failed to set TCP keepalive idle time: %s\n",
			       strerror(errno));
		}
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval_val, sizeof(interval_val));
		if (ret < 0) {
			LOGPIL(line, DLINP, LOGL_ERROR, "Failed to set TCP keepalive interval: %s\n",
			       strerror(errno));
		}
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &retry_count_val, sizeof(retry_count_val));
		if (ret < 0)
			LOGPIL(line, DLINP, LOGL_ERROR, "Failed to set TCP keepalive count: %s\n", strerror(errno));
		ret = setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &user_timeout_val, sizeof(user_timeout_val));
		if (ret < 0)
			LOGPIL(line, DLINP, LOGL_ERROR, "Failed to set TCP user timeout: %s\n", strerror(errno));
	}

	val = 1;
	ret = setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val));
	if (ret < 0)
		LOGPIL(line, DLINP, LOGL_ERROR, "Failed to set TCP_NODELAY: %s\n", strerror(errno));
}

/* callback of the OML listening filedescriptor */
static int ipaccess_bsc_oml_accept_cb(struct osmo_stream_srv_link *link, int fd)
{
	int ret;
	int i;
	struct e1inp_line *srv_link_line = osmo_stream_srv_link_get_data(link);
	struct e1inp_line *line;
	struct osmo_stream_srv *conn;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;
	char conn_name[128];

	/* clone virtual E1 line for this new OML link. */
	line = e1inp_line_clone(tall_ipa_ctx, srv_link_line, "ipa_bfd");
	if (line == NULL) {
		LOGP(DLINP, LOGL_ERROR, "could not clone E1 line\n");
		return -ENOMEM;
	}

	/* create virrtual E1 timeslots for signalling */
	e1inp_ts_config_sign(e1inp_line_ipa_oml_ts(line), line);

	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	e1i_ts = e1inp_line_ipa_oml_ts(line);

	snprintf(conn_name, sizeof(conn_name), "ts-%u-%u-oml", line->num, e1i_ts->num);
	conn = osmo_stream_srv_create2(link, link, fd, e1i_ts);
	OSMO_ASSERT(conn);
	osmo_stream_srv_set_name(conn, conn_name);
	osmo_stream_srv_set_read_cb(conn, ipaccess_bsc_conn_read_cb);
	osmo_stream_srv_set_closed_cb(conn, ipaccess_bsc_conn_closed_cb);
	osmo_stream_srv_set_segmentation_cb(conn, osmo_ipa_segmentation_cb);

	/* We use bfd->fd in here for osmo_stats_tcp, and bfd->data to access osmo_stream_srv from e1i_ts. */
	bfd = &e1i_ts->driver.ipaccess.fd;
	osmo_fd_setup(bfd, fd, 0, NULL, conn, E1INP_SIGN_OML);
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-oml");

	update_fd_settings(line, fd);

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(fd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n", strerror(errno));
		goto err_socket;
	}
	return 0;

err_socket:
	osmo_stream_srv_destroy(conn);
	return 0;
}

static int ipaccess_bsc_rsl_accept_cb(struct osmo_stream_srv_link *link, int fd)
{
	struct e1inp_line *srv_link_line = osmo_stream_srv_link_get_data(link);
	struct e1inp_line *line;
	struct osmo_stream_srv *conn;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;
	int i, ret;
	char conn_name[128];

	/* We don't know yet which OML link to associate it with. Thus, we
	 * allocate a temporary E1 line until we have received ID. */
	line = e1inp_line_clone(tall_ipa_ctx, srv_link_line, "ipa_bfd");
	if (line == NULL) {
		LOGP(DLINP, LOGL_ERROR, "could not clone E1 line\n");
		return -ENOMEM;
	}
	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	/* we need to initialize this in case to avoid crashes in case
	 * that the socket is closed before we've seen an ID_RESP. */
	e1inp_ts_config_sign(e1inp_line_ipa_oml_ts(line), line);

	e1i_ts = e1inp_line_ipa_rsl_ts(line, 0);

	snprintf(conn_name, sizeof(conn_name), "ts-%u-%u-rsl", line->num, e1i_ts->num);
	conn = osmo_stream_srv_create2(link, link, fd, e1i_ts);
	OSMO_ASSERT(conn);
	osmo_stream_srv_set_name(conn, conn_name);
	osmo_stream_srv_set_read_cb(conn, ipaccess_bsc_conn_read_cb);
	osmo_stream_srv_set_closed_cb(conn, ipaccess_bsc_conn_closed_cb);
	osmo_stream_srv_set_segmentation_cb(conn, osmo_ipa_segmentation_cb);

	/* We use bfd->fd in here for osmo_stats_tcp, and bfd->data to access osmo_stream_srv from e1i_ts. */
	bfd = &e1i_ts->driver.ipaccess.fd;
	osmo_fd_setup(bfd, fd, 0, NULL, conn, E1INP_SIGN_RSL);
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-rsl");

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(fd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n", strerror(errno));
		goto err_socket;
	}
	update_fd_settings(line, fd);
	return 0;

err_socket:
	osmo_stream_srv_destroy(conn);
	return 0;
}

static struct msgb *ipa_bts_id_ack(void)
{
	struct msgb *nmsg2;
	nmsg2 = ipa_msg_alloc(0);
	if (!nmsg2)
		return NULL;
	msgb_v_put(nmsg2, IPAC_MSGT_ID_ACK);
	ipa_prepend_header(nmsg2, IPAC_PROTO_IPACCESS);
	return nmsg2;
}

/* Same as ipaccess_bts_handle_ccm(), but using an osmo_stream_cli as backend
 * instead of ipa_client_conn.
 * The old ipaccess_bts_handle_ccm() needs to be kept as it's a public API. */
static int _ipaccess_bts_handle_ccm(struct osmo_stream_cli *cli,
				    struct ipaccess_unit *dev, struct msgb *msg)
{
	/* special handling for IPA CCM. */
	if (osmo_ipa_msgb_cb_proto(msg) != IPAC_PROTO_IPACCESS)
		return 0;

	int ret = 0;
	const uint8_t *data = msgb_l2(msg);
	int len = msgb_l2len(msg);
	OSMO_ASSERT(len > 0);
	uint8_t msg_type = *data;
	struct e1inp_ts *e1i_ts = osmo_stream_cli_get_data(cli);
	/* line might not exist if != bsc||bts */
	struct e1inp_line *line = e1i_ts->line;

	/* peek the pong for our keepalive fsm */
	if (line && msg_type == IPAC_MSGT_PONG) {
		struct osmo_ipa_ka_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
		osmo_ipa_ka_fsm_pong_received(ka_fsm);
	}

	/* ping, pong and acknowledgment cases. */
	struct osmo_fd tmp_ofd = { .fd = osmo_stream_cli_get_fd(cli) };
	OSMO_ASSERT(tmp_ofd.fd >= 0);
	ret = ipa_ccm_rcvmsg_bts_base(msg, &tmp_ofd);
	if (ret < 0)
		goto err;

	/* this is a request for identification from the BSC. */
	if (msg_type == IPAC_MSGT_ID_GET) {
		struct msgb *rmsg;
		/* The ipaccess_unit dev holds generic identity for the whole
		 * line, hence no trx_id. Patch ipaccess_unit during call to
		 * ipa_ccm_make_id_resp_from_req() to identify this TRX: */
		int store_trx_nr = dev->trx_id;
		dev->trx_id = ipaccess_e1i_ts_trx_nr(e1i_ts);
		LOGP(DLINP, LOGL_NOTICE, "received ID_GET for unit ID %u/%u/%u\n",
		     dev->site_id, dev->bts_id, dev->trx_id);
		rmsg = ipa_ccm_make_id_resp_from_req(dev, data + 1, len - 1);
		dev->trx_id = store_trx_nr;
		if (!rmsg) {
			LOGP(DLINP, LOGL_ERROR, "Failed parsing ID_GET message.\n");
			goto err;
		}
		osmo_stream_cli_send(cli, rmsg);

		/* send ID_ACK. */
		rmsg = ipa_bts_id_ack();
		if (!rmsg) {
			LOGP(DLINP, LOGL_ERROR, "Failed allocating ID_ACK message.\n");
			goto err;
		}
		osmo_stream_cli_send(cli, rmsg);
	}
	return 1;

err:
	return -1;
}

static int ipaccess_bts_read_cb(struct osmo_stream_cli *cli, int res, struct msgb *msg)
{
	enum ipaccess_proto ipa_proto = osmo_ipa_msgb_cb_proto(msg);
	struct e1inp_ts *e1i_ts = osmo_stream_cli_get_data(cli);
	struct e1inp_line *line = e1i_ts->line;
	struct e1inp_sign_link *sign_link;
	int ret;

	if (res <= 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "failed reading from socket: %d\n", res);
		goto err;
	}

	/* special handling for IPA CCM. */
	if (ipa_proto == IPAC_PROTO_IPACCESS) {
		uint8_t msg_type = *(msg->l2h);
		/* this is a request for identification from the BSC. */
		if (msg_type == IPAC_MSGT_ID_GET) {
			if (!line->ops->sign_link_up) {
				LOGPITS(e1i_ts, DLINP, LOGL_NOTICE,
					"Unable to set signal link, closing socket.\n");
				goto err;
			}
		}
	}

	/* core CCM handling */
	ret = _ipaccess_bts_handle_ccm(cli, line->ops->cfg.ipa.dev, msg);
	if (ret < 0)
		goto err;

	if (ret == 1 && ipa_proto == IPAC_PROTO_IPACCESS) {
		uint8_t msg_type = *(msg->l2h);
		if (msg_type == IPAC_MSGT_ID_GET) {
			enum e1inp_sign_type sign_type = ipaccess_e1i_ts_sign_type(e1i_ts);
			unsigned int trx_nr = ipaccess_e1i_ts_trx_nr(e1i_ts);
			sign_link = line->ops->sign_link_up(line->ops->cfg.ipa.dev,
							    line, sign_type + trx_nr);
			if (sign_link == NULL) {
				LOGPITS(e1i_ts, DLINP, LOGL_NOTICE,
					"Unable to set signal link, closing socket.\n");
				goto err;
			}
		}
		msgb_free(msg);
		return ret;
	}

	/* look up for some existing signaling link. */
	sign_link = e1inp_lookup_sign_link(e1i_ts, ipa_proto, 0);
	if (sign_link == NULL) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "no matching signalling link for "
			"ipa_proto=0x%02x\n", ipa_proto);
		goto err;
	}
	msg->dst = sign_link;

	/* XXX better use e1inp_ts_rx? */
	if (!line->ops->sign_link) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		goto err;
	}
	return line->ops->sign_link(msg);

err:
	msgb_free(msg);
	osmo_stream_cli_close(cli);
	return -EBADF;
}

static int ipaccess_bts_connect_cb(struct osmo_stream_cli *cli)
{
	struct e1inp_ts *e1i_ts = osmo_stream_cli_get_data(cli);
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_ipa_ka_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;

	update_fd_settings(line, osmo_stream_cli_get_fd(cli));
	if (ka_fsm && line->ipa_kap)
		osmo_ipa_ka_fsm_start(ka_fsm);
	return 0;
}

static int ipaccess_bts_disconnect_cb(struct osmo_stream_cli *cli)
{
	_ipaccess_bts_down_cb(cli);
	return 0;
}

static int ipaccess_line_update(struct e1inp_line *line)
{
	int ret = -ENOENT;
	struct ipaccess_line *il;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct ipaccess_line);

	if (!line->driver_data) {
		LOGPIL(line, DLINP, LOGL_ERROR, "ipaccess: OOM in line update\n");
		return -ENOMEM;
	}
	il = line->driver_data;

	switch(line->ops->cfg.ipa.role) {
	case E1INP_LINE_R_BSC: {
		/* We only initialize this line once. */
		if (il->line_already_initialized)
			return 0;
		struct osmo_stream_srv_link *oml_link, *rsl_link;
		const char *ipa = e1inp_ipa_get_bind_addr();

		LOGPIL(line, DLINP, LOGL_NOTICE, "enabling ipaccess BSC mode on %s "
		       "with OML %u and RSL %u TCP ports\n", ipa, IPA_TCP_PORT_OML, IPA_TCP_PORT_RSL);

		oml_link = osmo_stream_srv_link_create(tall_ipa_ctx);
		OSMO_ASSERT(oml_link);
		osmo_stream_srv_link_set_proto(oml_link, IPPROTO_TCP);
		osmo_stream_srv_link_set_addr(oml_link, ipa);
		osmo_stream_srv_link_set_port(oml_link, IPA_TCP_PORT_OML);
		osmo_stream_srv_link_set_data(oml_link, line);
		osmo_stream_srv_link_set_nodelay(oml_link, true);
		osmo_stream_srv_link_set_priority(oml_link, g_e1inp_ipaccess_pars.oml.dscp);
		osmo_stream_srv_link_set_ip_dscp(oml_link, g_e1inp_ipaccess_pars.oml.priority);
		osmo_stream_srv_link_set_accept_cb(oml_link, ipaccess_bsc_oml_accept_cb);

		if (osmo_stream_srv_link_open(oml_link)) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot open OML BTS link %s:%u (%s)\n",
			       ipa, IPA_TCP_PORT_OML, strerror(errno));
			osmo_stream_srv_link_destroy(oml_link);
			return -EIO;
		}

		rsl_link = osmo_stream_srv_link_create(tall_ipa_ctx);
		OSMO_ASSERT(rsl_link);
		osmo_stream_srv_link_set_proto(rsl_link, IPPROTO_TCP);
		osmo_stream_srv_link_set_addr(rsl_link, ipa);
		osmo_stream_srv_link_set_port(rsl_link, IPA_TCP_PORT_RSL);
		osmo_stream_srv_link_set_data(rsl_link, line);
		osmo_stream_srv_link_set_nodelay(rsl_link, true);
		osmo_stream_srv_link_set_priority(rsl_link, g_e1inp_ipaccess_pars.rsl.dscp);
		osmo_stream_srv_link_set_ip_dscp(rsl_link, g_e1inp_ipaccess_pars.rsl.priority);
		osmo_stream_srv_link_set_accept_cb(rsl_link, ipaccess_bsc_rsl_accept_cb);

		if (osmo_stream_srv_link_open(rsl_link)) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot open RSL BTS link %s:%u (%s)\n",
			       ipa, IPA_TCP_PORT_RSL, strerror(errno));
			osmo_stream_srv_link_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	case E1INP_LINE_R_BTS: {
		struct osmo_stream_cli *cli;
		struct e1inp_ts *e1i_ts = e1inp_line_ipa_oml_ts(line);
		char cli_name[128];

		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "enabling ipaccess BTS mode, "
			"OML connecting to %s:%u\n", line->ops->cfg.ipa.addr, IPA_TCP_PORT_OML);

		/* Drop previous line */
		if (il->ipa_cli[0]) {
			osmo_stream_cli_close(il->ipa_cli[0]);
			ipaccess_keepalive_fsm_cleanup(e1i_ts);
			e1i_ts->driver.ipaccess.fd.fd = -1;
			osmo_stream_cli_destroy(il->ipa_cli[0]);
			il->ipa_cli[0] = NULL;
		}

		e1inp_ts_config_sign(e1i_ts, line);

		cli = osmo_stream_cli_create(tall_ipa_ctx);
		OSMO_ASSERT(cli);

		snprintf(cli_name, sizeof(cli_name), "ts-%u-%u-oml", line->num, e1i_ts->num);
		osmo_stream_cli_set_name(cli, cli_name);
		osmo_stream_cli_set_data(cli, e1i_ts);
		osmo_stream_cli_set_addr(cli, line->ops->cfg.ipa.addr);
		osmo_stream_cli_set_port(cli, IPA_TCP_PORT_OML);
		osmo_stream_cli_set_proto(cli, IPPROTO_TCP);
		osmo_stream_cli_set_nodelay(cli, true);
		osmo_stream_cli_set_priority(cli, g_e1inp_ipaccess_pars.oml.dscp);
		osmo_stream_cli_set_ip_dscp(cli, g_e1inp_ipaccess_pars.oml.priority);

		/* Reconnect is handled by upper layers: */
		osmo_stream_cli_set_reconnect_timeout(cli, -1);

		osmo_stream_cli_set_segmentation_cb(cli, osmo_ipa_segmentation_cb);
		osmo_stream_cli_set_connect_cb(cli, ipaccess_bts_connect_cb);
		osmo_stream_cli_set_disconnect_cb(cli, ipaccess_bts_disconnect_cb);
		osmo_stream_cli_set_read_cb2(cli, ipaccess_bts_read_cb);

		if (osmo_stream_cli_open(cli)) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "cannot open OML BTS link: %s\n", strerror(errno));
			osmo_stream_cli_destroy(cli);
			return -EIO;
		}

		/* Compatibility with older ofd based implementation. osmo-bts accesses
		 * this fd directly in get_signlink_remote_ip() and get_rsl_local_ip() */
		e1i_ts->driver.ipaccess.fd.fd = osmo_stream_cli_get_fd(cli);

		ipaccess_bts_keepalive_fsm_alloc(e1i_ts, cli, "oml_bts_to_bsc");
		il->ipa_cli[0] = cli;
		ret = 0;
		break;
	}
	default:
		break;
	}

	il->line_already_initialized = true;
	return ret;
}

/* backwards compatibility */
int e1inp_ipa_bts_rsl_connect(struct e1inp_line *line,
			      const char *rem_addr, uint16_t rem_port)
{
	return e1inp_ipa_bts_rsl_connect_n(line, rem_addr, rem_port, 0);
}

int e1inp_ipa_bts_rsl_connect_n(struct e1inp_line *line,
				const char *rem_addr, uint16_t rem_port,
				uint8_t trx_nr)
{
	struct osmo_stream_cli *cli;
	struct e1inp_ts *e1i_ts = e1inp_line_ipa_rsl_ts(line, trx_nr);
	struct ipaccess_line *il;
	char cli_name[128];
	int rc;

	if (E1INP_SIGN_RSL+trx_nr-1 >= NUM_E1_TS) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "cannot create RSL BTS link: "
			"trx_nr (%d) out of range\n", trx_nr);
		return -EINVAL;
	}

	/* Drop previous line */
	if ((rc = e1inp_ipa_bts_rsl_close_n(line, trx_nr)) < 0)
		return rc;

	e1inp_ts_config_sign(e1i_ts, line);

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct ipaccess_line);
	il = line->driver_data;

	cli = osmo_stream_cli_create(tall_ipa_ctx);
	OSMO_ASSERT(cli);

	snprintf(cli_name, sizeof(cli_name), "ts-%u-%u-rsl-trx%u",
		 line->num, e1i_ts->num, trx_nr);
	osmo_stream_cli_set_name(cli, cli_name);
	osmo_stream_cli_set_data(cli, e1i_ts);
	osmo_stream_cli_set_addr(cli, rem_addr);
	osmo_stream_cli_set_port(cli, rem_port);
	osmo_stream_cli_set_proto(cli, IPPROTO_TCP);
	osmo_stream_cli_set_nodelay(cli, true);
	osmo_stream_cli_set_priority(cli, g_e1inp_ipaccess_pars.rsl.dscp);
	osmo_stream_cli_set_ip_dscp(cli, g_e1inp_ipaccess_pars.rsl.priority);

	/* Reconnect is handled by upper layers: */
	osmo_stream_cli_set_reconnect_timeout(cli, -1);

	osmo_stream_cli_set_segmentation_cb(cli, osmo_ipa_segmentation_cb);
	osmo_stream_cli_set_connect_cb(cli, ipaccess_bts_connect_cb);
	osmo_stream_cli_set_disconnect_cb(cli, ipaccess_bts_disconnect_cb);
	osmo_stream_cli_set_read_cb2(cli, ipaccess_bts_read_cb);

	if (osmo_stream_cli_open(cli)) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "cannot open RSL BTS link: %s\n", strerror(errno));
		osmo_stream_cli_destroy(cli);
		return -EIO;
	}

	/* Compatibility with older ofd based implementation. osmo-bts accesses
	 * this fd directly in get_signlink_remote_ip() and get_rsl_local_ip() */
	e1i_ts->driver.ipaccess.fd.fd = osmo_stream_cli_get_fd(cli);

	ipaccess_bts_keepalive_fsm_alloc(e1i_ts, cli, "rsl_bts_to_bsc");
	il->ipa_cli[1 + trx_nr] = cli;
	return 0;
}

/* Close the underlying IPA TCP socket of an RSL link */
int e1inp_ipa_bts_rsl_close_n(struct e1inp_line *line, uint8_t trx_nr)
{
	struct osmo_stream_cli *cli;
	struct ipaccess_line *il;
	struct e1inp_ts *e1i_ts;

	if (E1INP_SIGN_RSL+trx_nr-1 >= NUM_E1_TS) {
		LOGPIL(line, DLINP, LOGL_ERROR, "cannot close RSL BTS link: trx_nr (%d) out of range\n", trx_nr);
		return -EINVAL;
	}

	il = line->driver_data;
	if (!il)
		return 0; /* Nothing to do, no lines created */

	e1i_ts = e1inp_line_ipa_rsl_ts(line, trx_nr);
	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	/* Compatibility with older implementation: */
	e1i_ts->driver.ipaccess.fd.fd = -1;

	cli = il->ipa_cli[1 + trx_nr];
	if (cli != NULL) {
		osmo_stream_cli_destroy(cli);
		il->ipa_cli[1 + trx_nr] = NULL;
	}
	return 0;
}

static struct e1inp_driver ipaccess_driver = {
	.name = "ipa",
	.want_write = ts_want_write,
	.line_update = ipaccess_line_update,
	.close = ipaccess_close,
	.default_delay = 0,
	.has_keepalive = 1,
};

void e1inp_ipaccess_init(void)
{
	tall_ipa_ctx = talloc_named_const(libosmo_abis_ctx, 1, "ipa");
	e1inp_driver_register(&ipaccess_driver);
}

void e1inp_ipa_set_bind_addr(const char *ip_bind_addr)
{
	talloc_free((char*)ipaccess_driver.bind_addr);
	ipaccess_driver.bind_addr = NULL;

	if (ip_bind_addr)
		ipaccess_driver.bind_addr = talloc_strdup(tall_ipa_ctx,
							  ip_bind_addr);
}

const char *e1inp_ipa_get_bind_addr(void)
{
	return ipaccess_driver.bind_addr?
		ipaccess_driver.bind_addr
		: "0.0.0.0";
}

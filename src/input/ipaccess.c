/* OpenBSC Abis input driver for ip.access */

/* (C) 2009-2021 by Harald Welte <laforge@gnumonks.org>
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
#include <osmocom/gsm/tlv.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/macaddr.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>
#include <osmocom/abis/ipa.h>
#include <osmocom/core/backtrace.h>
#include <osmocom/gsm/ipa.h>
#include <osmocom/core/stats_tcp.h>
#include <osmocom/core/fsm.h>
#include <osmocom/netif/stream.h>
#include <osmocom/netif/ipa.h>

/* global parameters of IPA input driver */
struct ipa_pars g_e1inp_ipaccess_pars;

static void *tall_ipa_ctx;

#define TS1_ALLOC_SIZE	900

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
	struct osmo_fsm_inst *ka_fsm;

	ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	if (ka_fsm) {
		osmo_fsm_inst_term(ka_fsm, OSMO_FSM_TERM_REQUEST, NULL);
		e1i_ts->driver.ipaccess.ka_fsm = NULL;
	}
}

static int ipaccess_drop(struct osmo_fd *bfd, struct e1inp_line *line)
{
	int ret = 1;
	struct e1inp_ts *e1i_ts = ipaccess_line_ts(bfd, line);
	e1inp_line_get2(line, __func__);

	ipaccess_keepalive_fsm_cleanup(e1i_ts);

	/* Error case: we did not see any ID_RESP yet for this socket. */
	if (bfd->fd != -1) {
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "Forcing socket shutdown\n");
		osmo_fd_unregister(bfd);
		close(bfd->fd);
		bfd->fd = -1;
		switch (line->ops->cfg.ipa.role) {
		case E1INP_LINE_R_BSC:
			/* This is BSC code, ipaccess_drop() is only called for
			   accepted() sockets, hence the bfd holds a reference to
			   e1inp_line in ->data that needs to be released */
			OSMO_ASSERT(bfd->data == line);
			bfd->data = NULL;
			e1inp_line_put2(line, "ipa_bfd");
			break;
		case E1INP_LINE_R_BTS:
			/* BTS code: bfd->data contains pointer to struct
			 * ipa_client_conn. Leave it alive so it reconnects.
			 */
			break;
		default:
			break;
		}
		ret = -ENOENT;
	} else {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR,
			"Forcing socket shutdown with no signal link set\n");
	}

	msgb_free(e1i_ts->pending_msg);
	e1i_ts->pending_msg = NULL;

	/* e1inp_sign_link_destroy releases the socket descriptors for us. */
	if (line->ops->sign_link_down)
		line->ops->sign_link_down(line);

	e1inp_line_put2(line, __func__);
	return ret;
}

static void ipa_bsc_keepalive_write_server_cb(struct osmo_fsm_inst *fi, void *conn, struct msgb *msg)
{
	struct osmo_fd *bfd = (struct osmo_fd *)conn;
	write(bfd->fd, msg->data, msg->len);
	msgb_free(msg);
}

static int ipa_bsc_keepalive_timeout_cb(struct osmo_fsm_inst *fi, void *data)
{
	struct osmo_fd *bfd = (struct osmo_fd *)data;

	if (bfd->fd == -1)
		return 1;

	ipaccess_drop(bfd, (struct e1inp_line *)bfd->data);
	return 1;
}

static void ipaccess_bsc_keepalive_fsm_alloc(struct e1inp_ts *e1i_ts, struct osmo_fd *bfd, const char *id)
{
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fsm_inst *ka_fsm;

	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	if (!line->ipa_kap)
		return;

	ka_fsm = ipa_generic_conn_alloc_keepalive_fsm(tall_ipa_ctx, bfd, line->ipa_kap, id);
	e1i_ts->driver.ipaccess.ka_fsm = ka_fsm;
	if (!ka_fsm) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Failed to allocate IPA keepalive FSM\n");
		return;
	}

	ipa_keepalive_fsm_set_timeout_cb(ka_fsm, ipa_bsc_keepalive_timeout_cb);
	ipa_keepalive_fsm_set_send_cb(ka_fsm, ipa_bsc_keepalive_write_server_cb);
	ipa_keepalive_fsm_start(ka_fsm);
}

static void ipa_bts_keepalive_write_client_cb(struct osmo_fsm_inst *fi, void *conn, struct msgb *msg) {
	struct osmo_stream_cli *cli = (struct osmo_stream_cli *)conn;
	osmo_stream_cli_send(cli, msg);
}

static void _ipaccess_bts_down_cb(struct osmo_stream_cli *cli)
{
	struct e1inp_ts *e1i_ts = osmo_stream_cli_get_data(cli);
	struct e1inp_line *line = e1i_ts->line;

	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	if (line->ops->sign_link_down)
		line->ops->sign_link_down(line);
}

static int ipa_bts_keepalive_timeout_cb(struct osmo_fsm_inst *fi, void *conn) {
	struct osmo_stream_cli *cli = (struct osmo_stream_cli *)conn;
	_ipaccess_bts_down_cb(cli);
	return 1;
}

static void ipaccess_bts_keepalive_fsm_alloc(struct e1inp_ts *e1i_ts, struct osmo_stream_cli *client, const char *id)
{
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fsm_inst *ka_fsm;

	OSMO_ASSERT(e1i_ts->driver.ipaccess.ka_fsm == NULL);
	if (!line->ipa_kap)
		return;

	ka_fsm = ipa_generic_conn_alloc_keepalive_fsm(client, client, line->ipa_kap, id);
	e1i_ts->driver.ipaccess.ka_fsm = ka_fsm;
	if (!ka_fsm) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Failed to allocate IPA keepalive FSM\n");
		return;
	}

	ipa_keepalive_fsm_set_timeout_cb(ka_fsm, ipa_bts_keepalive_timeout_cb);
	ipa_keepalive_fsm_set_send_cb(ka_fsm, ipa_bts_keepalive_write_client_cb);
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

/* Returns -1 on error, and 0 or 1 on success. If -1 or 1 is returned, line has
 * been released and should not be used anymore by the caller. */
static int ipaccess_rcvmsg(struct e1inp_line *line, struct msgb *msg,
			   struct osmo_fd *bfd)
{
	struct tlv_parsed tlvp;
	uint8_t msg_type = *(msg->l2h);
	struct ipaccess_unit unit_data = {};
	struct e1inp_sign_link *sign_link;
	char *unitid;
	int len, ret;
	struct e1inp_ts *e1i_ts;
	struct osmo_fsm_inst *ka_fsm;

	/* peek the pong for our keepalive fsm */
	e1i_ts = ipaccess_line_ts(bfd, line);
	ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	if (ka_fsm && msg_type == IPAC_MSGT_PONG)
		ipa_keepalive_fsm_pong_received(ka_fsm);

	/* Handle IPA PING, PONG and ID_ACK messages. */
	ret = ipa_ccm_rcvmsg_base(msg, bfd);
	switch(ret) {
	case -1:
		/* error in IPA control message handling */
		goto err;
	case 1:
		/* this is an IPA control message, skip further processing */
		return 0;
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

			ipaccess_bsc_keepalive_fsm_alloc(e1i_ts, bfd, "oml_bsc_to_bts");

		} else if (bfd->priv_nr == E1INP_SIGN_RSL) {
			struct e1inp_ts *ts;
			struct osmo_fd *newbfd;
			struct e1inp_line *new_line;
			char tcp_stat_name[64];

			sign_link =
				line->ops->sign_link_up(&unit_data, line,
							E1INP_SIGN_RSL);
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
				return 0;
			}
			e1inp_line_get2(new_line, "ipa_bfd");
			ts = e1inp_line_ipa_rsl_ts(new_line, unit_data.trx_id);
			newbfd = &ts->driver.ipaccess.fd;
			OSMO_ASSERT(newbfd != bfd);

			/* preserve 'newbfd->when' flags potentially set by sign_link_up() */
			osmo_fd_setup(newbfd, bfd->fd, newbfd->when | bfd->when, bfd->cb,
				      new_line, E1INP_SIGN_RSL + unit_data.trx_id);


			/* now we can release the dummy RSL line (old temporary bfd). */
			osmo_fd_unregister(bfd);
			bfd->fd = -1;
			/* bfd->data holds a reference to line, drop it */
			OSMO_ASSERT(bfd->data == line);
			bfd->data = NULL;
			e1inp_line_put2(line, "ipa_bfd");

			ret = osmo_fd_register(newbfd);
			if (ret < 0) {
				LOGPITS(ts, DLINP, LOGL_ERROR, "could not register FD\n");
				goto err;
			}
			snprintf(tcp_stat_name, sizeof(tcp_stat_name), "site.%u.bts.%u.ipa-rsl.%u",
				unit_data.site_id, unit_data.bts_id, unit_data.trx_id);
			osmo_stats_tcp_osmo_fd_register(newbfd, tcp_stat_name);

			e1i_ts = ipaccess_line_ts(newbfd, new_line);
			ipaccess_bsc_keepalive_fsm_alloc(e1i_ts, newbfd, "rsl_bsc_to_bts");
			return 1;
		}
		break;
	default:
		LOGPIL(line, DLINP, LOGL_ERROR, "Unknown IPA message type\n");
		goto err;
	}
	return 0;
err:
	if (bfd->fd != -1) {
		osmo_fd_unregister(bfd);
		close(bfd->fd);
		bfd->fd = -1;
		/* This is a BSC accepted socket, bfd->data holds a reference to line, drop it */
		OSMO_ASSERT(bfd->data == line);
		bfd->data = NULL;
		e1inp_line_put2(line, "ipa_bfd");
	}
	return -1;
}

/* Returns -EBADF if bfd cannot be used by the caller anymore after return. */
static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts;
	struct e1inp_sign_link *link;
	struct ipaccess_head *hh;
	struct msgb *msg = NULL;
	int ret, rc;

	e1i_ts = ipaccess_line_ts(bfd, line);
	ret = ipa_msg_recv_buffered(bfd->fd, &msg, &e1i_ts->pending_msg);
	if (ret < 0) {
		if (ret == -EAGAIN)
			return 0;
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "Sign link problems, closing socket. Reason: %s\n",
			strerror(-ret));
		goto err;
	} else if (ret == 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "Sign link vanished, dead socket\n");
		goto err;
	}
	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	hh = (struct ipaccess_head *) msg->data;
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		ret = ipaccess_rcvmsg(line, msg, bfd);
		/* BIG FAT WARNING: bfd might no longer exist here (ret != 0),
		 * since ipaccess_rcvmsg() might have free'd it !!! */
		msgb_free(msg);
		return ret != 0 ? -EBADF : 0;
	} else if (e1i_ts->type == E1INP_TS_TYPE_NONE) {
		/* this sign link is not know yet.. complain. */
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Timeslot is not configured.\n");
		goto err_msg;
	}

	link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (!link) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "no matching signalling link for hh->proto=0x%02x\n", hh->proto);
		goto err_msg;
	}
	msg->dst = link;

	/* XXX better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops->sign_link) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Fix your application, no action set for signalling messages.\n");
		goto err_msg;
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
err_msg:
	msgb_free(msg);
err:
	ipaccess_drop(bfd, line);
	return -EBADF;
}

static void ipaccess_close(struct e1inp_sign_link *sign_link)
{
	struct e1inp_ts *e1i_ts = sign_link->ts;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
	struct osmo_stream_cli *cli;

	/* depending on caller the fsm might be dead */
	if (ka_fsm)
		ipa_keepalive_fsm_stop(ka_fsm);

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
	default:
		if (bfd->fd != -1) {
			osmo_fd_unregister(bfd);
			close(bfd->fd);
			bfd->fd = -1;
			/* If The bfd holds a reference to e1inp_line in ->data (BSC
			* accepted() sockets), then release it */
			if (bfd->data == line) {
				bfd->data = NULL;
				e1inp_line_put2(line, "ipa_bfd");
			}
		}
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
		rc |= ipaccess_bts_send_msg(e1i_ts, sign_link, cli, msg);
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
	case E1INP_LINE_R_NONE:
	case E1INP_LINE_R_BSC:
	default:
		osmo_fd_write_enable(&e1i_ts->driver.ipaccess.fd);
		/* ipaccess_fd_cb will be called from main loop and tx the msgb. */
		return 0;
	}
}

static void timeout_ts1_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	if (e1i_ts_has_pending_tx_msgs(e1i_ts))
		ts_want_write(e1i_ts);
}

static int __handle_ts1_write(struct osmo_fd *bfd, struct e1inp_line *line)
{
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts;
	struct e1inp_sign_link *sign_link;
	struct msgb *msg;
	int ret;

	e1i_ts = ipaccess_line_ts(bfd, line);

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		osmo_fd_write_disable(bfd);
		return 0;
	}

	switch (sign_link->type) {
	case E1INP_SIGN_OML:
	case E1INP_SIGN_RSL:
	case E1INP_SIGN_OSMO:
		break;
	default:
		/* leave WRITE flag enabled, come back for more msg */
		ret = -EINVAL;
		goto out;
	}

	msg->l2h = msg->data;
	ipa_prepend_header(msg, sign_link->tei);

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "TX %u: %s\n", ts_nr,
		osmo_hexdump(msg->l2h, msgb_l2len(msg)));

	ret = send(bfd->fd, msg->data, msg->len, 0);
	if (ret != msg->len) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "failed to send A-bis IPA signalling "
			"message. Reason: %s\n", strerror(errno));
		goto err;
	}

	/* this is some ancient code that apparently exists to slow down writes towards
	 * some even more ancient nanoBTS 900 units. See git commit
	 * d49fc5ae24fc9d44d2b284392ab619cc7a69a876 of openbsc.git (now osmo-bsc.git) */
	if (e1i_ts->sign.delay) {
		osmo_fd_write_disable(bfd);
		/* set tx delay timer for next event */
		osmo_timer_setup(&e1i_ts->sign.tx_timer, timeout_ts1_write, e1i_ts);
		osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, e1i_ts->sign.delay);
	} else {
out:
		if (!e1i_ts_has_pending_tx_msgs(e1i_ts))
			osmo_fd_write_disable(bfd);
	}
	msgb_free(msg);
	return ret;
err:
	ipaccess_drop(bfd, line);
	msgb_free(msg);
	return ret;
}

static int handle_ts1_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;

	return __handle_ts1_write(bfd, line);
}


/* callback from select.c in case one of the fd's can be read/written */
int ipaccess_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
	int rc = 0;

	if (what & OSMO_FD_READ)
		rc = handle_ts1_read(bfd);
	if (rc != -EBADF && (what & OSMO_FD_WRITE))
		rc = handle_ts1_write(bfd);

	return rc;
}

static int ipaccess_line_update(struct e1inp_line *line);

struct e1inp_driver ipaccess_driver = {
	.name = "ipa",
	.want_write = ts_want_write,
	.line_update = ipaccess_line_update,
	.close = ipaccess_close,
	.default_delay = 0,
	.has_keepalive = 1,
};

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
static int ipaccess_bsc_oml_cb(struct ipa_server_link *link, int fd)
{
	int ret;
	int i;
	struct e1inp_line *line;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;

	/* clone virtual E1 line for this new OML link. */
	line = e1inp_line_clone(tall_ipa_ctx, link->line, "ipa_bfd");
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

	bfd = &e1i_ts->driver.ipaccess.fd;
	osmo_fd_setup(bfd, fd, OSMO_FD_READ, ipaccess_fd_cb, line, E1INP_SIGN_OML);
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-oml");

	update_fd_settings(line, bfd->fd);

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n", strerror(errno));
		goto err_socket;
	}
	return ret;

err_socket:
	osmo_fd_unregister(bfd);
err_line:
	close(bfd->fd);
	bfd->fd = -1;
	bfd->data = NULL;
	e1inp_line_put2(line, "ipa_bfd");
	return ret;
}

static int ipaccess_bsc_rsl_cb(struct ipa_server_link *link, int fd)
{
	struct e1inp_line *line;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;
	int i, ret;

	/* We don't know yet which OML link to associate it with. Thus, we
	 * allocate a temporary E1 line until we have received ID. */
	line = e1inp_line_clone(tall_ipa_ctx, link->line, "ipa_bfd");
	if (line == NULL) {
		LOGP(DLINP, LOGL_ERROR, "could not clone E1 line\n");
		return -ENOMEM;
	}
	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	/* we need this to initialize this in case to avoid crashes in case
	 * that the socket is closed before we've seen an ID_RESP. */
	e1inp_ts_config_sign(e1inp_line_ipa_oml_ts(line), line);

	e1i_ts = e1inp_line_ipa_rsl_ts(line, 0);

	bfd = &e1i_ts->driver.ipaccess.fd;
	osmo_fd_setup(bfd, fd, OSMO_FD_READ, ipaccess_fd_cb, line, E1INP_SIGN_RSL);
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-rsl");

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n", strerror(errno));
		goto err_socket;
	}
	update_fd_settings(line, bfd->fd);
	return ret;

err_socket:
	osmo_fd_unregister(bfd);
err_line:
	close(bfd->fd);
	bfd->fd = -1;
	bfd->data = NULL;
	e1inp_line_put2(line, "ipa_bfd");
	return ret;
}

int ipaccess_bts_handle_ccm(struct ipa_client_conn *link,
			    struct ipaccess_unit *dev, struct msgb *msg)
{
	struct ipaccess_head *hh = (struct ipaccess_head *) msg->data;

	/* special handling for IPA CCM. */
	if (hh->proto != IPAC_PROTO_IPACCESS)
		return 0;

	int ret = 0;
	uint8_t *data = msgb_l2(msg);
	int len = msgb_l2len(msg);
	OSMO_ASSERT(len > 0);
	uint8_t msg_type = *data;
	/* line might not exist if != bsc||bts */
	struct e1inp_line *line = link->line;

	/* peek the pong for our keepalive fsm */
	if (line && msg_type == IPAC_MSGT_PONG) {
		struct osmo_fsm_inst *ka_fsm = ipaccess_line_ts(link->ofd, line)->driver.ipaccess.ka_fsm;
		ipa_keepalive_fsm_pong_received(ka_fsm);
	}

	/* ping, pong and acknowledgment cases. */
	ret = ipa_ccm_rcvmsg_bts_base(msg, link->ofd);
	if (ret < 0)
		goto err;

	/* this is a request for identification from the BSC. */
	if (msg_type == IPAC_MSGT_ID_GET) {
		struct msgb *rmsg;
		/* The ipaccess_unit dev holds generic identity for the whole
		 * line, hence no trx_id. Patch ipaccess_unit during call to
		 * ipa_ccm_make_id_resp_from_req() to identify this TRX: */
		int store_trx_nr = dev->trx_id;
		if (link->ofd->priv_nr >= E1INP_SIGN_RSL)
			dev->trx_id = link->ofd->priv_nr - E1INP_SIGN_RSL;
		else
			dev->trx_id = 0;
		LOGP(DLINP, LOGL_NOTICE, "received ID_GET for unit ID %u/%u/%u\n",
		     dev->site_id, dev->bts_id, dev->trx_id);
		rmsg = ipa_ccm_make_id_resp_from_req(dev, data + 1, len - 1);
		dev->trx_id = store_trx_nr;
		if (!rmsg) {
			LOGP(DLINP, LOGL_ERROR, "Failed parsing ID_GET message.\n");
			goto err;
		}

		ret = ipa_send(link->ofd->fd, rmsg->data, rmsg->len);
		if (ret != rmsg->len) {
			LOGP(DLINP, LOGL_ERROR, "cannot send ID_RESP message. Reason: %s\n",
				strerror(errno));
			msgb_free(rmsg);
			goto err;
		}
		msgb_free(rmsg);

		/* send ID_ACK. */
		ret = ipa_ccm_send_id_ack(link->ofd->fd);
		if (ret <= 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot send ID_ACK message. Reason: %s\n",
				strerror(errno));
			goto err;
		}
	}
	return 1;

err:
	ipa_client_conn_close(link);
	return -1;
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
		struct osmo_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;
		ipa_keepalive_fsm_pong_received(ka_fsm);
	}

	/* ping, pong and acknowledgment cases. */
	struct osmo_fd tmp_ofd = { .fd = osmo_stream_cli_get_fd(cli) };
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
	struct osmo_fsm_inst *ka_fsm = e1i_ts->driver.ipaccess.ka_fsm;

	update_fd_settings(line, osmo_stream_cli_get_fd(cli));
	if (ka_fsm && line->ipa_kap)
		ipa_keepalive_fsm_start(ka_fsm);
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
		struct ipa_server_link *oml_link, *rsl_link;
		const char *ipa = e1inp_ipa_get_bind_addr();

		LOGPIL(line, DLINP, LOGL_NOTICE, "enabling ipaccess BSC mode on %s "
		       "with OML %u and RSL %u TCP ports\n", ipa, IPA_TCP_PORT_OML, IPA_TCP_PORT_RSL);

		oml_link = ipa_server_link_create(tall_ipa_ctx, line, ipa,
						  IPA_TCP_PORT_OML,
						  ipaccess_bsc_oml_cb, NULL);
		if (oml_link == NULL) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot create OML BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		oml_link->dscp = g_e1inp_ipaccess_pars.oml.dscp;
		oml_link->priority = g_e1inp_ipaccess_pars.oml.priority;
		if (ipa_server_link_open(oml_link) < 0) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot open OML BSC link: %s\n", strerror(errno));
			ipa_server_link_destroy(oml_link);
			return -EIO;
		}
		rsl_link = ipa_server_link_create(tall_ipa_ctx, line, ipa,
						  IPA_TCP_PORT_RSL,
						  ipaccess_bsc_rsl_cb, NULL);
		if (rsl_link == NULL) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot create RSL BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		rsl_link->dscp = g_e1inp_ipaccess_pars.rsl.dscp;
		rsl_link->priority = g_e1inp_ipaccess_pars.rsl.priority;
		if (ipa_server_link_open(rsl_link) < 0) {
			LOGPIL(line, DLINP, LOGL_ERROR, "cannot open RSL BSC link: %s\n", strerror(errno));
			ipa_server_link_destroy(rsl_link);
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

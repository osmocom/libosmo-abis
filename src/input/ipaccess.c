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

/* global parameters of IPA input driver */
struct ipa_pars g_e1inp_ipaccess_pars;

static void *tall_ipa_ctx;

#define TS1_ALLOC_SIZE	900

#define DEFAULT_TCP_KEEPALIVE_IDLE_TIMEOUT 30
#define DEFAULT_TCP_KEEPALIVE_INTERVAL     3
#define DEFAULT_TCP_KEEPALIVE_RETRY_COUNT  10

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
		ipa_keepalive_fsm_stop(ka_fsm);
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
	if (!ka_fsm)
		return;

	ipa_keepalive_fsm_set_timeout_cb(ka_fsm, ipa_bsc_keepalive_timeout_cb);
	ipa_keepalive_fsm_set_send_cb(ka_fsm, ipa_bsc_keepalive_write_server_cb);
	ipa_keepalive_fsm_start(ka_fsm);
}

static void ipa_bts_keepalive_write_client_cb(struct osmo_fsm_inst *fi, void *conn, struct msgb *msg) {
	struct ipa_client_conn *link = (struct ipa_client_conn *)conn;
	int ret = 0;

	ret = ipa_send(link->ofd->fd, msg->data, msg->len);
	if (ret != msg->len) {
		LOGP(DLINP, LOGL_ERROR, "cannot send message. Reason: %s\n", strerror(errno));
	}
	msgb_free(msg);
}

static void update_fd_settings(struct e1inp_line *line, int fd);
static void ipaccess_bts_updown_cb(struct ipa_client_conn *link, int up);

static int ipa_bts_keepalive_timeout_cb(struct osmo_fsm_inst *fi, void *conn) {
	ipaccess_bts_updown_cb(conn, false);
	return 1;
}

static void ipaccess_bts_keepalive_fsm_alloc(struct e1inp_ts *e1i_ts, struct ipa_client_conn *client, const char *id)
{
	struct e1inp_line *line = e1i_ts->line;
	struct osmo_fsm_inst *ka_fsm;

	ipaccess_keepalive_fsm_cleanup(e1i_ts);
	if (!line->ipa_kap)
		return;

	ka_fsm = ipa_client_conn_alloc_keepalive_fsm(client, line->ipa_kap, id);
	e1i_ts->driver.ipaccess.ka_fsm = ka_fsm;
	if (!ka_fsm)
		return;

	ipa_keepalive_fsm_set_timeout_cb(ka_fsm, ipa_bts_keepalive_timeout_cb);
	ipa_keepalive_fsm_set_send_cb(ka_fsm, ipa_bts_keepalive_write_client_cb);
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
		LOGP(DLINP, LOGL_ERROR, "Unexpected return from "
					"ipa_ccm_rcvmsg_base "
					"(ret=%d)\n", ret);
		goto err;
	}

	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		DEBUGP(DLMI, "ID_RESP ");
		/* parse tags, search for Unit ID */
		ret = ipa_ccm_id_resp_parse(&tlvp, (const uint8_t *)msg->l2h+1, msgb_l2len(msg)-1);
		DEBUGPC(DLMI, "\n");
		if (ret < 0) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"with malformed TLVs\n");
			goto err;
		}
		if (!TLVP_PRESENT(&tlvp, IPAC_IDTAG_UNIT)) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"without unit ID\n");
			goto err;

		}
		len = TLVP_LEN(&tlvp, IPAC_IDTAG_UNIT);
		if (len < 1) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"with too small unit ID\n");
			goto err;
		}
		unitid = (char *) TLVP_VAL(&tlvp, IPAC_IDTAG_UNIT);
		unitid[len - 1] = '\0';
		ret = ipa_parse_unitid(unitid, &unit_data);
		if (ret) {
			LOGP(DLINP, LOGL_ERROR, "Failed to parse unit ID '%s'\n", unitid);
			goto err;
		}

		if (!line->ops->sign_link_up) {
			LOGP(DLINP, LOGL_ERROR,
			     "Unable to set signal link, closing socket.\n");
			goto err;
		}
		/* the BSC creates the new sign links at this stage. */
		if (bfd->priv_nr == E1INP_SIGN_OML) {
			sign_link =
				line->ops->sign_link_up(&unit_data, line,
							E1INP_SIGN_OML);
			if (sign_link == NULL) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
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
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				goto err;
			}
			/* Finally, we know which OML link is associated with
			 * this RSL link, attach it to this socket. */
			new_line = sign_link->ts->line;
			/* this is a bugtrap, the BSC should be using the
			 * virtual E1 line used by OML for this RSL link. */
			if (new_line == line) {
				LOGP(DLINP, LOGL_ERROR,
					"Fix your BSC, you should use the "
					"E1 line used by the OML link for "
					"your RSL link.\n");
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
				LOGP(DLINP, LOGL_ERROR,
				     "could not register FD\n");
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
		LOGP(DLINP, LOGL_ERROR, "Unknown IPA message type\n");
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
		LOGP(DLINP, LOGL_NOTICE, "Sign link problems, "
			"closing socket. Reason: %s\n", strerror(-ret));
		goto err;
	} else if (ret == 0) {
		LOGP(DLINP, LOGL_NOTICE, "Sign link vanished, dead socket\n");
		goto err;
	}
	DEBUGP(DLMI, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	hh = (struct ipaccess_head *) msg->data;
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		ret = ipaccess_rcvmsg(line, msg, bfd);
		/* BIG FAT WARNING: bfd might no longer exist here (ret != 0),
		 * since ipaccess_rcvmsg() might have free'd it !!! */
		msgb_free(msg);
		return ret != 0 ? -EBADF : 0;
	} else if (e1i_ts->type == E1INP_TS_TYPE_NONE) {
		/* this sign link is not know yet.. complain. */
		LOGP(DLINP, LOGL_ERROR, "Timeslot is not configured.\n");
		goto err_msg;
	}

	link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (!link) {
		LOGP(DLINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		goto err_msg;
	}
	msg->dst = link;

	/* XXX better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops->sign_link) {
		LOGP(DLINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		goto err_msg;
	}
	rc = e1i_ts->line->ops->sign_link(msg);
	if (rc < 0) {
		/* Don't close the signalling link if the upper layers report
		 * an error, that's too strict. BTW, the signalling layer is
		 * resposible for releasing the message.
		 */
		LOGP(DLINP, LOGL_ERROR, "Bad signalling message,"
		     " sign_link returned error: %s.\n", strerror(-rc));
	}

	return rc;
err_msg:
	msgb_free(msg);
err:
	ipaccess_drop(bfd, line);
	return -EBADF;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	osmo_fd_write_enable(&e1i_ts->driver.ipaccess.fd);

	return 0;
}

static void ipaccess_close(struct e1inp_sign_link *sign_link)
{
	struct e1inp_ts *e1i_ts = sign_link->ts;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	struct e1inp_line *line = e1i_ts->line;

	/* line might not exist if != bsc||bts */
	if (line) {
		/* depending on caller the fsm might be dead */
		struct osmo_fsm_inst* ka_fsm = ipaccess_line_ts(bfd, line)->driver.ipaccess.ka_fsm;
		if (ka_fsm)
			ipa_keepalive_fsm_stop(ka_fsm);

	}

	e1inp_int_snd_event(e1i_ts, sign_link, S_L_INP_TEI_DN);
	/* the first e1inp_sign_link_destroy call closes the socket. */
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

static int ipaccess_bts_write_cb(struct ipa_client_conn *link)
{
	struct e1inp_line *line = link->line;

	return __handle_ts1_write(link->ofd, line);
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
	int val;

	if (line->keepalive_num_probes) {
		/* Enable TCP keepalive to find out if the connection is gone */
		val = 1;
		ret = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val));
		if (ret < 0)
			LOGP(DLINP, LOGL_NOTICE, "Failed to set keepalive: %s\n",
			     strerror(errno));
		else
			LOGP(DLINP, LOGL_NOTICE, "Keepalive is set: %i\n", ret);

#if defined(TCP_KEEPIDLE) && defined(TCP_KEEPINTVL) && defined(TCP_KEEPCNT)
		/* The following options are not portable! */
		val = line->keepalive_idle_timeout > 0 ?
			line->keepalive_idle_timeout :
			DEFAULT_TCP_KEEPALIVE_IDLE_TIMEOUT;
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,
				 &val, sizeof(val));
		if (ret < 0)
			LOGP(DLINP, LOGL_NOTICE,
			     "Failed to set keepalive idle time: %s\n",
			     strerror(errno));
		val = line->keepalive_probe_interval > -1 ?
			line->keepalive_probe_interval :
			DEFAULT_TCP_KEEPALIVE_INTERVAL;
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL,
				 &val, sizeof(val));
		if (ret < 0)
			LOGP(DLINP, LOGL_NOTICE,
			     "Failed to set keepalive interval: %s\n",
			     strerror(errno));
		val = line->keepalive_num_probes > 0 ?
			line->keepalive_num_probes :
			DEFAULT_TCP_KEEPALIVE_RETRY_COUNT;
		ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,
				 &val, sizeof(val));
		if (ret < 0)
			LOGP(DLINP, LOGL_NOTICE,
			     "Failed to set keepalive count: %s\n",
			     strerror(errno));
#if defined(TCP_USER_TIMEOUT)
                val = 1000 * line->keepalive_num_probes *
                        line->keepalive_probe_interval +
                        line->keepalive_idle_timeout;
                ret = setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT,
                                 &val, sizeof(val));
                if (ret < 0)
                        LOGP(DLINP, LOGL_NOTICE,
                             "Failed to set user timoeut: %s\n",
                             strerror(errno));
#endif
#endif
	}

	val = 1;
	ret = setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val));
	if (ret < 0)
		LOGP(DLINP, LOGL_ERROR, "Failed to set TCP_NODELAY: %s\n", strerror(errno));
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
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-oml");

	update_fd_settings(line, bfd->fd);

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n",
			strerror(errno));
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
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}
	osmo_stats_tcp_osmo_fd_register(bfd, "ipa-rsl");

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipa_ccm_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n",
			strerror(errno));
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

#define IPA_STRING_MAX 64

static struct msgb *
ipa_bts_id_resp(const struct ipaccess_unit *dev, uint8_t *data, int len, int trx_nr)
{
	struct msgb *nmsg;
	char str[IPA_STRING_MAX];
	uint8_t *tag;

	memset(str, 0, sizeof(str));

	nmsg = ipa_msg_alloc(0);
	if (!nmsg)
		return NULL;

	*msgb_put(nmsg, 1) = IPAC_MSGT_ID_RESP;
	while (len) {
		if (len < 2) {
			LOGP(DLINP, LOGL_NOTICE,
				"Short read of ipaccess tag\n");
			msgb_free(nmsg);
			return NULL;
		}
		switch (data[1]) {
		case IPAC_IDTAG_UNIT:
			snprintf(str, sizeof(str), "%u/%u/%u",
				dev->site_id, dev->bts_id, trx_nr);
			break;
		case IPAC_IDTAG_MACADDR:
			snprintf(str, sizeof(str),
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 dev->mac_addr[0], dev->mac_addr[1],
				 dev->mac_addr[2], dev->mac_addr[3],
				 dev->mac_addr[4], dev->mac_addr[5]);
			break;
		case IPAC_IDTAG_LOCATION1:
			if (dev->location1)
				osmo_strlcpy(str, dev->location1, sizeof(str));
			break;
		case IPAC_IDTAG_LOCATION2:
			if (dev->location2)
				osmo_strlcpy(str, dev->location2, sizeof(str));
			break;
		case IPAC_IDTAG_EQUIPVERS:
			if (dev->equipvers)
				osmo_strlcpy(str, dev->equipvers, sizeof(str));
			break;
		case IPAC_IDTAG_SWVERSION:
			if (dev->swversion)
				osmo_strlcpy(str, dev->swversion, sizeof(str));
			break;
		case IPAC_IDTAG_UNITNAME:
			snprintf(str, sizeof(str),
				 "%s-%02x-%02x-%02x-%02x-%02x-%02x",
				 dev->unit_name,
				 dev->mac_addr[0], dev->mac_addr[1],
				 dev->mac_addr[2], dev->mac_addr[3],
				 dev->mac_addr[4], dev->mac_addr[5]);
			break;
		case IPAC_IDTAG_SERNR:
			if (dev->serno)
				osmo_strlcpy(str, dev->serno, sizeof(str));
			break;
		default:
			LOGP(DLINP, LOGL_NOTICE,
				"Unknown ipaccess tag 0x%02x\n", *data);
			msgb_free(nmsg);
			return NULL;
		}

		LOGP(DLINP, LOGL_INFO, " tag %d: %s\n", data[1], str);
		tag = msgb_put(nmsg, 3 + strlen(str) + 1);
		tag[0] = 0x00;
		tag[1] = 1 + strlen(str) + 1;
		tag[2] = data[1];
		memcpy(tag + 3, str, strlen(str) + 1);
		data += 2;
		len -= 2;
	}
	ipa_msg_push_header(nmsg, IPAC_PROTO_IPACCESS);
	return nmsg;
}

static struct msgb *ipa_bts_id_ack(void)
{
	struct msgb *nmsg2;

	nmsg2 = ipa_msg_alloc(0);
	if (!nmsg2)
		return NULL;

	*msgb_put(nmsg2, 1) = IPAC_MSGT_ID_ACK;
	ipa_msg_push_header(nmsg2, IPAC_PROTO_IPACCESS);

	return nmsg2;
}

static void ipaccess_bts_updown_cb(struct ipa_client_conn *link, int up)
{
	struct e1inp_line *line = link->line;

	if (up) {
		struct osmo_fsm_inst *ka_fsm = ipaccess_line_ts(link->ofd, line)->driver.ipaccess.ka_fsm;

		update_fd_settings(line, link->ofd->fd);
		if (ka_fsm && line->ipa_kap)
			ipa_keepalive_fsm_start(ka_fsm);
		return;
	}

	if (line->ops->sign_link_down)
		line->ops->sign_link_down(line);
}

/* handle incoming message to BTS, check if it is an IPA CCM, and if yes,
 * handle it accordingly (PING/PONG/ID_REQ/ID_RESP/ID_ACK) */
int ipaccess_bts_handle_ccm(struct ipa_client_conn *link,
			    struct ipaccess_unit *dev, struct msgb *msg)
{
	struct ipaccess_head *hh = (struct ipaccess_head *) msg->data;
	struct msgb *rmsg;
	int ret = 0;
	/* line might not exist if != bsc||bts */
	struct e1inp_line *line = link->line;

	/* special handling for IPA CCM. */
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		uint8_t msg_type = *(msg->l2h);
		struct osmo_fsm_inst* ka_fsm = NULL;

		/* peek the pong for our keepalive fsm */
		if (line && msg_type == IPAC_MSGT_PONG) {
			ka_fsm = ipaccess_line_ts(link->ofd, line)->driver.ipaccess.ka_fsm;
			ipa_keepalive_fsm_pong_received(ka_fsm);
		}

		/* ping, pong and acknowledgment cases. */
		ret = ipa_ccm_rcvmsg_bts_base(msg, link->ofd);
		if (ret < 0)
			goto err;

		/* this is a request for identification from the BSC. */
		if (msg_type == IPAC_MSGT_ID_GET) {
			uint8_t *data = msgb_l2(msg);
			int len = msgb_l2len(msg);
			int trx_nr = 0;

			if (link->ofd->priv_nr >= E1INP_SIGN_RSL)
				trx_nr = link->ofd->priv_nr - E1INP_SIGN_RSL;

			LOGP(DLINP, LOGL_NOTICE, "received ID_GET for unit ID %u/%u/%u\n",
			     dev->site_id, dev->bts_id, trx_nr);
			rmsg = ipa_bts_id_resp(dev, data + 1, len - 1, trx_nr);
			ret = ipa_send(link->ofd->fd, rmsg->data, rmsg->len);
			if (ret != rmsg->len) {
				LOGP(DLINP, LOGL_ERROR, "cannot send ID_RESP "
				     "message. Reason: %s\n", strerror(errno));
				goto err_rmsg;
			}
			msgb_free(rmsg);

			/* send ID_ACK. */
			rmsg = ipa_bts_id_ack();
			ret = ipa_send(link->ofd->fd, rmsg->data, rmsg->len);
			if (ret != rmsg->len) {
				LOGP(DLINP, LOGL_ERROR, "cannot send ID_ACK "
				     "message. Reason: %s\n", strerror(errno));
				goto err_rmsg;
			}
			msgb_free(rmsg);
		}
		return 1;
	}

	return 0;

err_rmsg:
	msgb_free(rmsg);
err:
	ipa_client_conn_close(link);
	return -1;
}

static int ipaccess_bts_read_cb(struct ipa_client_conn *link, struct msgb *msg)
{
	struct ipaccess_head *hh = (struct ipaccess_head *) msg->data;
	struct e1inp_ts *e1i_ts = NULL;
	struct e1inp_sign_link *sign_link;
	uint8_t msg_type = *(msg->l2h);
	int ret = 0;

	/* special handling for IPA CCM. */
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		/* this is a request for identification from the BSC. */
		if (msg_type == IPAC_MSGT_ID_GET) {
			if (!link->line->ops->sign_link_up) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				goto err;
			}
		}
	}

	/* core CCM handling */
	ret = ipaccess_bts_handle_ccm(link, link->line->ops->cfg.ipa.dev, msg);
	if (ret < 0)
		goto err;

	if (ret == 1 && hh->proto == IPAC_PROTO_IPACCESS) {
		if (msg_type == IPAC_MSGT_ID_GET) {
			sign_link = link->line->ops->sign_link_up(msg,
					link->line,
					link->ofd->priv_nr);
			if (sign_link == NULL) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				goto err;
			}
		}
		msgb_free(msg);
		return ret;
	} else if (link->port == IPA_TCP_PORT_OML)
		e1i_ts = e1inp_line_ipa_oml_ts(link->line);
	else if (link->port == IPA_TCP_PORT_RSL)
		e1i_ts = e1inp_line_ipa_rsl_ts(link->line, link->ofd->priv_nr - E1INP_SIGN_RSL);

	OSMO_ASSERT(e1i_ts != NULL);

	if (e1i_ts->type == E1INP_TS_TYPE_NONE) {
		LOGP(DLINP, LOGL_ERROR, "Signalling link not initialized. Discarding."
		     " port=%u msg_type=%u\n", link->port, msg_type);
		goto err;
	}

	/* look up for some existing signaling link. */
	sign_link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (sign_link == NULL) {
		LOGP(DLINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		goto err;
	}
	msg->dst = sign_link;

	/* XXX better use e1inp_ts_rx? */
	if (!link->line->ops->sign_link) {
		LOGP(DLINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		goto err;
	}
	return link->line->ops->sign_link(msg);

err:
	ipa_client_conn_close(link);
	msgb_free(msg);
	return -EBADF;
}

struct ipaccess_line {
	bool line_already_initialized;
	struct ipa_client_conn *ipa_cli[NUM_E1_TS]; /* 0=OML, 1+N=TRX_N */
};

static int ipaccess_line_update(struct e1inp_line *line)
{
	int ret = -ENOENT;
	struct ipaccess_line *il;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct ipaccess_line);

	if (!line->driver_data) {
		LOGP(DLINP, LOGL_ERROR, "ipaccess: OOM in line update\n");
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

		LOGP(DLINP, LOGL_NOTICE, "enabling ipaccess BSC mode on %s "
		     "with OML %u and RSL %u TCP ports\n", ipa,
		     IPA_TCP_PORT_OML, IPA_TCP_PORT_RSL);

		oml_link = ipa_server_link_create(tall_ipa_ctx, line, ipa,
						  IPA_TCP_PORT_OML,
						  ipaccess_bsc_oml_cb, NULL);
		if (oml_link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create OML "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		oml_link->dscp = g_e1inp_ipaccess_pars.oml.dscp;
		oml_link->priority = g_e1inp_ipaccess_pars.oml.priority;
		if (ipa_server_link_open(oml_link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open OML BSC link: %s\n",
				strerror(errno));
			ipa_server_link_destroy(oml_link);
			return -EIO;
		}
		rsl_link = ipa_server_link_create(tall_ipa_ctx, line, ipa,
						  IPA_TCP_PORT_RSL,
						  ipaccess_bsc_rsl_cb, NULL);
		if (rsl_link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create RSL "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		rsl_link->dscp = g_e1inp_ipaccess_pars.rsl.dscp;
		rsl_link->priority = g_e1inp_ipaccess_pars.rsl.priority;
		if (ipa_server_link_open(rsl_link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open RSL BSC link: %s\n",
				strerror(errno));
			ipa_server_link_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	case E1INP_LINE_R_BTS: {
		struct ipa_client_conn *link;
		struct e1inp_ts *e1i_ts;

		LOGP(DLINP, LOGL_NOTICE, "enabling ipaccess BTS mode, "
		     "OML connecting to %s:%u\n", line->ops->cfg.ipa.addr,
		     IPA_TCP_PORT_OML);

		/* Drop previous line */
		if (il->ipa_cli[0]) {
			ipa_client_conn_close(il->ipa_cli[0]);
			ipa_client_conn_destroy(il->ipa_cli[0]);
			il->ipa_cli[0] = NULL;
		}

		link = ipa_client_conn_create2(tall_ipa_ctx,
					      e1inp_line_ipa_oml_ts(line),
					      E1INP_SIGN_OML,
					      NULL, 0,
					      line->ops->cfg.ipa.addr,
					      IPA_TCP_PORT_OML,
					      ipaccess_bts_updown_cb,
					      ipaccess_bts_read_cb,
					      ipaccess_bts_write_cb,
					      line);
		if (link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create OML "
				"BTS link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		link->dscp = g_e1inp_ipaccess_pars.oml.dscp;
		link->priority = g_e1inp_ipaccess_pars.oml.priority;
		if (ipa_client_conn_open(link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open OML BTS link: %s\n",
				strerror(errno));
			ipa_client_conn_close(link);
			ipa_client_conn_destroy(link);
			return -EIO;
		}

		e1i_ts = e1inp_line_ipa_oml_ts(line);
		ipaccess_bts_keepalive_fsm_alloc(e1i_ts, link, "oml_bts_to_bsc");
		il->ipa_cli[0] = link;
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
	struct ipa_client_conn *rsl_link;
	struct e1inp_ts *e1i_ts = e1inp_line_ipa_rsl_ts(line, trx_nr);
	struct ipaccess_line *il;
	int rc;

	if (E1INP_SIGN_RSL+trx_nr-1 >= NUM_E1_TS) {
		LOGP(DLINP, LOGL_ERROR, "cannot create RSL BTS link: "
			"trx_nr (%d) out of range\n", trx_nr);
		return -EINVAL;
	}

	/* Drop previous line */
	if ((rc = e1inp_ipa_bts_rsl_close_n(line, trx_nr)) < 0)
		return rc;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct ipaccess_line);
	il = line->driver_data;

	rsl_link = ipa_client_conn_create2(tall_ipa_ctx,
					  e1inp_line_ipa_rsl_ts(line, trx_nr),
					  E1INP_SIGN_RSL+trx_nr,
					  NULL, 0,
					  rem_addr, rem_port,
					  ipaccess_bts_updown_cb,
					  ipaccess_bts_read_cb,
					  ipaccess_bts_write_cb,
					  line);
	if (rsl_link == NULL) {
		LOGP(DLINP, LOGL_ERROR, "cannot create RSL "
			"BTS link: %s\n", strerror(errno));
		return -ENOMEM;
	}
	rsl_link->dscp = g_e1inp_ipaccess_pars.rsl.dscp;
	rsl_link->priority = g_e1inp_ipaccess_pars.rsl.priority;
	if (ipa_client_conn_open(rsl_link) < 0) {
		LOGP(DLINP, LOGL_ERROR, "cannot open RSL BTS link: %s\n",
			strerror(errno));
		ipa_client_conn_close(rsl_link);
		ipa_client_conn_destroy(rsl_link);
		return -EIO;
	}
	ipaccess_bts_keepalive_fsm_alloc(e1i_ts, rsl_link, "rsl_bts_to_bsc");
	il->ipa_cli[1 + trx_nr] = rsl_link;
	return 0;
}

/* Close the underlying IPA TCP socket of an RSL link */
int e1inp_ipa_bts_rsl_close_n(struct e1inp_line *line, uint8_t trx_nr)
{
	struct ipaccess_line *il;

	if (E1INP_SIGN_RSL+trx_nr-1 >= NUM_E1_TS) {
		LOGP(DLINP, LOGL_ERROR,
		     "cannot close RSL BTS link: trx_nr (%d) out of range\n", trx_nr);
		return -EINVAL;
	}

	il = line->driver_data;
	if (!il)
		return 0; /* Nothing to do, no lines created */

	if (il->ipa_cli[1 + trx_nr]) {
		ipa_client_conn_close(il->ipa_cli[1 + trx_nr]);
		ipa_client_conn_destroy(il->ipa_cli[1 + trx_nr]);
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

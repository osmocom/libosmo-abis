/* OpenBSC Abis input driver for osmo-e1d */

/* (C) 2019 by Sylvain Munaut <tnt@246tNt.com>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "config.h"
#include "internal.h"

#ifdef HAVE_E1D

#include <errno.h>
#include <unistd.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/fsm.h>
#include <osmocom/core/signal.h>

#include <osmocom/vty/vty.h>

#include <osmocom/abis/subchan_demux.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/lapd.h>

#include <osmocom/e1d/proto.h>
#include <osmocom/e1d/proto_clnt.h>


#define TS_SIGN_ALLOC_SIZE 300
#define S(x) (1 << (x))


struct osmo_e1dp_client *g_e1d;
struct osmo_fsm_inst *g_e1d_fsm_inst = NULL;

static int invertbits = 1;

/* pre-declaration */
extern struct e1inp_driver e1d_driver;
static int e1d_want_write(struct e1inp_ts *e1i_ts);
static int e1d_fd_cb(struct osmo_fd *bfd, unsigned int what);
static int e1d_line_update(struct e1inp_line *line);

/* flag array to remember which lines are handled by osmo-e1d */
bool lines[256];

enum fsm_e1d_client_states {
	ST_DISCONNECTED,
	ST_CONNECTED,
};

enum fsm_e1d_client_evt {
	EV_CONN_LOST,
	EV_CONNECT,
};

static const struct value_string fsm_e1d_client_evt_names[] = {
	OSMO_VALUE_STRING(EV_CONN_LOST),
	OSMO_VALUE_STRING(EV_CONNECT),
	{ 0, NULL }
};

struct e1inp_line *e1inp_port_find(uint8_t port_nr)
{
	struct e1inp_line *e1i_line;

	/* iterate over global list of e1 lines */
	llist_for_each_entry(e1i_line, &e1inp_line_list, list) {
		if (e1i_line->port_nr == port_nr)
			return e1i_line;
	}
	return NULL;
}

static void e1d_client_event_cb(enum osmo_e1dp_msg_type event, uint8_t intf, uint8_t line, uint8_t ts, uint8_t *data, int len)
{
	struct e1inp_line *e1_line;
	struct input_signal_data isd;
	int signal;
	struct osmo_e1dp_cas_bits *cas;

	memset(&isd, 0, sizeof(isd));

	/* we use higher 4 bits for interface, lower 4 bits for line,
	 * resulting in max. 16 interfaces with 16 lines each */
	e1_line = e1inp_port_find((intf << 4) | line);
	if (!e1_line)
		return;
	isd.line = e1_line;

	switch (event) {
	case E1DP_EVT_LOS_ON:
		signal = S_L_INP_LINE_LOS;
		break;
	case E1DP_EVT_LOS_OFF:
		signal = S_L_INP_LINE_NOLOS;
		break;
	case E1DP_EVT_AIS_ON:
		signal = S_L_INP_LINE_AIS;
		break;
	case E1DP_EVT_AIS_OFF:
		signal = S_L_INP_LINE_NOAIS;
		break;
	case E1DP_EVT_RAI_ON:
		signal = S_L_INP_LINE_RAI;
		break;
	case E1DP_EVT_RAI_OFF:
		signal = S_L_INP_LINE_NORAI;
		break;
	case E1DP_EVT_LOF_ON:
		signal = S_L_INP_LINE_LOF;
		break;
	case E1DP_EVT_LOF_OFF:
		signal = S_L_INP_LINE_NOLOF;
		break;
	case E1DP_EVT_SABITS:
		signal = S_L_INP_LINE_SA_BITS;
		if (len < 1)
			return;
		isd.sa_bits = *data;
	case E1DP_EVT_CAS:
		signal = S_L_INP_LINE_CAS;
		cas = (struct osmo_e1dp_cas_bits *)data;
		if (len < sizeof(*cas))
			return;
		isd.cas = cas->bits;
		isd.ts_nr = ts;
		break;
	default:
		/* Ignore all other events. */
		return;
	}

	osmo_signal_dispatch(SS_L_INPUT, signal, &isd);
}

static int fsm_e1_client_timer_cb(struct osmo_fsm_inst *fi)
{
	osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONNECT, NULL);
	return 0;
}

static void fsm_e1d_client_disconnected_cb(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	switch (event) {
	case EV_CONNECT:
		if (!g_e1d) {
			g_e1d = osmo_e1dp_client_create(NULL, "/tmp/osmo-e1d.ctl");
			if (!g_e1d) {
				LOGPFSML(fi, LOGL_ERROR, "Unable to (re)connect to osmo-e1d daemon, retrying...\n");
				osmo_fsm_inst_state_chg(g_e1d_fsm_inst, ST_DISCONNECTED, 1, 0);
				return;
			}
			osmo_e1dp_client_event_register(g_e1d, e1d_client_event_cb);
		}

		LOGPFSML(fi, LOGL_NOTICE, "Successfully (re)connected to osmo-e1d daemon!\n");
		osmo_fsm_inst_state_chg(g_e1d_fsm_inst, ST_CONNECTED, 0, 0);
		break;
	default:
		OSMO_ASSERT(false);
		break;
	}
}

static void terminate_line(struct e1inp_line *line)
{
	int ts;

	/* There should be technically no way to get a non e1d line into our private memory */
	OSMO_ASSERT(line->driver == &e1d_driver);

	for (ts = 1; ts < line->num_ts; ts++) {
		unsigned int idx = ts - 1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.e1d.fd;

		/* Only affect file descriptors that are currently in use (do not reset file descriptor value since we
		 * will use this value to detect if the file descriptor was in use when the connection broke.) */
		if (bfd->fd >= 0) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Terminating osmo-e1d connection to timeslot: %d\n", ts);
			osmo_fd_unregister(bfd);
			close(bfd->fd);
			bfd->fd = -1;
		}
	}
}

static void fsm_e1d_client_connected_onenter_cb(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	unsigned int i;
	struct e1inp_line *line;
	int ret;

	/* Run a line update to re-establish lost connections */
	for (i = 0; i < ARRAY_SIZE(lines); i++) {
		if (!lines[i])
			continue;

		line = e1inp_line_find(i);
		if (!line) {
			/* Apparantly we lost a line - this should not happen */
			lines[i] = false;
			continue;
		}

		ret = e1d_line_update(line);
		if (ret < 0)
			LOGPFSML(fi, LOGL_ERROR, "Line update failed after (re)connecting to osmo-e1d daemon!\n");
	}
}

static void fsm_e1d_client_connected_cb(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	unsigned int i;
	struct e1inp_line *line;

	switch (event) {
	case EV_CONN_LOST:
		LOGPFSML(fi, LOGL_ERROR, "Lost connection to osmo-e1d daemon!\n");

		/* Destroy e1d clinet */
		if (g_e1d) {
			osmo_e1dp_client_destroy(g_e1d);
			g_e1d = NULL;
		}

		/* Terminate all lines at once */
		for (i = 0; i < ARRAY_SIZE(lines); i++) {
			if (!lines[i])
				continue;
			line = e1inp_line_find(i);
			if (!line) {
				/* Apparantly we lost a line - this should not happen */
				lines[i] = false;
				continue;
			}
			terminate_line(line);
		}

		osmo_fsm_inst_state_chg(fi, ST_DISCONNECTED, 1, 0);
		break;
	default:
		OSMO_ASSERT(false);
		break;
	}
}

static bool e1d_connected(void)
{
	if (g_e1d_fsm_inst && g_e1d_fsm_inst->state == ST_CONNECTED)
		return true;

	LOGPFSML(g_e1d_fsm_inst, LOGL_ERROR, "No connection to osmo-e1d daemon!\n");
	return false;
}

static struct osmo_fsm_state fsm_e1d_client_states[] = {

	/* Initial CRCX state. This state is immediately entered and executed
	 * when the FSM is started. The rationale is that we first have to
	 * create a connectin before we can execute other operations on that
	 * connection. */
	[ST_DISCONNECTED] = {
			     .in_event_mask = S(EV_CONNECT),
			     .out_state_mask = S(ST_CONNECTED) | S(ST_DISCONNECTED),
			     .name = OSMO_STRINGIFY(ST_DISCONNECTED),
			     .action = fsm_e1d_client_disconnected_cb,
			      },

	/* Wait for the response to a CRCX operation, check and process the
	 * results, change to ST_READY afterwards. */
	[ST_CONNECTED] = {
			  .in_event_mask = S(EV_CONN_LOST),
			  .out_state_mask = S(ST_DISCONNECTED),
			  .name = OSMO_STRINGIFY(ST_CONNECTED),
			  .action = fsm_e1d_client_connected_cb,
			  .onenter = fsm_e1d_client_connected_onenter_cb,
			   },

};

static struct osmo_fsm fsm_e1d_client = {
	.name = "e1d_client",
	.states = fsm_e1d_client_states,
	.num_states = ARRAY_SIZE(fsm_e1d_client_states),
	.log_subsys = DLINP,
	.event_names = fsm_e1d_client_evt_names,
	.timer_cb = fsm_e1_client_timer_cb,
};

static int handle_ts_sign_read(struct osmo_fd *bfd)
{
        struct e1inp_line *line = bfd->data;
        unsigned int ts_nr = bfd->priv_nr;
        struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TS_SIGN_ALLOC_SIZE, "E1D Signaling TS");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, TS_SIGN_ALLOC_SIZE - 16);
	if (ret < 1) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "%s read error: %d %s\n", __func__, ret,
			ret < 0 ? strerror(errno) : "bytes read");
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
		return ret;
	}

	msgb_put(msg, ret);

        return e1inp_rx_ts_lapd(e1i_ts, msg);
}

static void timeout_ts_sign_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	e1d_want_write(e1i_ts);
}

static int handle_ts_sign_write(struct osmo_fd *bfd)
{
        struct e1inp_line *line = bfd->data;
        unsigned int ts_nr = bfd->priv_nr;
        struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *sign_link;
	struct msgb *msg;

	osmo_fd_write_disable(bfd);

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		return 0;
	}

	DEBUGP(DLMI, "TX: %s\n", osmo_hexdump(msg->data, msg->len));
	lapd_transmit(e1i_ts->lapd, sign_link->tei,
		sign_link->sapi, msg);

	/* set tx delay timer for next event */
	osmo_timer_setup(&e1i_ts->sign.tx_timer, timeout_ts_sign_write, e1i_ts);
	osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, 50000);

	return 0;
}

#define D_TSX_ALLOC_SIZE (D_BCHAN_TX_GRAN)

static int handle_ts_trau_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	uint8_t tx_buf[D_BCHAN_TX_GRAN];
	struct subch_mux *mx = &e1i_ts->trau.mux;
	int ret;

	ret = subchan_mux_out(mx, tx_buf, D_BCHAN_TX_GRAN);

	if (ret != D_BCHAN_TX_GRAN) {
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "Huh, got ret of %d\n", ret);
		if (ret < 0)
			return ret;
	}

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "BCHAN TX: %s\n", osmo_hexdump(tx_buf, D_BCHAN_TX_GRAN));

	if (invertbits)
		osmo_revbytebits_buf(tx_buf, ret);

	ret = write(bfd->fd, tx_buf, ret);
	if (ret < D_BCHAN_TX_GRAN) {
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "send returns %d instead of %d\n",
			ret, D_BCHAN_TX_GRAN);
		if (ret <= 0)
			osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
	}

	return ret;
}


static int handle_ts_trau_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(D_TSX_ALLOC_SIZE, "E1D Rx TSx");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, D_TSX_ALLOC_SIZE);
	if (ret < 0 || ret != D_TSX_ALLOC_SIZE) {
		/* FIXME: The socket that we read from is of type SOCK_STREAM. This means that we might read less then
		 * D_TSX_ALLOC_SIZE even though the connection is still fine. Since data is continuously written (in
		 * chunks of D_TSX_ALLOC_SIZE) on the other side we should not get partial reads too often but it is
		 * still possible and when it happens, a reconnect cycle will be triggered. To fix this we should add a
		 * buffering mechainsm that buffers the incomplete read instead of dropping the connection. (changing
		 * the socket type to SOCK_SEQPACKET would be an alternative, but it would break backward compatibility
		 * of the interface.) */
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "%s read error: %d %s\n", __func__, ret,
			ret < 0 ? strerror(errno) : "bytes read");
		if (ret <= 0)
			osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
		return ret;
	}

	if (invertbits)
		osmo_revbytebits_buf(msg->data, ret);

	msgb_put(msg, ret);

	msg->l2h = msg->data;
	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "BCHAN RX: %s\n", osmo_hexdump(msgb_l2(msg), ret));
	ret = e1inp_rx_ts(e1i_ts, msg, 0, 0);
	/* physical layer indicates that data has been sent,
	 * we thus can send some more data */
	ret = handle_ts_trau_write(bfd);

	return ret;
}

/* write to a raw channel TS */
static int handle_ts_raw_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	int ret;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, NULL);
	if (!msg) {
		osmo_fd_write_disable(bfd);
		return 0;
	}

	if (msg->len != D_BCHAN_TX_GRAN) {
		/* This might lead to a transmit underrun, as we call tx
		 * from the rx path, as there's no select/poll on dahdi
		 * */
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "unexpected msg->len = %u, "
		     "expected %u\n", msg->len, D_BCHAN_TX_GRAN);
	}

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN TX: %s\n", osmo_hexdump(msg->data, msg->len));

	if (0/*invertbits*/)
		osmo_revbytebits_buf(msg->data, msg->len);

	ret = write(bfd->fd, msg->data, msg->len);
	if (ret < msg->len) {
		/* FIXME: The socket that we write to is of type SOCK_STREAM. This means that it may happen that the
		 * syscall is not able to write the full data chunk at once. This is a rare event, but when it happens,
		 * a reconnect cycle is triggered, even though the connection is still fine. To fix this, we should
		 * buffer the remainder of the data to write it in the next cycle instead of dropping the connection.
		 * (changing the socket type to SOCK_SEQPACKET would be an alternative, but it would break backward
		 * compatibility of the interface.) */
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "send returns %d instead of %d\n", ret, msg->len);
		if (ret <= 0)
			osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
	}
	msgb_free(msg);

	return ret;
}

static int handle_ts_raw_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(D_TSX_ALLOC_SIZE, "E1D Raw TS");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, D_TSX_ALLOC_SIZE);
	if (ret < 0 || ret != D_TSX_ALLOC_SIZE) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "%s read error: %d %s\n", __func__, ret,
			ret < 0 ? strerror(errno) : "bytes read");
		msgb_free(msg);
		if (ret <= 0)
			osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
		return ret;
	}

	if (0/*invertbits*/)
		osmo_revbytebits_buf(msg->data, ret);

	msgb_put(msg, ret);

	msg->l2h = msg->data;
	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN RX: %s\n", msgb_hexdump_l2(msg));
	ret = e1inp_rx_ts(e1i_ts, msg, 0, 0);

	return ret;
}

/* write to a hdlc channel TS */
static int handle_ts_hdlc_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	int ret;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, NULL);
	if (!msg) {
		osmo_fd_write_disable(bfd);
		return 0;
	}

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "HDLC CHAN TX: %s\n", osmo_hexdump(msg->data, msg->len));

	ret = write(bfd->fd, msg->data, msg->len);
	if (ret < msg->len) {
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "send returns %d instead of %d\n", ret, msg->len);
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
	}
	msgb_free(msg);

	return ret;
}

#define TSX_ALLOC_SIZE 4096

/* read from a hdlc channel TS */
static int handle_ts_hdlc_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TSX_ALLOC_SIZE, "E1D HDLC TS");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, TSX_ALLOC_SIZE);
	if (ret <= 0) {
		LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "%s read error: %d %s\n", __func__, ret, strerror(errno));
		msgb_free(msg);
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
		return ret;
	}

	msgb_put(msg, ret);

	msg->l2h = msg->data;
	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "HDLC CHAN RX: %s\n", msgb_hexdump_l2(msg));
	ret = e1inp_rx_ts(e1i_ts, msg, 0, 0);

	return ret;
}

static void e1d_write_msg(struct msgb *msg, void *cbdata)
{
	struct osmo_fd *bfd = cbdata;
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	int ret;

	if (!e1d_connected()) {
		msgb_free(msg);
		return;
	}

	ret = write(bfd->fd, msg->data, msg->len);
	msgb_free(msg);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLMI, LOGL_NOTICE, "%s write failed %d\n", __func__, ret);
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, line);
	}
}

static int e1d_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
        struct e1inp_line *line = bfd->data;
        unsigned int ts_nr = bfd->priv_nr;
        unsigned int idx = ts_nr-1;
        struct e1inp_ts *e1i_ts = &line->ts[idx];
	int ret = 0;

	if (!e1d_connected())
		return -EIO;

	switch (e1i_ts->type) {
	case E1INP_TS_TYPE_SIGN:
		if (what & OSMO_FD_READ)
			ret = handle_ts_sign_read(bfd);
		if (what & OSMO_FD_WRITE)
			ret = handle_ts_sign_write(bfd);
		break;
	case E1INP_TS_TYPE_TRAU:
		if (what & OSMO_FD_READ)
			ret = handle_ts_trau_read(bfd);
		/* handle_ts_trau_write() is called inside handle_ts_trau_read().
		 * OSMO_FD_WRITE flag is not required here and will not be set.
		 */
		break;
	case E1INP_TS_TYPE_RAW:
		if (what & OSMO_FD_READ)
			ret = handle_ts_raw_read(bfd);
		if (what & OSMO_FD_WRITE)
			ret = handle_ts_raw_write(bfd);
		break;
	case E1INP_TS_TYPE_HDLC:
		if (what & OSMO_FD_READ)
			ret = handle_ts_hdlc_read(bfd);
		if (what & OSMO_FD_WRITE)
			ret = handle_ts_hdlc_write(bfd);
		break;
	default:
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "unknown/unsupported E1 TS type %u\n", e1i_ts->type);
		break;
	}

	return ret;
}


static int e1d_want_write(struct e1inp_ts *e1i_ts)
{
	if (!e1d_connected())
		return -EIO;

        /* We never include the DAHDI B-Channel FD into the writeset */
	if (e1i_ts->type == E1INP_TS_TYPE_TRAU ||
	    e1i_ts->type == E1INP_TS_TYPE_I460) {
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "Trying to write TRAU ts\n");
		return 0;
	}

	osmo_fd_write_enable(&e1i_ts->driver.e1d.fd);

	return 0;
}

static int e1d_line_update(struct e1inp_line *line)
{
	int ts;
	int ret;

	/* we use higher 4 bits for interface, lower 4 bits for line,
	 * resulting in max. 16 interfaces with 16 lines each */
	uint8_t e1d_intf = (line->port_nr >> 4) & 0xF;
	uint8_t e1d_line = line->port_nr & 0xF;
	struct osmo_e1dp_ts_info *ts_info;
	int num_ts_info;

	if (line->driver != &e1d_driver)
		return -EINVAL;

	/* Memorize that osmo-e1d is responsible for this line */
	lines[line->num] = true;

	/* Spawn FSM to handle connection towards osmo-e1d */
	if (!g_e1d_fsm_inst) {
		g_e1d_fsm_inst = osmo_fsm_inst_alloc(&fsm_e1d_client, NULL, NULL, LOGL_DEBUG, "fsm_e1d_client");
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONNECT, NULL);
		OSMO_ASSERT(g_e1d_fsm_inst);
	}

	/* In case no connection to osmo-e1d is available, we may postpone the line update until the connection is
	 * available (again) */
	if (!e1d_connected()) {
		LOGPIL(line, DLINP, LOGL_NOTICE, "No connection to osmo-e1d daemon, postponing Line update: %d %d=E1D(%d:%d) %d\n", line->num, line->port_nr,
		       e1d_intf, e1d_line, line->num_ts);
		return 0;
	}
	LOGPIL(line, DLINP, LOGL_NOTICE, "Line update: %d %d=E1D(%d:%d) %d\n", line->num, line->port_nr,
		e1d_intf, e1d_line, line->num_ts);

	ret = osmo_e1dp_client_ts_query(g_e1d, &ts_info, &num_ts_info, e1d_intf, e1d_line, E1DP_INVALID);
	if (ret < 0) {
		LOGPIL(line, DLINP, LOGL_ERROR, "Cannot query E1D for timeslot information: %d, postpoining line update\n", ret);
		osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, NULL);
		/* Since we have mechanisms in place that allow us to postpone the line update until the connection
		 * to osmo-e1d is up again, we may pretend that the line update went ok. */
		return 0;
	}

	for (ts=1; ts<line->num_ts; ts++)
	{
		unsigned int idx = ts-1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.e1d.fd;

		/* unregister FD if it was already registered/in use */
		if (osmo_fd_is_registered(bfd))
			osmo_fd_unregister(bfd);

		if (e1i_ts->type != E1INP_TS_TYPE_NONE && ts >= num_ts_info) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Timeslot configured, but not existent "
				"on E1D side; skipping\n");
			continue;
		}

		switch (e1i_ts->type) {
		case E1INP_TS_TYPE_NONE:
			/* close/release LAPD instance, if any */
			if (e1i_ts->lapd) {
				lapd_instance_free(e1i_ts->lapd);
				e1i_ts->lapd = NULL;
			}
			if (bfd->fd >= 0) {
				close(bfd->fd);
				bfd->fd = -1;
			}
                        continue;
		case E1INP_TS_TYPE_SIGN:
			if (bfd->fd >= 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_HDLCFCS) {
				close(bfd->fd);
				bfd->fd = -1;
			}
			if (bfd->fd < 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_HDLCFCS, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
				osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, NULL);
				return -EIO;
			}
			bfd->when = OSMO_FD_READ;

			if (!e1i_ts->lapd) {
				char name[32];
				e1inp_ts_name(name, sizeof(name), e1i_ts);
				e1i_ts->lapd = lapd_instance_alloc2(1,
					e1d_write_msg, bfd, e1inp_dlsap_up,
					e1i_ts, &lapd_profile_abis, name);
			}
			break;
		case E1INP_TS_TYPE_HDLC:
			/* close/release LAPD instance, if any */
			if (e1i_ts->lapd) {
				lapd_instance_free(e1i_ts->lapd);
				e1i_ts->lapd = NULL;
			}
			/* close, if old timeslot mode doesn't match new config */
			if (bfd->fd >= 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_HDLCFCS) {
				close(bfd->fd);
				bfd->fd = -1;
			}
			if (bfd->fd < 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_HDLCFCS, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
				osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, NULL);
				return -EIO;
			}
			bfd->when = OSMO_FD_READ;
			break;
		case E1INP_TS_TYPE_TRAU:
		case E1INP_TS_TYPE_I460:
		case E1INP_TS_TYPE_RAW:
			/* close/release LAPD instance, if any */
			if (e1i_ts->lapd) {
				lapd_instance_free(e1i_ts->lapd);
				e1i_ts->lapd = NULL;
			}
			/* close, if old timeslot mode doesn't match new config */
			if (bfd->fd >= 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_RAW) {
				close(bfd->fd);
				bfd->fd = -1;
			}
			if (bfd->fd < 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_RAW, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
				osmo_fsm_inst_dispatch(g_e1d_fsm_inst, EV_CONN_LOST, NULL);
				return -EIO;
			}
			bfd->when = OSMO_FD_READ;
			break;
		};

		osmo_fd_setup(bfd, bfd->fd, bfd->when, e1d_fd_cb, line, ts);
		ret = osmo_fd_register(bfd);
		if (ret < 0) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not register FD: %s\n", strerror(ret));
			talloc_free(ts_info);
			close(bfd->fd);
			bfd->fd = -1;
			return ret;
		}
	}

	talloc_free(ts_info);
	return 0;
}

static int e1d_line_create(struct e1inp_line *line)
{
	int ts;

	for (ts = 1; ts < line->num_ts; ts++) {
		unsigned int idx = ts - 1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.e1d.fd;

		bfd->fd = -1;
	}

	return 0;
}

static int set_sa_bits(struct e1inp_line *line, uint8_t sa_bits)
{
	/* we use higher 4 bits for interface, lower 4 bits for line,
	 * resulting in max. 16 interfaces with 16 lines each */
	uint8_t e1d_intf = (line->port_nr >> 4) & 0xF;
	uint8_t e1d_line = line->port_nr & 0xF;

	return osmo_e1dp_client_set_sa_bits(g_e1d, e1d_intf, e1d_line, sa_bits);
}

static int set_cas(struct e1inp_ts *ts, uint8_t bits, bool query_rx)
{
	/* we use higher 4 bits for interface, lower 4 bits for line,
	 * resulting in max. 16 interfaces with 16 lines each */
	uint8_t e1d_intf = (ts->line->port_nr >> 4) & 0xF;
	uint8_t e1d_line = ts->line->port_nr & 0xF;
	struct osmo_e1dp_cas_bits cas = { bits, query_rx };

	return osmo_e1dp_client_set_cas(g_e1d, e1d_intf, e1d_line, ts->num, &cas);
}

struct e1inp_driver e1d_driver = {
	.name        = "e1d",
	.want_write  = e1d_want_write,
	.set_sa_bits = set_sa_bits,
	.set_cas     = set_cas,
	.line_update = e1d_line_update,
	.line_create = e1d_line_create,
};

int e1inp_e1d_init(void)
{
	OSMO_ASSERT(osmo_fsm_register(&fsm_e1d_client) == 0);
	memset(lines, 0, sizeof(lines));

	/* register the driver with the core */
	return e1inp_driver_register(&e1d_driver);
}

#endif /* HAVE_E1D */

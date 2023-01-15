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

#include <osmocom/vty/vty.h>

#include <osmocom/abis/subchan_demux.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/lapd.h>

#include <osmocom/e1d/proto.h>
#include <osmocom/e1d/proto_clnt.h>


#define TS_SIGN_ALLOC_SIZE  300

struct osmo_e1dp_client *g_e1d;

static int invertbits = 1;

/* pre-declaration */
extern struct e1inp_driver e1d_driver;
static int e1d_want_write(struct e1inp_ts *e1i_ts);


static int
handle_ts_sign_read(struct osmo_fd *bfd)
{
        struct e1inp_line *line = bfd->data;
        unsigned int ts_nr = bfd->priv_nr;
        struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TS_SIGN_ALLOC_SIZE, "E1D Signaling TS");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, TS_SIGN_ALLOC_SIZE - 16);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "%s read failed %d (%s)\n", __func__, ret, strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);
	if (ret <= 1) {
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "%s read failed %d (%s)\n", __func__, ret, strerror(errno));
		return ret;
	}

        return e1inp_rx_ts_lapd(e1i_ts, msg);
}

static void
timeout_ts_sign_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	e1d_want_write(e1i_ts);
}

static int
handle_ts_sign_write(struct osmo_fd *bfd)
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

static int
handle_ts_trau_write(struct osmo_fd *bfd)
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
	if (ret < D_BCHAN_TX_GRAN)
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "send returns %d instead of %d\n",
			ret, D_BCHAN_TX_GRAN);

	return ret;
}


static int
handle_ts_trau_read(struct osmo_fd *bfd)
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
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "read error  %d %s\n", ret, strerror(errno));
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
	if (ret < msg->len)
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "send returns %d instead of %d\n", ret, msg->len);
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
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "read error  %d %s\n", ret, strerror(errno));
		msgb_free(msg);
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

static void
e1d_write_msg(struct msgb *msg, void *cbdata)
{
	struct osmo_fd *bfd = cbdata;
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	int ret;

	ret = write(bfd->fd, msg->data, msg->len);
	msgb_free(msg);
	if (ret < 0)
		LOGPITS(e1i_ts, DLMI, LOGL_NOTICE, "%s write failed %d\n", __func__, ret);
}

static int
e1d_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
        struct e1inp_line *line = bfd->data;
        unsigned int ts_nr = bfd->priv_nr;
        unsigned int idx = ts_nr-1;
        struct e1inp_ts *e1i_ts = &line->ts[idx];
	int ret = 0;

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
		if (what & OSMO_FD_WRITE)
			ret = handle_ts_trau_write(bfd);
		break;
	case E1INP_TS_TYPE_RAW:
		if (what & OSMO_FD_READ)
			ret = handle_ts_raw_read(bfd);
		if (what & OSMO_FD_WRITE)
			ret = handle_ts_raw_write(bfd);
		break;
	default:
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "unknown/unsupported E1 TS type %u\n", e1i_ts->type);
		break;
	}

	return ret;
}


static int
e1d_want_write(struct e1inp_ts *e1i_ts)
{
        /* We never include the DAHDI B-Channel FD into the writeset */
	if (e1i_ts->type == E1INP_TS_TYPE_TRAU ||
	    e1i_ts->type == E1INP_TS_TYPE_I460) {
		LOGPITS(e1i_ts, DLINP, LOGL_DEBUG, "Trying to write TRAU ts\n");
		return 0;
	}

	osmo_fd_write_enable(&e1i_ts->driver.e1d.fd);

	return 0;
}

static int
e1d_line_update(struct e1inp_line *line)
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

	if (!g_e1d) {
		/* Connect to daemon */
		g_e1d = osmo_e1dp_client_create(NULL, "/tmp/osmo-e1d.ctl");
		if (!g_e1d) {
			LOGPIL(line, DLINP, LOGL_ERROR, "Unable to connect to osmo-e1d daemon\n");
			return -EPIPE;
		}
	}

	LOGPIL(line, DLINP, LOGL_NOTICE, "Line update %d %d=E1D(%d:%d) %d\n", line->num, line->port_nr,
		e1d_intf, e1d_line, line->num_ts);

	ret = osmo_e1dp_client_ts_query(g_e1d, &ts_info, &num_ts_info, e1d_intf, e1d_line, E1DP_INVALID);
	if (ret < 0) {
		LOGPIL(line, DLINP, LOGL_ERROR, "Cannot query E1D for timeslot information: %d\n", ret);
		return -EIO;
	}

	for (ts=1; ts<line->num_ts; ts++)
	{
		unsigned int idx = ts-1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.e1d.fd;

		/* unregister FD if it was already registered */
		if (bfd->list.next && bfd->list.next != LLIST_POISON1)
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
			if (bfd->fd) {
				close(bfd->fd);
				bfd->fd = 0;
			}
                        continue;
		case E1INP_TS_TYPE_SIGN:
			if (bfd->fd > 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_HDLCFCS) {
				close(bfd->fd);
				bfd->fd = 0;
			}
			if (bfd->fd <= 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_HDLCFCS, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
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
			if (bfd->fd > 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_HDLCFCS) {
				close(bfd->fd);
				bfd->fd = 0;
			}
			if (bfd->fd <= 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_HDLCFCS, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
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
			if (bfd->fd > 0 && ts_info[ts].cfg.mode != E1DP_TSMODE_RAW) {
				close(bfd->fd);
				bfd->fd = 0;
			}
			if (bfd->fd <= 0) {
				bfd->fd = osmo_e1dp_client_ts_open(g_e1d, e1d_intf, e1d_line, ts,
								   E1DP_TSMODE_RAW, D_BCHAN_TX_GRAN);
			}
			if (bfd->fd < 0) {
				LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "Could not open timeslot %d\n", ts);
				talloc_free(ts_info);
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
			return ret;
		}
	}

	talloc_free(ts_info);
	return 0;
}

struct e1inp_driver e1d_driver = {
	.name        = "e1d",
	.want_write  = e1d_want_write,
	.line_update = e1d_line_update,
};

int
e1inp_e1d_init(void)
{
	/* register the driver with the core */
	return e1inp_driver_register(&e1d_driver);
}

#endif /* HAVE_E1D */

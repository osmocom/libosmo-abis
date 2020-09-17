/* OpenBSC Abis input driver for mISDNuser */

/* (C) 2008-2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2009 by Holger Hans Peter Freyther <zecke@selfish.org>
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

/*! \file misdn.c
 *  \brief Osmocom A-bis input driver for mISDN
 *
 * This driver has two modes of operations, exported via two different
 * \ref e1_input_driver structures: 
 * "misdn" is the classic version and it uses the in-kernel LAPD
 * implementation.  This is somewhat limited in e.g. the fact that
 * you can only have one E1 timeslot in signaling mode.
 * "misdn_lapd" is a newer version which uses userspace LAPD code
 * contained in libosmo-abis.  It offers the same flexibilty as the
 * DAHDI driver, i.e. any number of signaling slots.
 */

#include "internal.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <mISDNif.h>

//#define AF_COMPATIBILITY_FUNC
//#include <compat_af_isdn.h>
#ifndef AF_ISDN
#define AF_ISDN 34
#define PF_ISDN AF_ISDN
#endif

#include <osmocom/core/select.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/lapd.h>
#include <osmocom/core/talloc.h>

#define TS1_ALLOC_SIZE	300

/*! \brief driver-specific data for \ref e1inp_line::driver_data */
struct misdn_line {
	int use_userspace_lapd;
	int dummy_dchannel;
};

const struct value_string prim_names[] = {
	{ PH_CONTROL_IND, "PH_CONTROL_IND" },
	{ PH_DATA_IND, "PH_DATA_IND" },
	{ PH_DATA_CNF, "PH_DATA_CNF" },
	{ PH_ACTIVATE_IND, "PH_ACTIVATE_IND" },
	{ DL_ESTABLISH_IND, "DL_ESTABLISH_IND" },
	{ DL_ESTABLISH_CNF, "DL_ESTABLISH_CNF" },
	{ DL_RELEASE_IND, "DL_RELEASE_IND" },
	{ DL_RELEASE_CNF, "DL_RELEASE_CNF" },
	{ DL_DATA_IND, "DL_DATA_IND" },
	{ DL_UNITDATA_IND, "DL_UNITDATA_IND" },
	{ DL_INFORMATION_IND, "DL_INFORMATION_IND" },
	{ MPH_ACTIVATE_IND, "MPH_ACTIVATE_IND" },
	{ MPH_DEACTIVATE_IND, "MPH_DEACTIVATE_IND" },
	{ 0, NULL }
};

static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *link;
	struct msgb *msg = msgb_alloc(TS1_ALLOC_SIZE, "mISDN TS1");
	struct sockaddr_mISDN l2addr;
	struct mISDNhead *hh;
	socklen_t alen;
	int ret;

	if (!msg)
		return -ENOMEM;

	hh = (struct mISDNhead *) msg->data;

	alen = sizeof(l2addr);
	ret = recvfrom(bfd->fd, msg->data, 300, 0,
		       (struct sockaddr *) &l2addr, &alen);
	if (ret < 0) {
		fprintf(stderr, "recvfrom error  %s\n", strerror(errno));
		msgb_free(msg);
		return ret;
	}

	if (alen != sizeof(l2addr)) {
		fprintf(stderr, "%s error len\n", __func__);
		msgb_free(msg);
		return -EINVAL;
	}

	msgb_put(msg, ret);

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "alen =%d, dev(%d) channel(%d) sapi(%d) tei(%d)\n",
		alen, l2addr.dev, l2addr.channel, l2addr.sapi, l2addr.tei);

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "<= len = %d, prim(0x%x) id(0x%x): %s\n",
		ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case DL_INFORMATION_IND:
		/* mISDN tells us which channel number is allocated for this
		 * tuple of (SAPI, TEI). */
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "DL_INFORMATION_IND: use channel(%d) sapi(%d) tei(%d) for now\n",
			l2addr.channel, l2addr.sapi, l2addr.tei);
		link = e1inp_lookup_sign_link(e1i_ts, l2addr.tei, l2addr.sapi);
		if (!link) {
			LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "mISDN message for unknown sign_link\n");
			msgb_free(msg);
			return -EINVAL;
		}
		/* save the channel number in the driver private struct */
		link->driver.misdn.channel = l2addr.channel;
		msgb_free(msg);
		break;
	case DL_ESTABLISH_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "DL_ESTABLISH_IND: channel(%d) sapi(%d) tei(%d)\n",
			l2addr.channel, l2addr.sapi, l2addr.tei);
		/* For some strange reason, sometimes the DL_INFORMATION_IND tells
		 * us the wrong channel, and we only get the real channel number
		 * during the DL_ESTABLISH_IND */
		link = e1inp_lookup_sign_link(e1i_ts, l2addr.tei, l2addr.sapi);
		if (!link) {
			LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "mISDN message for unknown sign_link\n");
			msgb_free(msg);
			return -EINVAL;
		}
		/* save the channel number in the driver private struct */
		link->driver.misdn.channel = l2addr.channel;
		ret = e1inp_event(e1i_ts, S_L_INP_TEI_UP, l2addr.tei, l2addr.sapi);
		msgb_free(msg);
		break;
	case DL_RELEASE_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "DL_RELEASE_IND: channel(%d) sapi(%d) tei(%d)\n",
		l2addr.channel, l2addr.sapi, l2addr.tei);
		ret = e1inp_event(e1i_ts, S_L_INP_TEI_DN, l2addr.tei, l2addr.sapi);
		msgb_free(msg);
		break;
	case DL_DATA_IND:
	case DL_UNITDATA_IND:
		msg->l2h = msg->data + MISDN_HEADER_LEN;
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "RX: %s\n", osmo_hexdump(msgb_l2(msg), ret - MISDN_HEADER_LEN));
		if (mline->use_userspace_lapd) {
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "DL_DATA_IND but userspace LAPD ?!?\n");
			msgb_free(msg);
			return -EIO;
		}
		ret = e1inp_rx_ts(e1i_ts, msg, l2addr.tei, l2addr.sapi);
		break;
	case PH_ACTIVATE_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "PH_ACTIVATE_IND: channel(%d) sapi(%d) tei(%d)\n",
		l2addr.channel, l2addr.sapi, l2addr.tei);
		msgb_free(msg);
		break;
	case PH_DEACTIVATE_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "PH_DEACTIVATE_IND: channel(%d) sapi(%d) tei(%d)\n",
		l2addr.channel, l2addr.sapi, l2addr.tei);
		msgb_free(msg);
		break;
	case PH_DATA_IND:
		if (!mline->use_userspace_lapd) {
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "PH_DATA_IND but kernel LAPD ?!?\n");
			return -EIO;
		}
		/* remove the Misdn Header */
		msgb_pull(msg, MISDN_HEADER_LEN);
		/* hand into the LAPD code */
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "RX: %s\n", osmo_hexdump(msg->data, msg->len));
		ret = e1inp_rx_ts_lapd(e1i_ts, msg);
		break;
	default:
		msgb_free(msg);
		break;
	}
	return ret;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	/* We never include the mISDN B-Channel FD into the
	 * writeset, since it doesn't support poll() based
	 * write flow control */		
	if (e1i_ts->type == E1INP_TS_TYPE_TRAU ||
	    e1i_ts->type == E1INP_TS_TYPE_I460)
		return 0;

	e1i_ts->driver.misdn.fd.when |= BSC_FD_WRITE;

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
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *sign_link;
	struct sockaddr_mISDN sa;
	struct msgb *msg;
	struct mISDNhead *hh;
	uint8_t *l2_data;
	int ret;

	bfd->when &= ~BSC_FD_WRITE;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		return 0;
	}

	if (mline->use_userspace_lapd) {
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "TX %u/%u/%u: %s\n", line->num, sign_link->tei,
			sign_link->sapi, osmo_hexdump(msg->data, msg->len));
		lapd_transmit(e1i_ts->lapd, sign_link->tei,
				sign_link->sapi, msg);
		ret = 0;
	} else {
		l2_data = msg->data;

		/* prepend the mISDNhead */
		hh = (struct mISDNhead *) msgb_push(msg, sizeof(*hh));
		hh->prim = DL_DATA_REQ;

		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "TX channel(%d) TEI(%d) SAPI(%d): %s\n",
			sign_link->driver.misdn.channel, sign_link->tei,
			sign_link->sapi, osmo_hexdump(l2_data, msg->len - MISDN_HEADER_LEN));

		/* construct the sockaddr */
		sa.family = AF_ISDN;
		sa.sapi = sign_link->sapi;
		sa.dev = sign_link->tei;
		sa.channel = sign_link->driver.misdn.channel;

		ret = sendto(bfd->fd, msg->data, msg->len, 0,
			     (struct sockaddr *)&sa, sizeof(sa));
		if (ret < 0)
			fprintf(stderr, "%s sendto failed %d\n", __func__, ret);

		msgb_free(msg);
	}


	/* set tx delay timer for next event */
	osmo_timer_setup(&e1i_ts->sign.tx_timer, timeout_ts1_write, e1i_ts);
	osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, e1i_ts->sign.delay);

	return ret;
}

/*! \brief call-back from LAPD code, called when it wants to Tx data */
static void misdn_write_msg(struct msgb *msg, void *cbdata)
{
	struct osmo_fd *bfd = cbdata;
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct mISDNhead *hh;
	int ret;

	LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "PH_DATA_REQ: len=%d %s\n", msg->len,
		osmo_hexdump(msg->data, msg->len));

	hh = (struct mISDNhead *) msgb_push(msg, MISDN_HEADER_LEN);
	hh->prim = PH_DATA_REQ;
	hh->id = 0;

	ret = write(bfd->fd, msg->data, msg->len);
	if (ret < 0)
		LOGPITS(e1i_ts, DLMI, LOGL_NOTICE, "write failed %d\n", ret);

	msgb_free(msg);
}

/* write to a B channel TS */
static int handle_tsX_write(struct osmo_fd *bfd, int len)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct mISDNhead *hh;
	uint8_t tx_buf[len + sizeof(*hh)];
	struct subch_mux *mx = &e1i_ts->trau.mux;
	int ret;

	hh = (struct mISDNhead *) tx_buf;
	hh->prim = PH_DATA_REQ;
	hh->id = 0;

	subchan_mux_out(mx, tx_buf+sizeof(*hh), len);

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "BCHAN TX: %s\n", osmo_hexdump(tx_buf+sizeof(*hh), len));

	ret = send(bfd->fd, tx_buf, sizeof(*hh) + len, 0);
	if (ret < sizeof(*hh) + len)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "send returns %d instead of %zu\n", ret, sizeof(*hh) + len);

	return ret;
}

#define TSX_ALLOC_SIZE 4096
/* FIXME: read from a B channel TS */
static int handle_tsX_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TSX_ALLOC_SIZE, "mISDN TSx");
	struct mISDNhead *hh;
	int ret;

	if (!msg)
		return -ENOMEM;

	hh = (struct mISDNhead *) msg->data;

	ret = recv(bfd->fd, msg->data, TSX_ALLOC_SIZE, 0);
	if (ret < 0) {
		fprintf(stderr, "recvfrom error  %s\n", strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);

	if (hh->prim != PH_CONTROL_IND)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "<= BCHAN len = %d, prim(0x%x) id(0x%x): %s\n",
			ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case PH_DATA_IND:
		msg->l2h = msg->data + MISDN_HEADER_LEN;
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "BCHAN RX: %s\n",
			osmo_hexdump(msgb_l2(msg), ret - MISDN_HEADER_LEN));
		/* the number of bytes received indicates that data to send */
		handle_tsX_write(bfd, msgb_l2len(msg));
		return e1inp_rx_ts(e1i_ts, msg, 0, 0);
	case PH_ACTIVATE_IND:
	case PH_DATA_CNF:
		break;
	default:
		break;
	}
	/* FIXME: why do we free signalling msgs in the caller, and trau not? */
	msgb_free(msg);

	return ret;
}

/* write to a raw channel TS */
static int handle_ts_raw_write(struct osmo_fd *bfd, unsigned int len)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	struct mISDNhead *hh;
	int ret;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, NULL);
	if (!msg)
		return 0;

	if (msg->len != len) {
		/* This might lead to a transmit underrun, as we call tx
		 * from the rx path, as there's no select/poll on dahdi
		 * */
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "unexpected msg->len = %u, expected %u\n", msg->len, len);
	}

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN TX: %s\n", osmo_hexdump(msg->data, msg->len));

	hh = (struct mISDNhead *) msgb_push(msg, sizeof(*hh));
	hh->prim = PH_DATA_REQ;
	hh->id = 0;

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
	struct msgb *msg = msgb_alloc(TSX_ALLOC_SIZE, "mISDN Tx RAW");
	struct mISDNhead *hh;
	int ret;

	if (!msg)
		return -ENOMEM;

	hh = (struct mISDNhead *) msg->data;

	ret = recv(bfd->fd, msg->data, TSX_ALLOC_SIZE, 0);
	if (ret < 0) {
		fprintf(stderr, "recvfrom error  %s\n", strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);

	if (hh->prim != PH_CONTROL_IND)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "<= RAW CHAN len = %d, prim(0x%x) id(0x%x): %s\n",
			ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case PH_DATA_IND:
		msg->l2h = msg->data + MISDN_HEADER_LEN;
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN RX: %s\n",
			osmo_hexdump(msgb_l2(msg), ret - MISDN_HEADER_LEN));
		/* the number of bytes received indicates that data to send */
		handle_ts_raw_write(bfd, msgb_l2len(msg));
		return e1inp_rx_ts(e1i_ts, msg, 0, 0);
	case PH_ACTIVATE_IND:
	case PH_DATA_CNF:
		break;
	default:
		break;
	}
	/* FIXME: why do we free signalling msgs in the caller, and trau not? */
	msgb_free(msg);

	return ret;
}

/* callback from select.c in case one of the fd's can be read/written */
static int misdn_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	unsigned int idx = ts_nr-1;
	struct e1inp_ts *e1i_ts = &line->ts[idx];
	int rc = 0;

	switch (e1i_ts->type) {
	case E1INP_TS_TYPE_SIGN:
		if (what & BSC_FD_READ)
			rc = handle_ts1_read(bfd);
		if (what & BSC_FD_WRITE)
			rc = handle_ts1_write(bfd);
		break;
	case E1INP_TS_TYPE_TRAU:
		if (what & BSC_FD_READ)
			rc = handle_tsX_read(bfd);
		/* We never include the mISDN B-Channel FD into the
		 * writeset, since it doesn't support poll() based
		 * write flow control */		
		break;
	case E1INP_TS_TYPE_RAW:
		if (what & BSC_FD_READ)
			rc = handle_ts_raw_read(bfd);
		/* We never include the mISDN B-Channel FD into the
		 * writeset, since it doesn't support poll() based
		 * write flow control */
		break;
	default:
		fprintf(stderr, "unknown E1 TS type %u\n", e1i_ts->type);
		break;
	}

	return rc;
}

static int activate_bchan(struct e1inp_line *line, int ts, int act)
{
	struct mISDNhead hh;
	int ret;
	unsigned int idx = ts-1;
	struct e1inp_ts *e1i_ts = &line->ts[idx];
	struct osmo_fd *bfd = &e1i_ts->driver.misdn.fd;

	fprintf(stdout, "activate bchan\n");
	if (act)
		hh.prim = PH_ACTIVATE_REQ;
	else
		hh.prim = PH_DEACTIVATE_REQ;

	hh.id = MISDN_ID_ANY;
	ret = sendto(bfd->fd, &hh, sizeof(hh), 0, NULL, 0);
	if (ret < 0) {
		fprintf(stdout, "could not send ACTIVATE_RQ %s\n",
			strerror(errno));
	}

	return ret;
}

static int mi_e1_line_update(struct e1inp_line *line);
static int mi_e1_line_update_lapd(struct e1inp_line *line);

struct e1inp_driver misdn_driver = {
	.name = "misdn",
	.want_write = ts_want_write,
	.default_delay = 50000,
	.line_update = &mi_e1_line_update,
};

struct e1inp_driver misdn_lapd_driver = {
	.name = "misdn_lapd",
	.want_write = ts_want_write,
	.default_delay = 50000,
	.line_update = &mi_e1_line_update_lapd,
};

static int mi_e1_setup(struct e1inp_line *line, int release_l2)
{
	struct misdn_line *mline = line->driver_data;
	int ts, ret;

	mline->dummy_dchannel = -1;
	if (mline->use_userspace_lapd) {
		/* Open dummy d-channel in order to use b-channels.
		 * Also it is required to define the mode.
		 */
		if (mline->dummy_dchannel < 0) {
			struct sockaddr_mISDN addr;

			mline->dummy_dchannel = socket(PF_ISDN, SOCK_DGRAM,
							ISDN_P_NT_E1);
			if (mline->dummy_dchannel < 0) {
				fprintf(stderr, "%s could not open socket %s\n",
					__func__, strerror(errno));
				return mline->dummy_dchannel;
			}
			memset(&addr, 0, sizeof(addr));
			addr.family = AF_ISDN;
			addr.dev = line->port_nr;
			addr.channel = 0;
			addr.sapi = 0;
			addr.tei = GROUP_TEI;
			ret = bind(mline->dummy_dchannel,
				(struct sockaddr *) &addr, sizeof(addr));
			if (ret < 0) {
				fprintf(stderr, "could not bind l2 socket %s\n",
					strerror(errno));
				return -EIO;
			}
		}
	}

	/* TS0 is CRC4, don't need any fd for it */
	for (ts = 1; ts < NUM_E1_TS; ts++) {
		unsigned int idx = ts-1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.misdn.fd;
		struct sockaddr_mISDN addr;

		bfd->data = line;
		bfd->priv_nr = ts;
		bfd->cb = misdn_fd_cb;

		switch (e1i_ts->type) {
		case E1INP_TS_TYPE_NONE:
			continue;
			break;
		case E1INP_TS_TYPE_HDLC:
			bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
				ISDN_P_B_HDLC);
			bfd->when = BSC_FD_READ;
			break;
		case E1INP_TS_TYPE_SIGN:
			if (mline->use_userspace_lapd)
				bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
					ISDN_P_B_HDLC);
			else
				bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
					ISDN_P_LAPD_NT);
			bfd->when = BSC_FD_READ;
			break;
		case E1INP_TS_TYPE_TRAU:
		case E1INP_TS_TYPE_I460:
		case E1INP_TS_TYPE_RAW:
			bfd->fd = socket(PF_ISDN, SOCK_DGRAM, ISDN_P_B_RAW);
			/* We never include the mISDN B-Channel FD into the
	 		* writeset, since it doesn't support poll() based
	 		* write flow control */		
			bfd->when = BSC_FD_READ;
			break;
		}

		if (bfd->fd < 0) {
			fprintf(stderr, "%s could not open socket %s\n",
				__func__, strerror(errno));
			return bfd->fd;
		}

		memset(&addr, 0, sizeof(addr));
		addr.family = AF_ISDN;
		addr.dev = line->port_nr;
		switch (e1i_ts->type) {
		case E1INP_TS_TYPE_SIGN:
			if (mline->use_userspace_lapd) {
				char name[32];
				addr.channel = ts;
				e1inp_ts_name(name, sizeof(name), e1i_ts);
				e1i_ts->lapd = lapd_instance_alloc2(1,
					misdn_write_msg, bfd, e1inp_dlsap_up,
					e1i_ts, &lapd_profile_abis, name);
			} else {
				addr.channel = 0;
				/* SAPI not supported yet in kernel */
				//addr.sapi = e1inp_ts->sign.sapi;
				addr.sapi = 0;
				addr.tei = GROUP_TEI;
			}
			break;
		case E1INP_TS_TYPE_HDLC:
		case E1INP_TS_TYPE_TRAU:
			addr.channel = ts;
			break;
		default:
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "unsupported E1 TS type: %u\n", e1i_ts->type);
			break;
		}

		ret = bind(bfd->fd, (struct sockaddr *) &addr, sizeof(addr));
		if (ret < 0) {
			fprintf(stderr, "could not bind l2 socket %s\n",
				strerror(errno));
			return -EIO;
		}

		if (e1i_ts->type == E1INP_TS_TYPE_SIGN) {
			if (!mline->use_userspace_lapd) {
				ret = ioctl(bfd->fd, IMCLEAR_L2, &release_l2);
				if (ret < 0) {
					fprintf(stderr, "could not send IOCTL IMCLEAN_L2 %s\n", strerror(errno));
					return -EIO;
				}
			} else
				activate_bchan(line, ts, 1);
		}

		/* FIXME: only activate B-Channels once we start to
		 * use them to conserve CPU power */
		if (e1i_ts->type == E1INP_TS_TYPE_TRAU)
			activate_bchan(line, ts, 1);

		ret = osmo_fd_register(bfd);
		if (ret < 0) {
			fprintf(stderr, "could not register FD: %s\n",
				strerror(-ret));
			return ret;
		}
	}

	return 0;
}

static int _mi_e1_line_update(struct e1inp_line *line)
{
	struct mISDN_devinfo devinfo;
	int sk, ret, cnt;

	if (line->driver != &misdn_driver &&
	    line->driver != &misdn_lapd_driver)
		return -EINVAL;

	/* open the ISDN card device */
	sk = socket(PF_ISDN, SOCK_RAW, ISDN_P_BASE);
	if (sk < 0) {
		fprintf(stderr, "%s could not open socket %s\n",
			__func__, strerror(errno));
		return sk;
	}

	ret = ioctl(sk, IMGETCOUNT, &cnt);
	if (ret) {
		fprintf(stderr, "%s error getting interf count: %s\n",
			__func__, strerror(errno));
		close(sk);
		return -ENODEV;
	}
	//LOGPIL(line, DLMI, LOGL_DEBUG, "%d device%s found\n", cnt, (cnt==1)?"":"s");
	printf("%d device%s found\n", cnt, (cnt==1)?"":"s");
#if 1
	devinfo.id = line->port_nr;
	ret = ioctl(sk, IMGETDEVINFO, &devinfo);
	if (ret < 0) {
		fprintf(stdout, "error getting info for device %d: %s\n",
			line->port_nr, strerror(errno));
		close(sk);
		return -ENODEV;
	}
	fprintf(stdout, "        id:             %d\n", devinfo.id);
	fprintf(stdout, "        Dprotocols:     %08x\n", devinfo.Dprotocols);
	fprintf(stdout, "        Bprotocols:     %08x\n", devinfo.Bprotocols);
	fprintf(stdout, "        protocol:       %d\n", devinfo.protocol);
	fprintf(stdout, "        nrbchan:        %d\n", devinfo.nrbchan);
	fprintf(stdout, "        name:           %s\n", devinfo.name);
#endif
	close(sk);

	if (!(devinfo.Dprotocols & (1 << ISDN_P_NT_E1))) {
		fprintf(stderr, "error: card is not of type E1 (NT-mode)\n");
		return -EINVAL;
	}

	ret = mi_e1_setup(line, 1);
	if (ret)
		return ret;

	return 0;
}

static int mi_e1_line_update(struct e1inp_line *line)
{
	struct misdn_line *ml;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct misdn_line);

	ml = line->driver_data;
	ml->use_userspace_lapd = 0;

	return _mi_e1_line_update(line);
}

static int mi_e1_line_update_lapd(struct e1inp_line *line)
{
	struct misdn_line *ml;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct misdn_line);

	ml = line->driver_data;
	ml->use_userspace_lapd = 1;

	return _mi_e1_line_update(line);
}

void e1inp_misdn_init(void)
{
	/* register the driver with the core */
	e1inp_driver_register(&misdn_driver);
	e1inp_driver_register(&misdn_lapd_driver);
}

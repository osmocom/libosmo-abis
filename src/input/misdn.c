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
#include <osmocom/core/signal.h>

#define TS1_ALLOC_SIZE	300

/*! \brief driver-specific data for \ref e1inp_line::driver_data */
struct misdn_line {
	int use_userspace_lapd;
	int unconfirmed_ts[NUM_E1_TS];
	enum e1inp_ts_type type_ts[NUM_E1_TS];
	int dummy_dchannel;
	int los, ais, rai;
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

static int mph_information_ind(struct e1inp_line *line, uint32_t info)
{
	struct misdn_line *mline = line->driver_data;
	struct input_signal_data isd;
	unsigned int signal;
	int was_alarm, is_alarm;

	memset(&isd, 0, sizeof(isd));
	isd.line = line;

	was_alarm = mline->los || mline->ais || mline->rai;

	if ((info & ~L1_SIGNAL_SA_MASK) == L1_SIGNAL_SA_BITS) {
		isd.sa_bits = info & L1_SIGNAL_SA_MASK;
		LOGP(DLMI, LOGL_DEBUG, "%s: Sa61/6=%d, Sa62=%d, Sa63=%d, Sa64=%d, Sa4=%d, Sa5=%d, Sa7=%d, Sa8=%d\n",
		     get_value_string(e1inp_signal_names, S_L_INP_LINE_SA_BITS),
		     isd.sa_bits & 1, (isd.sa_bits >> 1) & 1, (isd.sa_bits >> 2) & 1, (isd.sa_bits >> 3) & 1,
		     (isd.sa_bits >> 4) & 1, (isd.sa_bits >> 5) & 1, (isd.sa_bits >> 6) & 1, isd.sa_bits >> 7);
		osmo_signal_dispatch(SS_L_INPUT, S_L_INP_LINE_SA_BITS, &isd);
		return 0;
	}

	switch (info) {
	case L1_SIGNAL_LOS_OFF:
		signal = S_L_INP_LINE_NOLOS;
		mline->los = 0;
		break;
	case L1_SIGNAL_LOS_ON:
		signal = S_L_INP_LINE_LOS;
		mline->los = 1;
		break;
	case L1_SIGNAL_AIS_OFF:
		signal = S_L_INP_LINE_NOAIS;
		mline->ais = 0;
		break;
	case L1_SIGNAL_AIS_ON:
		signal = S_L_INP_LINE_AIS;
		mline->ais = 1;
		break;
	case L1_SIGNAL_RDI_OFF:
		signal = S_L_INP_LINE_NORAI;
		mline->rai = 0;
		break;
	case L1_SIGNAL_RDI_ON:
		signal = S_L_INP_LINE_RAI;
		mline->rai = 1;
		break;
	default:
		LOGP(DLMI, LOGL_DEBUG, "Unknown MPH_INFORMATION_IND: 0x%04x\n", info);
		return -EINVAL;
	}

	LOGP(DLMI, LOGL_DEBUG, "%s\n", get_value_string(e1inp_signal_names, signal));
	osmo_signal_dispatch(SS_L_INPUT, signal, &isd);

	is_alarm = mline->los || mline->ais || mline->rai;

	if (!was_alarm && is_alarm) {
		osmo_signal_dispatch(SS_L_INPUT, S_L_INP_LINE_ALARM, &isd);
		LOGP(DLMI, LOGL_DEBUG, "%s\n", get_value_string(e1inp_signal_names, S_L_INP_LINE_ALARM));
	}
	if (was_alarm && !is_alarm) {
		osmo_signal_dispatch(SS_L_INPUT, S_L_INP_LINE_NOALARM, &isd);
		LOGP(DLMI, LOGL_DEBUG, "%s\n", get_value_string(e1inp_signal_names, S_L_INP_LINE_NOALARM));
	}

	return 0;
}

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
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "recvfrom error: %s\n", strerror(errno));
		msgb_free(msg);
		return ret;
	}

	if (alen != sizeof(l2addr)) {
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "error len\n");
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
	case MPH_INFORMATION_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "MPH_INFORMATION_IND\n");
		/* remove the Misdn Header */
		msgb_pull(msg, MISDN_HEADER_LEN);
		mph_information_ind(line, *((uint32_t *)msg->data));
		msgb_free(msg);
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

	osmo_fd_write_enable(&e1i_ts->driver.misdn.fd);

	return 0;
}

static int set_sa_bits(struct e1inp_line *line, uint8_t sa_bits)
{
	struct e1inp_ts *e1i_ts = &line->ts[16-1];
	struct osmo_fd *bfd = &e1i_ts->driver.misdn.fd;
	uint8_t buffer[sizeof(struct mISDNhead) + sizeof(uint32_t)];
	struct mISDNhead *hh = (struct mISDNhead *)buffer;
	uint32_t *info = (uint32_t *)(buffer + sizeof(struct mISDNhead));
	int ret;

	LOGP(DLMI, LOGL_DEBUG, "MPH_INFORMATION_REQ: Sa4=%d, Sa5=%d, Sa6=%d, Sa7=%d, Sa8=%d\n",
	     (sa_bits >> 4) & 1, (sa_bits >> 5) & 1, sa_bits & 1, (sa_bits >> 6) & 1, sa_bits >> 7);

	hh->prim = MPH_INFORMATION_REQ;
	hh->id = TEI_SAPI | (GROUP_TEI << 8); /* manager address */
	*info = L1_SIGNAL_SA_BITS | sa_bits;

	ret = write(bfd->fd, &buffer, sizeof(buffer));
	if (ret < 0)
		LOGP(DLINP, LOGL_ERROR, "MPH_INFORMATION_REQ returns %d\n", ret);
	return ret;
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

	osmo_fd_write_disable(bfd);

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
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "sendto error: %s\n", strerror(errno));

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
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "recv error: %s\n", strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);

	if (hh->prim != PH_CONTROL_IND)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "<= BCHAN len = %d, prim(0x%x) id(0x%x): %s\n",
			ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case PH_DATA_IND:
		/* remove the Misdn Header */
		msg->l2h = msgb_pull(msg, MISDN_HEADER_LEN);
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "BCHAN RX: %s\n",
			osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));
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
static int handle_ts_raw_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	struct mISDNhead *hh;
	int ret;

	osmo_fd_write_disable(bfd);

	if (mline->unconfirmed_ts[ts_nr-1])
		return 0;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, NULL);
	if (!msg)
		return 0;

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN TX: %s\n", osmo_hexdump(msg->data, msg->len));

	osmo_revbytebits_buf(msg->data, msg->len);
	hh = (struct mISDNhead *) msgb_push(msg, sizeof(*hh));
	hh->prim = PH_DATA_REQ;
	hh->id = 0;

	ret = write(bfd->fd, msg->data, msg->len);
	if (ret < msg->len)
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "send returns %d instead of %d\n", ret, msg->len);
	else
		mline->unconfirmed_ts[ts_nr-1] = 1;
	msgb_free(msg);


	return ret;
}

static int handle_ts_raw_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TSX_ALLOC_SIZE, "mISDN TS RAW");
	struct mISDNhead *hh;
	int ret;

	if (!msg)
		return -ENOMEM;

	hh = (struct mISDNhead *) msg->data;

	ret = recv(bfd->fd, msg->data, TSX_ALLOC_SIZE, 0);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "recv error: %s\n", strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);

	if (hh->prim != PH_CONTROL_IND)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "<= RAW CHAN len = %d, prim(0x%x) id(0x%x): %s\n",
			ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case PH_DATA_IND:
		/* remove the Misdn Header */
		msg->l2h = msgb_pull(msg, MISDN_HEADER_LEN);
		osmo_revbytebits_buf(msg->data, msg->len);
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "RAW CHAN RX: %s\n",
			osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));
		return e1inp_rx_ts(e1i_ts, msg, 0, 0);
	case PH_ACTIVATE_IND:
		break;
	case PH_DATA_CNF:
		mline->unconfirmed_ts[ts_nr-1] = 0;
		osmo_fd_write_enable(bfd);
		break;
	default:
		break;
	}
	/* FIXME: why do we free signalling msgs in the caller, and trau not? */
	msgb_free(msg);

	return ret;
}

/* write to a hdlc channel TS */
static int handle_ts_hdlc_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg;
	struct mISDNhead *hh;
	int ret;

	osmo_fd_write_disable(bfd);

	if (mline->unconfirmed_ts[ts_nr-1])
		return 0;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, NULL);
	if (!msg)
		return 0;

	LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "HDLC CHAN TX: %s\n", osmo_hexdump(msg->data, msg->len));

	hh = (struct mISDNhead *) msgb_push(msg, sizeof(*hh));
	hh->prim = PH_DATA_REQ;
	hh->id = 0;

	ret = write(bfd->fd, msg->data, msg->len);
	if (ret < msg->len)
		LOGPITS(e1i_ts, DLINP, LOGL_NOTICE, "send returns %d instead of %d\n", ret, msg->len);
	else
		mline->unconfirmed_ts[ts_nr-1] = 1;
	msgb_free(msg);


	return ret;
}

static int handle_ts_hdlc_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct misdn_line *mline = line->driver_data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct msgb *msg = msgb_alloc(TSX_ALLOC_SIZE, "mISDN TS HDLC");
	struct mISDNhead *hh;
	int ret;

	if (!msg)
		return -ENOMEM;

	hh = (struct mISDNhead *) msg->data;

	ret = recv(bfd->fd, msg->data, TSX_ALLOC_SIZE, 0);
	if (ret < 0) {
		LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "recv error: %s\n", strerror(errno));
		return ret;
	}

	msgb_put(msg, ret);

	if (hh->prim != PH_CONTROL_IND)
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "<= HDLC CHAN len = %d, prim(0x%x) id(0x%x): %s\n",
			ret, hh->prim, hh->id, get_value_string(prim_names, hh->prim));

	switch (hh->prim) {
	case PH_DATA_IND:
		/* remove the Misdn Header */
		msgb_pull(msg, MISDN_HEADER_LEN);
		msg->l2h = msg->data;
		LOGPITS(e1i_ts, DLMIB, LOGL_DEBUG, "HDLC CHAN RX: %s\n",
			osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));
		return e1inp_rx_ts(e1i_ts, msg, 0, 0);
	case PH_ACTIVATE_IND:
		break;
	case PH_DATA_CNF:
		mline->unconfirmed_ts[ts_nr-1] = 0;
		osmo_fd_write_enable(bfd);
		break;
	case MPH_INFORMATION_IND:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "MPH_INFORMATION_IND\n");
		/* remove the Misdn Header */
		msgb_pull(msg, MISDN_HEADER_LEN);
		mph_information_ind(line, *((uint32_t *)msg->data));
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
		if (what & OSMO_FD_READ)
			rc = handle_ts1_read(bfd);
		if (what & OSMO_FD_WRITE)
			rc = handle_ts1_write(bfd);
		break;
	case E1INP_TS_TYPE_TRAU:
		if (what & OSMO_FD_READ)
			rc = handle_tsX_read(bfd);
		/* We never include the mISDN B-Channel FD into the
		 * writeset, since it doesn't support poll() based
		 * write flow control */		
		break;
	case E1INP_TS_TYPE_RAW:
		if (what & OSMO_FD_READ)
			rc = handle_ts_raw_read(bfd);
		if (what & OSMO_FD_WRITE)
			rc = handle_ts_raw_write(bfd);
		break;
	case E1INP_TS_TYPE_HDLC:
		if (what & OSMO_FD_READ)
			rc = handle_ts_hdlc_read(bfd);
		if (what & OSMO_FD_WRITE)
			rc = handle_ts_hdlc_write(bfd);
		break;
	default:
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "unknown E1 TS type %u\n", e1i_ts->type);
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

	if (act)
		hh.prim = PH_ACTIVATE_REQ;
	else
		hh.prim = PH_DEACTIVATE_REQ;

	hh.id = MISDN_ID_ANY;
	ret = sendto(bfd->fd, &hh, sizeof(hh), 0, NULL, 0);
	if (ret < 0)
		LOGPITS(e1i_ts, DLMI, LOGL_DEBUG, "could not send ACTIVATE_RQ %s\n", strerror(errno));

	return ret;
}

static int mi_e1_line_update(struct e1inp_line *line);
static int mi_e1_line_update_lapd(struct e1inp_line *line);

struct e1inp_driver misdn_driver = {
	.name = "misdn",
	.want_write = ts_want_write,
	.set_sa_bits = set_sa_bits,
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

	/* check if sign type is used */
	for (ts = 1; ts < line->num_ts; ts++) {
		unsigned int idx = ts - 1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		if (e1i_ts->type == E1INP_TS_TYPE_SIGN)
			break;
	}
	if (ts < line->num_ts && mline->use_userspace_lapd) {
		/* Open dummy d-channel in order to use b-channels.
		 * Also it is required to define the mode.
		 */
		if (mline->dummy_dchannel <= 0) {
			struct sockaddr_mISDN addr;

			mline->dummy_dchannel = socket(PF_ISDN, SOCK_DGRAM,
							ISDN_P_NT_E1);
			if (mline->dummy_dchannel < 0) {
				LOGPIL(line, DLMI, LOGL_ERROR, "could not open socket %s\n", strerror(errno));
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
				LOGPIL(line, DLMI, LOGL_ERROR, "could not bind l2 socket %s\n", strerror(errno));
				return -EIO;
			}
		}
	} else {
		/* close dummy d-channel */
		if (mline->dummy_dchannel > 0) {
			close(mline->dummy_dchannel);
			mline->dummy_dchannel = -1;
		}
	}

	LOGPIL(line, DLINP, LOGL_NOTICE, "Line update %d %d %d\n", line->num, line->port_nr, line->num_ts);

	/* TS0 is CRC4, don't need any fd for it */
	for (ts = 1; ts < line->num_ts; ts++) {
		unsigned int idx = ts - 1;
		struct e1inp_ts *e1i_ts = &line->ts[idx];
		struct osmo_fd *bfd = &e1i_ts->driver.misdn.fd;
		struct sockaddr_mISDN addr;

		/* no change in type */
		if (mline->type_ts[idx] == e1i_ts->type)
			continue;

		LOGPIL(line, DLINP, LOGL_INFO, "Time slot %d changes from type %s to type %s\n", ts,
		       e1inp_tstype_name(mline->type_ts[idx]), e1inp_tstype_name(e1i_ts->type));

		/* unregister FD if it was already registered */
		if (bfd->list.next && bfd->list.next != LLIST_POISON1)
			osmo_fd_unregister(bfd);

		/* close/release LAPD instance, if any */
		if (e1i_ts->lapd) {
			lapd_instance_free(e1i_ts->lapd);
			e1i_ts->lapd = NULL;
		}

		/* close old file descriptor */
		if (bfd->fd) {
			close(bfd->fd);
			bfd->fd = 0;
		}

		/* clear 'unconfirmed' state */
		mline->unconfirmed_ts[idx] = 0;

		/* set new type */
		mline->type_ts[idx] = e1i_ts->type;

		switch (e1i_ts->type) {
		case E1INP_TS_TYPE_NONE:
			/* keep closed */
			continue;
		case E1INP_TS_TYPE_HDLC:
			/* open hdlc socket */
			/* TS 16 is the D-channel, so we use D-channel proto */
			bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
				(ts == 16) ? ISDN_P_NT_E1 : ISDN_P_B_HDLC);
			break;
		case E1INP_TS_TYPE_SIGN:
			/* open and allocate user space lapd, if required */
			if (mline->use_userspace_lapd)
				bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
					ISDN_P_B_HDLC);
			else
				bfd->fd = socket(PF_ISDN, SOCK_DGRAM,
					ISDN_P_LAPD_NT);
			break;
		case E1INP_TS_TYPE_TRAU:
		case E1INP_TS_TYPE_I460:
		case E1INP_TS_TYPE_RAW:
			/* open raw socket */
			bfd->fd = socket(PF_ISDN, SOCK_DGRAM, ISDN_P_B_RAW);
			break;
		}

		/* failed to open? */
		if (bfd->fd < 0) {
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "could not open socket: %s\n", strerror(errno));
			return bfd->fd;
		}

		/* configure mISDN socket, also add user space lapd, if enabled */
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
		case E1INP_TS_TYPE_RAW:
		case E1INP_TS_TYPE_TRAU:
			/* TS 16 is D-channel, so we use channel 0 */
			addr.channel = (ts == 16) ? 0 : ts;
			break;
		default:
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "unsupported E1 TS type: %u\n", e1i_ts->type);
			break;
		}

		/* bind mISDN socket */
		ret = bind(bfd->fd, (struct sockaddr *) &addr, sizeof(addr));
		if (ret < 0) {
			LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "could not bind l2 socket %s\n",
				strerror(errno));
			return -EIO;
		}

		/* release L2 in case of kernel space lapd, activate b-channel in other cases */
		if (e1i_ts->type == E1INP_TS_TYPE_SIGN) {
			if (!mline->use_userspace_lapd) {
				ret = ioctl(bfd->fd, IMCLEAR_L2, &release_l2);
				if (ret < 0) {
					LOGPITS(e1i_ts, DLMI, LOGL_ERROR, "could not send IOCTL IMCLEAN_L2 %s\n",
						strerror(errno));
					return -EIO;
				}
			} else
				activate_bchan(line, ts, 1);
		} else
			activate_bchan(line, ts, 1);

		/* register file descriptor with read set enabled */
		osmo_fd_setup(bfd, bfd->fd, OSMO_FD_READ, misdn_fd_cb, line, ts);
		ret = osmo_fd_register(bfd);
		if (ret < 0) {
			LOGPITS(e1i_ts, DLINP, LOGL_ERROR, "could not register FD: %s\n", strerror(ret));
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
		LOGPIL(line, DLMI, LOGL_ERROR, "error, could not open socket: %s\n", strerror(errno));
		return sk;
	}

	ret = ioctl(sk, IMGETCOUNT, &cnt);
	if (ret) {
		LOGPIL(line, DLMI, LOGL_ERROR, "error getting interface count: %s\n", strerror(errno));
		close(sk);
		return -ENODEV;
	}
	LOGPIL(line, DLMI, LOGL_INFO, "%d device%s found\n", cnt, (cnt == 1) ? "" : "s");
#if 1
	memset(&devinfo, 0, sizeof(devinfo));
	devinfo.id = line->port_nr;
	ret = ioctl(sk, IMGETDEVINFO, &devinfo);
	if (ret < 0) {
		LOGPIL(line, DLMI, LOGL_ERROR, "error getting info for device %d: %s\n",
		       line->port_nr, strerror(errno));
		close(sk);
		return -ENODEV;
	}
	LOGPIL(line, DLMI, LOGL_INFO, "        id:             %d\n", devinfo.id);
	LOGPIL(line, DLMI, LOGL_INFO, "        Dprotocols:     %08x\n", devinfo.Dprotocols);
	LOGPIL(line, DLMI, LOGL_INFO, "        Bprotocols:     %08x\n", devinfo.Bprotocols);
	LOGPIL(line, DLMI, LOGL_INFO, "        protocol:       %d\n", devinfo.protocol);
	LOGPIL(line, DLMI, LOGL_INFO, "        nrbchan:        %d\n", devinfo.nrbchan);
	LOGPIL(line, DLMI, LOGL_INFO, "        name:           %s\n", devinfo.name);
#endif
	close(sk);

	if (!(devinfo.Dprotocols & (1 << ISDN_P_NT_E1))) {
		LOGPIL(line, DLMI, LOGL_ERROR, "error, card is not of type E1 (NT-mode)\n");
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

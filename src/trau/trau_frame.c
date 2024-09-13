/* New (2020) TRAU frame handling according to GSM TS 48.060 + 48.061 */

/* (C) 2020 by Harald Welte <laforge@gnumonks.org>
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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <osmocom/trau/trau_frame.h>
#include <osmocom/core/logging.h>

/*! \addtogroup trau_frame
 *  @{
 *
 *  \file trau_frame.c
 */

static uint32_t get_bits(const ubit_t *bitbuf, int offset, int num)
{
	int i;
	uint32_t ret = 0;

	for (i = offset; i < offset + num; i++) {
		ret = ret << 1;
		if (bitbuf[i])
			ret |= 1;
	}
	return ret;
}

const struct value_string osmo_trau_frame_type_names[] = {
	{ OSMO_TRAU_FT_NONE,		"NONE" },
	{ OSMO_TRAU16_FT_FR,		"FR" },
	{ OSMO_TRAU16_FT_HR,		"HR" },
	{ OSMO_TRAU16_FT_EFR,		"EFR" },
	{ OSMO_TRAU16_FT_AMR,		"AMR" },
	{ OSMO_TRAU16_FT_OAM,		"OAM" },
	{ OSMO_TRAU16_FT_DATA,		"DATA" },
	{ OSMO_TRAU16_FT_EDATA,		"EDATA" },
	{ OSMO_TRAU16_FT_D145_SYNC,	"D145_SYNC" },
	{ OSMO_TRAU16_FT_DATA_HR,	"DATA_HR" },
	{ OSMO_TRAU16_FT_IDLE,		"IDLE" },
	{ OSMO_TRAU8_SPEECH,		"8SPEECH" },
	{ OSMO_TRAU8_DATA,		"8DATA" },
	{ OSMO_TRAU8_OAM,		"8OAM" },
	{ OSMO_TRAU8_AMR_LOW,		"8AMR_LOW" },
	{ OSMO_TRAU8_AMR_6k7,		"8AMR_6k7" },
	{ OSMO_TRAU8_AMR_7k4,		"8AMR_7k4" },
	{ 0, NULL }
};

/*********************************************************************************
 * New API; introduced in 2020 for use by osmo-mgw
 *********************************************************************************/

/* Bits C21..C22 (16k) or C4..C5 (8k) */
enum ts48060_amr_frame_classification {
	TS48060_AMR_FC_SPEECH_GOOD	= 0x3,
	TS48060_AMR_FC_SPEECH_DEGRADED	= 0x2,
	TS48060_AMR_FC_SPEECH_BAD	= 0x1,
	TS48060_AMR_FC_NO_SPEECH	= 0x0,
};

/* GSM 08.60 frame types, 16k sub-slots */
static const ubit_t ft_fr_up_bits[5] =		{ 0, 0, 0, 1, 0 };
static const ubit_t ft_fr_down_bits[5] =	{ 1, 1, 1, 0, 0 };
static const ubit_t ft_data_up_bits[5] =	{ 0, 1, 0, 0, 0 };
static const ubit_t ft_data_down_bits[5] =	{ 1, 0, 1, 1, 0 };
static const ubit_t ft_idle_up_bits[5] =	{ 1, 0, 0, 0, 0 };
static const ubit_t ft_idle_down_bits[5] =	{ 0, 1, 1, 1, 0 };
static const ubit_t ft_efr_bits[5] =		{ 1, 1, 0, 1, 0 };
static const ubit_t ft_amr_bits[5] =		{ 0, 0, 1, 1, 0 };
static const ubit_t ft_oam_up_bits[5] = 	{ 0, 0, 1, 0, 1 };
static const ubit_t ft_oam_down_bits[5] =	{ 1, 1, 0, 1, 1 };
static const ubit_t ft_d145s_bits[5] =		{ 1, 0, 1, 0, 0 };
static const ubit_t ft_edata_bits[5] =		{ 1, 1, 1, 1, 1 };

/* GSM 08.61 frame types, still on 16k sub-slots */
static const ubit_t ft_hr_up_bits[5] =		{ 0, 0, 0, 1, 1 };
static const ubit_t ft_hr_down_bits[5] =	{ 1, 1, 1, 0, 1 };
static const ubit_t ft_data_hr_up_bits[5] =	{ 0, 1, 0, 0, 1 };
static const ubit_t ft_data_hr_down_bits[5] =	{ 1, 0, 1, 1, 1 };


/* generate the sync pattern described in TS 08.60 4.8.1 */
static void encode_sync16(ubit_t *trau_bits)
{
	int i;

	/* 16 '0' bits in  header */
	memset(trau_bits, 0, 16);
	/* '1'-bit in regular intervals */
	for (i = 16; i < 40*8; i += 16)
		trau_bits[i] = 1;
}

#define T16_500us	8	/* 500 us = 8 bits */
#define T16_250us	4	/* 250 us = 4 bits */

#define T8_250us	2	/* 250 us = 2 bits */
#define T8_125us	1	/* 125 us = 1 bits */

/* How many 16k bits to delay / advance (TS 48.060 Section 5.5.1.1.1) */
static int trau_frame_16_ta_us(uint8_t c6_11)
{
	if (c6_11 <= 0x27)
		return c6_11 * 500;	/* delay frame N x 500us */
	else if (c6_11 == 0x3e)
		return 250;		/* delay frame 250us */
	else if (c6_11 == 0x3f)
		return -250;	/* advance frame 250us */
	else
		return 0;
}

/* How many 8k bits to delay / advance (TS 48.061 Table 6.1) */
static int trau_frame_8_ta_us(uint8_t c6_8)
{
	/* TA2..TA1..TA0 == C6..C7..C8 */
	switch (c6_8) {
	case 0x7:
		return 0;
	case 0x6:
		return -250;
	case 0x5:
		return 250;
	case 0x3:
		return 500;
	case 0x4:
		return 1000;
	case 0x2:
		return 2000;
	case 0x1:
		return 6000;
	case 0:
		return 9000;
	default:
		return 0;
	}
}

/*! Determine the time alignment in us requested by CCU in a UL frame */
int osmo_trau_frame_dl_ta_us(const struct osmo_trau_frame *fr)
{
	uint8_t c6_11, c6_8, fc;

	/* timing alignment is only communicated from CCU to TRAU in UL */
	if (fr->dir == OSMO_TRAU_DIR_DL)
		return 0;

	switch (fr->type) {
	case OSMO_TRAU16_FT_FR:
	case OSMO_TRAU16_FT_EFR:
	case OSMO_TRAU16_FT_HR:
	case OSMO_TRAU16_FT_AMR:
		c6_11 = (fr->c_bits[5] << 5) | (fr->c_bits[6] << 4) | (fr->c_bits[7] << 3) |
			(fr->c_bits[8] << 2) | (fr->c_bits[9] << 1) | (fr->c_bits[10] << 0);
		return trau_frame_16_ta_us(c6_11);
		break;

	case OSMO_TRAU8_SPEECH:
		c6_8 = (fr->c_bits[5] << 2) | (fr->c_bits[6] << 1) | (fr->c_bits[7] << 0);
		return trau_frame_8_ta_us(c6_8);
		break;

	case OSMO_TRAU8_AMR_LOW:
		fc = (fr->c_bits[3] << 1) | (fr->c_bits[4] << 0);
		if (fc == TS48060_AMR_FC_NO_SPEECH) {
			c6_11 = (fr->c_bits[5] << 5) | (fr->c_bits[6] << 4) | (fr->c_bits[7] << 3) |
				(fr->c_bits[8] << 2) | (fr->c_bits[9] << 1) | (fr->c_bits[10] << 0);
			/* For AMR speech on 8 kBit/s submultiplexing the same procedures as for AMR
			 * speech on 16 kBit/s submultiplexing shall be applied, see 3GPP TS 48.060 */
			return trau_frame_16_ta_us(c6_11);
		} else
			return 0;
		break;

	default:
		return 0;
	}
}

static int encode16_handle_ta(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	if (fr->dir == OSMO_TRAU_DIR_DL) {
		int num_bits = fr->dl_ta_usec * T16_250us / 250;
		if (num_bits > 0) {
			if (num_bits > 39 * 8)
				num_bits = 39;
			memset(trau_bits + 40 * 8, 1, num_bits);
			return 40 * 8 + num_bits;
		} else if (num_bits < 0) {
			if (num_bits < -1 * T16_250us)
				num_bits = -1 * T16_250us;
			return 40 * 8 + num_bits;
		}
	}
	return 40 * 8;
}

static int encode8_handle_ta(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	if (fr->dir == OSMO_TRAU_DIR_DL) {
		int num_bits = fr->dl_ta_usec * T8_250us / 250;
		if (num_bits > 0) {
			if (num_bits > 20 * 8 - 2)
				num_bits = 20 * 8 - 2;
			memset(trau_bits + 20 * 8, 1, num_bits);
			return 20 * 8 + num_bits;
		} else if (num_bits < 0) {
			if (num_bits < -1 * T8_250us)
				num_bits = -2;
			return 20 * 8 + num_bits;
		}
	}
	return 20 * 8;
}

/* TS 08.60 Section 3.1.1 */
static int encode16_fr(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5;
	int i;
	int d_idx = 0;

	switch (fr->type) {
	case OSMO_TRAU16_FT_IDLE:
		if (fr->dir == OSMO_TRAU_DIR_UL)
			cbits5 = ft_idle_up_bits;
		else
			cbits5 = ft_idle_down_bits;
		break;
	case OSMO_TRAU16_FT_FR:
		if (fr->dir == OSMO_TRAU_DIR_UL)
			cbits5 = ft_fr_up_bits;
		else
			cbits5 = ft_fr_down_bits;
		break;
	case OSMO_TRAU16_FT_EFR:
		cbits5 = ft_efr_bits;
		break;
	default:
		return -EINVAL;
	}

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5 + 0, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 15 - 5);
	/* D1 .. D255 */
	for (i = 32; i < 304; i += 16) {
		trau_bits[i] = 1;
		memcpy(trau_bits + i + 1, fr->d_bits + d_idx, 15);
		d_idx += 15;
	}
	/* D256 .. D260 */
	trau_bits[304] = 1;
	memcpy(trau_bits + 305, fr->d_bits + d_idx, 5);
	/* C16 .. C21 */
	memcpy(trau_bits + 310, fr->c_bits + 15, 6);

	/* T1 .. T4 */
	memcpy(trau_bits + 316, fr->t_bits+0, 4);

	/* handle timing adjustment */
	return encode16_handle_ta(trau_bits, fr);
}

/* TS 08.60 Section 3.1.1 */
static int decode16_fr(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			enum osmo_trau_frame_direction dir)
{
	int i;
	int d_idx = 0;

	/* C1 .. C15 */
	memcpy(fr->c_bits + 0, trau_bits + 17, 15);
	/* C16 .. C21 */
	memcpy(fr->c_bits + 15, trau_bits + 310, 6);
	/* T1 .. T4 */
	memcpy(fr->t_bits + 0, trau_bits + 316, 4);
	/* D1 .. D255 */
	for (i = 32; i < 304; i+= 16) {
		memcpy(fr->d_bits + d_idx, trau_bits + i + 1, 15);
		d_idx += 15;
	}
	/* D256 .. D260 */
	memcpy(fr->d_bits + d_idx, trau_bits + 305, 5);

	return 0;
}

/* TS 08.60 Section 3.1.2 */
static int encode16_amr(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5 = ft_amr_bits;
	int i, d_idx;

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5 + 0, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 15 - 5);

	trau_bits[32] = 1;
	/* C16 .. C25 */
	memcpy(trau_bits + 33, fr->c_bits + 15, 10);
	/* D1 .. D5 */
	memcpy(trau_bits + 43, fr->d_bits + 0, 5);

	/* D6 .. D256 */
	for (i = 48, d_idx = 5; i <= 315; i += 16, d_idx += 15) {
		trau_bits[i] = 1;
		memcpy(trau_bits + i + 1, fr->d_bits + d_idx, 15);
	}

	/* T1 .. T4 */
	memcpy(trau_bits + 316, fr->t_bits + 0, 4);

	return encode16_handle_ta(trau_bits, fr);

	/* FIXME: handle TAE (Timing Alignment Extension) */
}

/* TS 08.60 Section 3.1.2 */
static int decode16_amr(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			enum osmo_trau_frame_direction dir)
{
	int i;
	int d_idx = 0;

	/* C1 .. C15 */
	memcpy(fr->c_bits + 0, trau_bits + 17, 15);
	/* C16 .. C25 */
	memcpy(fr->c_bits + 15, trau_bits + 33, 10);
	/* T1 .. T4 */
	memcpy(fr->t_bits + 0, trau_bits + 316, 4);
	/* D1 .. D5 */
	memcpy(fr->d_bits, trau_bits + 43, 5);
	d_idx += 5;
	/* D6 .. D245 */
	for (i = 48; i < 304; i += 16) {
		memcpy(fr->d_bits + d_idx, trau_bits + i + 1, 15);
		d_idx += 15;
	}
	/* D246 .. D256 */
	memcpy(fr->d_bits + d_idx, trau_bits + 305, 11);

	return 0;
}

/* TS 08.60 Section 3.2 */
static int encode16_oam(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5;
	int i, d_idx;

	if (fr->dir == OSMO_TRAU_DIR_UL)
		cbits5 = ft_oam_up_bits;
	else
		cbits5 = ft_oam_down_bits;

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits, 15 - 5);

	/* D1 .. D255 */
	for (i = 32, d_idx = 0; i < 304; i += 16, d_idx += 15) {
		trau_bits[i] = 1;
		memcpy(trau_bits + i + 1, fr->d_bits + d_idx, 15);
	}
	/* D256 .. D264 */
	memcpy(trau_bits + 305, fr->d_bits + 256, 9);

	/* S1 .. S6 */
	memcpy(trau_bits + 314, fr->s_bits, 6);

	return 40 * 8;
}

/* TS 08.60 Section 3.2 */
static int decode16_oam(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			enum osmo_trau_frame_direction dir)
{
	int i, d_idx;

	/* C1 .. C15 */
	memcpy(fr->c_bits + 0, trau_bits + 17, 15);

	/* D1 .. D255 */
	for (i = 33, d_idx = 0; i < 312; i+= 16, d_idx += 15)
		memcpy(fr->d_bits + d_idx, trau_bits + i + 1, 15);
	/* D256 .. D264 */
	memcpy(fr->d_bits+d_idx, trau_bits + 305, 9);

	/* S1 .. S6 */
	memcpy(fr->s_bits, trau_bits + 314, 6);

	return 0;
}

/* TS 08.61 Section 5.1.1.1 */
static int encode16_hr(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int d_idx = 0;
	const ubit_t *cbits5;

	if (fr->dir == OSMO_TRAU_DIR_UL)
		cbits5 = ft_hr_up_bits;
	else
		cbits5 = ft_hr_down_bits;

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 10);

	/*  UFI */
	trau_bits[33] = fr->ufi;

	/* D1 .. D14 */
	memcpy(trau_bits + 4 * 8 + 2, fr->d_bits+d_idx, 14); d_idx += 14;
	/* D15 .. D29 */
	memcpy(trau_bits + 6 * 8 + 1, fr->d_bits+d_idx, 15); d_idx += 15;
	/* D30 .. D44 */
	memcpy(trau_bits + 8 * 8 + 1, fr->d_bits+d_idx, 15); d_idx += 15;
	/* CRC */
	memcpy(trau_bits + 10 * 8 + 1, fr->crc_bits, 3);
	/* D45 .. D56 */
	memcpy(trau_bits + 10 * 8 + 4, fr->d_bits+d_idx, 12); d_idx += 12;
	/* D57 .. D71 */
	memcpy(trau_bits + 12 * 8 + 1, fr->d_bits+d_idx, 15); d_idx += 15;
	/* D72 .. D86 */
	memcpy(trau_bits + 14 * 8 + 1, fr->d_bits+d_idx, 15); d_idx += 15;
	/* D87 .. D101 */
	memcpy(trau_bits + 16 * 8 + 1, fr->d_bits+d_idx, 15); d_idx += 15;
	/* D102 .. D112 */
	memcpy(trau_bits + 18 * 8 + 1, fr->d_bits+d_idx, 11); d_idx += 11;

	memset(trau_bits + 19 * 8 + 4, 0x01, 4 + 18 * 8 + 6);
	/* C16 .. C21 */
	memcpy(trau_bits + 38 * 8 + 6, fr->c_bits + 15, 6);

	/* T1 .. T4 */
	memcpy(trau_bits + 39 * 8 + 4, fr->t_bits, 4);

	/* handle timing adjustment */
	return encode16_handle_ta(trau_bits, fr);
}

/* TS 08.61 Section 5.1.1.1 */
static int decode16_hr(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			enum osmo_trau_frame_direction dir)
{
	int d_idx = 0;

	/* C1 .. C15 */
	memcpy(fr->c_bits + 0, trau_bits + 17, 15);
	/* C16 .. C21 */
	memcpy(fr->c_bits + 15, trau_bits + 38 * 8 + 6, 6);
	/* T1 .. T4 */
	memcpy(fr->t_bits + 0, trau_bits + 39 * 8 + 4, 4);

	/* UFI */
	fr->ufi = trau_bits[33];
	/* D1 .. D14 */
	memcpy(fr->d_bits + d_idx, trau_bits + 4 * 8 + 2, 14); d_idx += 14;
	/* D15 .. D29 */
	memcpy(fr->d_bits + d_idx, trau_bits + 6 * 8 + 1, 15); d_idx += 15;
	/* D30 .. D44 */
	memcpy(fr->d_bits + d_idx, trau_bits + 8 * 8 + 1, 15); d_idx += 15;
	/* CRC0..2 */
	memcpy(fr->crc_bits, trau_bits + 10 * 8 + 1, 3);
	/* D45 .. D56 */
	memcpy(fr->d_bits + d_idx, trau_bits + 10 * 8 + 4, 12); d_idx += 12;
	/* D57 .. D71 */
	memcpy(fr->d_bits + d_idx, trau_bits + 12 * 8 + 1, 15); d_idx += 15;
	/* D72 .. D86 */
	memcpy(fr->d_bits + d_idx, trau_bits + 14 * 8 + 1, 15); d_idx += 15;
	/* D87 .. D101 */
	memcpy(fr->d_bits + d_idx, trau_bits + 16 * 8 + 1, 15); d_idx += 15;
	/* D102 .. D112 */
	memcpy(fr->d_bits + d_idx, trau_bits + 18 * 8 + 1, 11); d_idx += 11;

	return 0;
}

static struct osmo_trau_frame fr_idle_frame = {
	.type = OSMO_TRAU16_FT_IDLE,
	.dir = OSMO_TRAU_DIR_DL,
	.t_bits = { 1, 1, 1, 1 },
};
#define TRAU_FRAME_BITS (40*8)
static ubit_t encoded_idle_frame[TRAU_FRAME_BITS];
static int dbits_initted = 0;

/*! \brief return pointer to global buffer containing a TRAU idle frame */
static ubit_t *trau_idle_frame(void)
{
	/* only initialize during the first call */
	if (!dbits_initted) {
		/* set all D-bits to 1 */
		memset(&fr_idle_frame.d_bits, 0x01, 260);

		memset(&fr_idle_frame.c_bits, 0x01, 25); /* spare are set to 1 */
		/* set Downlink Idle Speech Frame pattern */
		fr_idle_frame.c_bits[0] = 0; /* C1 */
		fr_idle_frame.c_bits[1] = 1; /* C2 */
		fr_idle_frame.c_bits[2] = 1; /* C3 */
		fr_idle_frame.c_bits[3] = 1; /* C4 */
		fr_idle_frame.c_bits[4] = 0; /* C5 */
		/* set no Time Alignment pattern */
		fr_idle_frame.c_bits[5] = 0; /* C6 */
		fr_idle_frame.c_bits[6] = 0; /* C7 */
		fr_idle_frame.c_bits[7] = 0; /* C8 */
		fr_idle_frame.c_bits[8] = 0; /* C9 */
		fr_idle_frame.c_bits[9] = 0; /* C10 */
		fr_idle_frame.c_bits[10] = 0; /* C11 */
		/* already set to 1, but maybe we need to modify it in the future */
		fr_idle_frame.c_bits[11] = 1; /* C12 (UFE), good frame */
		fr_idle_frame.c_bits[15] = 1; /* C16 (SP), no DTX */

		encode16_fr(encoded_idle_frame, &fr_idle_frame);
		dbits_initted = 1; /* set it to 1 to not call it again */
	}
	return encoded_idle_frame;
}

/* TS 08.60 Section 3.4 */
static int encode16_idle(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	memcpy(trau_bits, trau_idle_frame(), 40*8);
	return 40 * 8;
}

/* TS 08.60 Section 3.3.1 */
static int decode16_data(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			 enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* C1 .. C15 */
	memcpy(fr->c_bits + 0, trau_bits + 17, 15);

	/* D1 .. D63 */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (4 + i) * 8 + 1, 7);
		d_idx += 7;
	}
	/* D64 .. D126 */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (13 + i) * 8 + 1, 7);
		d_idx += 7;
	}

	/* D127 .. D189 */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (22 + i) * 8 + 1, 7);
		d_idx += 7;
	}

	/* D190 .. D252 */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (31 + i) * 8 + 1, 7);
		d_idx += 7;
	}

	return 0;
}

/* TS 08.60 Section 3.3.1 */
static int encode16_data(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5;
	int i, d_idx = 0;

	if (fr->dir == OSMO_TRAU_DIR_UL)
		cbits5 = ft_data_up_bits;
	else
		cbits5 = ft_data_down_bits;

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 15 - 5);

	/* D1 .. D63 */
	for (i = 0; i < 9; i++) {
		unsigned int offset = (4 + i) * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	/* D64 .. D126 */
	for (i = 0; i < 9; i++) {
		unsigned int offset = (13 + i) * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	/* D127 .. D189 */
	for (i = 0; i < 9; i++) {
		unsigned int offset = (22 + i) * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	/* D190 .. D252 */
	for (i = 0; i < 9; i++) {
		unsigned int offset = (31 + i) * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	return 40 * 8;
}

/* TS 08.60 3.3.2 */
static int decode16_edata(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			  enum osmo_trau_frame_direction dir)
{
	/* C1 .. C13 */
	memcpy(fr->c_bits, trau_bits + 17, 13);

	/* M1 .. M2 */
	memcpy(fr->m_bits, trau_bits + 30, 2);

	/* D1 .. D288 */
	memcpy(fr->d_bits, trau_bits + 4 * 8, 288);

	return 0;
}

/* TS 08.60 Section 3.3.2 */
static int encode16_edata(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5;

	if (fr->type == OSMO_TRAU16_FT_D145_SYNC)
		cbits5 = ft_d145s_bits;
	else
		cbits5 = ft_edata_bits;

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5, 5);

	/* C6 .. C13 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 8);

	/* M1 .. M2 */
	memcpy(trau_bits + 30, fr->m_bits, 2);

	/* D1 .. D288 */
	memcpy(trau_bits + 4 * 8, fr->d_bits, 288);

	return 40 * 8;
}

/* TS 08.61 Section 5.1.2 */
static int decode16_data_hr(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			  enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* C1 .. C15 */
	memcpy(fr->c_bits+0, trau_bits + 17, 15);

	/* D1 .. D63 */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (4 + i) * 8 + 1, 7);
		d_idx += 7;
	}

	/* D'1 .. D'63 (mapped to D64..D127) */
	for (i = 0; i < 9; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + (22 + i) * 8 + 1, 7);
		d_idx += 7;
	}

	return 0;
}

/* TS 08.61 Section 5.1.2 */
static int encode16_data_hr(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	const ubit_t *cbits5;
	int i, d_idx = 0;

	if (fr->dir == OSMO_TRAU_DIR_UL)
		cbits5 = ft_data_hr_up_bits;
	else
		cbits5 = ft_data_hr_down_bits;

	encode_sync16(trau_bits);

	/* C1 .. C5 */
	memcpy(trau_bits + 17, cbits5, 5);
	/* C6 .. C15 */
	memcpy(trau_bits + 17 + 5, fr->c_bits + 5, 15 - 5);

	/* D1 .. D63 */
	for (i = 4; i < 4 + 9; i++) {
		unsigned int offset = i * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	memset(trau_bits + 13*8, 1, 9*8);

	/* D'1 .. D'63 (mapped to D64..D127) */
	for (i = 22; i < 22 + 9; i++) {
		unsigned int offset = i * 8;
		trau_bits[offset] = 1;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	memset(trau_bits + 31*8, 1, 9*8);

	return 40 * 8;
}

/* TS 08.61 Section 5.2.1.1 */
static int decode8_hr(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
		      enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* C1 .. C5 */
	memcpy(fr->c_bits, trau_bits + 9, 5);
	/* XC1 .. XC2 */
	memcpy(fr->xc_bits, trau_bits + 8 + 6, 2);
	/* XC3 .. XC6 */
	memcpy(fr->xc_bits + 2, trau_bits + 2 * 8 + 2, 4);
	/* D1 .. D2 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 6, 2);
	d_idx += 2;

	/* D1 .. D44 */
	for (i = 3; i < 3 + 6; i++) {
		int offset = i * 8;
		memcpy(fr->d_bits + d_idx, trau_bits + offset + 1, 7);
		d_idx += 7;
	}

	/* CRC0 .. CRC2 */
	memcpy(fr->crc_bits, trau_bits + 73, 3);

	/* D45 .. D48 */
	memcpy(fr->d_bits + d_idx, trau_bits + 76, 4);
	d_idx += 4;

	/* D49 .. D111 */
	for (i = 10; i < 10 + 9; i++) {
		int offset = i * 8;
		memcpy(fr->d_bits + d_idx, trau_bits + offset + 1, 7);
		d_idx += 7;
	}
	/* D112 */
	fr->d_bits[d_idx++] = trau_bits[19 * 8 + 1];

	/* C6 .. C9 */
	memcpy(fr->c_bits + 5, trau_bits + 19 * 8 + 2, 4);

	/* T1 .. T2 */
	fr->t_bits[0] = trau_bits[19 * 8 + 6];
	fr->t_bits[1] = trau_bits[19 * 8 + 7];

	return 0;
}

/* compute the odd parity bit of the given input bit sequence */
static ubit_t compute_odd_parity(const ubit_t *in, unsigned int num_bits)
{
	int i;
	unsigned int sum = 0;

	for (i = 0; i < num_bits; i++) {
		if (in[i])
			sum++;
	}

	if (sum & 1)
		return 0;
	else
		return 1;
}

/* TS 08.61 Section 5.2.1.1 */
static int encode8_hr(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int i, d_idx = 0;

	/* sync pattern */
	memset(trau_bits, 0, 8);
	trau_bits[8] = 1;
	trau_bits[16] = 0;
	trau_bits[17] = 1;
	for (i = 3; i < 20; i++)
		trau_bits[i * 8] = 1;

	/* C1 .. C5 */
	ubit_t *cbits_out = trau_bits + 1 * 8 + 1;
	if (fr->dir == OSMO_TRAU_DIR_UL) {
		cbits_out[0] = 0;
		cbits_out[1] = 0;
		cbits_out[2] = 0;
		cbits_out[3] = 1;
		cbits_out[4] = 0;
	} else {
		cbits_out[0] = 0;
		cbits_out[1] = 0;
		cbits_out[2] = 0;
		cbits_out[3] = fr->c_bits[3];
		cbits_out[4] = compute_odd_parity(cbits_out, 4);
	}

	/* XC1 .. XC2 */
	memcpy(trau_bits + 1 * 8 + 6, fr->xc_bits, 2);
	/* XC3 .. XC6 */
	memcpy(trau_bits + 2 * 8 + 2, fr->xc_bits + 2, 4);
	/* D1 ..  D2 */
	memcpy(trau_bits + 2 * 8 + 6, fr->d_bits, 2);
	d_idx += 2;

	/* D1 .. D44 */
	for (i = 3; i < 3 + 6; i++) {
		int offset = i * 8;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	};

	/* CRC0 .. CRC2 */
	memcpy(trau_bits + 73, fr->crc_bits, 3);

	/* D45 .. D48 */
	memcpy(trau_bits + 76, fr->d_bits + d_idx, 4);
	d_idx += 4;

	/* D49 .. D111 */
	for (i = 10; i < 10 + 9; i++) {
		int offset = i * 8;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}
	/* D112 */
	trau_bits[19 * 8 + 1] = fr->d_bits[d_idx++];

	/* C6 .. C9 */
	memcpy(trau_bits + 19 * 8 + 2, fr->c_bits + 5, 4);

	/* T1 .. T2 */
	trau_bits[19 * 8 + 6] = fr->t_bits[0];
	trau_bits[19 * 8 + 7] = fr->t_bits[1];

	/* handle timing adjustment */
	return encode8_handle_ta(trau_bits, fr);
}

/* TS 08.61 Section 5.2.1.2.1 */
static int decode8_amr_low(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			   enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* D1 .. D7 */
	memcpy(fr->d_bits + d_idx, trau_bits + 1 * 8 + 1, 7);
	d_idx += 7;
	/* C1 .. C5 */
	memcpy(fr->c_bits, trau_bits + 2 * 8 + 1, 5);
	/* D8 .. D9 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 6, 2);
	d_idx += 2;
	/* D10 .. D15 */
	memcpy(fr->d_bits + d_idx, trau_bits + 3 * 8 + 2, 6);
	d_idx += 6;
	/* D16 .. D120 */
	for (i = 4; i < 19; i++) {
		int offset = i * 8;
		memcpy(fr->d_bits + d_idx, trau_bits + offset + 1, 7);
		d_idx += 7;
	}
	/* D121 .. D126 */
	memcpy(fr->d_bits + d_idx, trau_bits + 19 * 8 + 1, 6);
	d_idx += 6;

	/* T1 */
	fr->t_bits[0] = trau_bits[19 * 8 + 7];

	return 0;
}

/* TS 08.61 Section 5.2.1.2.1 */
static int encode8_amr_low(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int i, d_idx = 0;

	/* sync pattern */
	memset(trau_bits, 0, 8);
	trau_bits[8] = 1;
	trau_bits[16] = 1;
	trau_bits[24] = 0;
	trau_bits[25] = 1;
	for (i = 4; i < 20; i++)
		trau_bits[i * 8] = 1;

	/* D1 .. D7 */
	memcpy(trau_bits + 1 * 8 + 1, fr->d_bits + d_idx, 7);
	d_idx += 7;
	/* C1 .. C5 */
	memcpy(trau_bits + 2 * 8 + 1, fr->c_bits, 5);
	/* D8 .. D9 */
	memcpy(trau_bits + 2 * 8 + 6, fr->d_bits + d_idx, 2);
	d_idx += 2;
	/* D10 .. D15 */
	memcpy(trau_bits + 3 * 8 + 2, fr->d_bits + d_idx, 6);
	d_idx += 6;
	/* D16 .. D120 */
	for (i = 4; i < 19; i++) {
		int offset = i * 8;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}
	/* D121 .. D126 */
	memcpy(trau_bits + 19 * 8 + 1, fr->d_bits + d_idx, 6);
	d_idx += 6;

	/* T1 */
	trau_bits[19 * 8 + 7] = fr->t_bits[0];

	return encode8_handle_ta(trau_bits, fr);
}

/* TS 08.61 Section 5.2.1.2.2 */
static int decode8_amr_67(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			  enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* D1 .. D7 */
	memcpy(fr->d_bits + d_idx, trau_bits + 1 * 8 + 1, 7);
	d_idx += 7;
	/* C1 .. C3 */
	memcpy(fr->c_bits, trau_bits + 2 * 8 + 1, 3);
	/* D8 .. D11 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 4, 4);
	d_idx += 4;
	/* D12 .. D39 */
	for (i = 3; i < 7; i++) {
		int offset = i * 8;
		memcpy(fr->d_bits + d_idx, trau_bits + offset + 2, 7);
		d_idx += 7;
	}

	/* D40 .. D137 */
	for (i = 7; i < 20; i+= 2) {
		int offset = i * 8;
		memcpy(fr->d_bits + d_idx, trau_bits + offset + 1, 15);
		d_idx += 15;
	}

	return 0;
}

/* TS 08.61 Section 5.1.2.2 */
static int encode8_amr_67(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int i, d_idx = 0;

	/* sync pattern */
	memset(trau_bits, 0, 8);
	trau_bits[1 * 8] = 1;
	trau_bits[2 * 8] = 1;
	trau_bits[3 * 8] = 1;
	trau_bits[4 * 8] = 1;
	trau_bits[5 * 8] = 0;
	for (i = 5; i < 20; i += 2)
		trau_bits[i * 8] = 1;

	/* D1 .. D7 */
	memcpy(trau_bits + 1 * 8 + 1, fr->d_bits + d_idx, 7);
	d_idx += 7;
	/* C1 .. C3 */
	memcpy(trau_bits + 2 * 8 + 1, fr->c_bits, 3);
	/* D8 .. D11 */
	memcpy(trau_bits + 2 * 8 + 4, fr->d_bits + d_idx, 4);
	d_idx += 4;
	/* D12 .. D39 */
	for (i = 3; i < 7; i++) {
		int offset = i * 8;
		memcpy(trau_bits + offset + 2, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}

	/* D40 .. D137 */
	for (i = 7; i < 20; i+= 2) {
		int offset = i * 8;
		memcpy(trau_bits + offset + 1, fr->d_bits + d_idx, 15);
		d_idx += 15;
	}

	return encode8_handle_ta(trau_bits, fr);
}

/* TS 08.61 Section 5.1.2.3 */
static int decode8_amr_74(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			  enum osmo_trau_frame_direction dir)
{
	int d_idx = 0;

	/* D1 .. D5 */
	memcpy(fr->d_bits + d_idx, trau_bits + 3, 5);
	d_idx += 5;
	/* D6 .. D12 */
	memcpy(fr->d_bits + d_idx, trau_bits + 1 * 8 + 1, 7);
	d_idx += 7;
	/* C1.. C3 */
	memcpy(fr->c_bits, trau_bits + 2 * 8 + 1, 3);
	/* D13 .. D16 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 4, 4);
	d_idx += 4;
	/* D17 .. D23 */
	memcpy(fr->d_bits + d_idx, trau_bits + 3 * 8 + 1, 7);
	d_idx += 7;
	/* D24 .. D151 */
	memcpy(fr->d_bits + d_idx, trau_bits + 4 * 8, 16 * 8);

	return 0;
}

/* TS 08.61 Section 5.1.2.3 */
static int encode8_amr_74(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int d_idx = 0;

	/* sync pattern */
	trau_bits[0] = 0;
	trau_bits[1] = 0;
	trau_bits[2] = 1;
	trau_bits[1*8] = 0;
	trau_bits[2*8] = 1;
	trau_bits[3*8] = 0;

	/* D1 .. D5 */
	memcpy(trau_bits + 3, fr->d_bits + d_idx, 5);
	d_idx += 5;
	/* D6 .. D12 */
	memcpy(trau_bits + 1 * 8 + 1, fr->d_bits + d_idx, 7);
	d_idx += 7;
	/* C1.. C3 */
	memcpy(trau_bits + 2 * 8 + 1, fr->c_bits, 3);
	/* D13 .. D16 */
	memcpy(trau_bits + 2 * 8 + 4, fr->d_bits + d_idx, 4);
	d_idx += 4;
	/* D17 .. D23 */
	memcpy(trau_bits + 3 * 8 + 1, fr->d_bits + d_idx, 7);
	d_idx += 7;
	/* D24 .. D151 */
	memcpy(trau_bits + 4 * 8, fr->d_bits + d_idx, 16 * 8);

	return encode8_handle_ta(trau_bits, fr);
}

/* TS 08.61 Section 5.2.2 */
static int decode8_data(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
			enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* C1 .. C5 */
	memcpy(fr->c_bits, trau_bits + 1 * 8 + 1, 5);
	/* D1 .. D2 */
	memcpy(fr->d_bits + d_idx, trau_bits + 1 * 8 + 6, 2);
	d_idx += 2;
	/* D3 .. D8 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 2, 6);
	d_idx += 6;
	/* D9 .. D63 + D'1 .. D'57 */
	for (i = 3; i < 19; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + i * 8 + 1, 7);
		d_idx += 7;
	}
	/* D'58 .. D'63 */
	memcpy(fr->d_bits + d_idx, trau_bits + 19 * 8 + 1, 6);
	d_idx += 6;

	return 0;
}

/* TS 08.61 Section 5.2.2 */
static int encode8_data(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int i, d_idx = 0;

	/* sync pattern */
	memset(trau_bits, 0, 8);
	trau_bits[1 * 8] = 1;
	trau_bits[2 * 8] = 0;
	trau_bits[2 * 8 + 1] = 1;
	for (i = 3; i < 20; i++)
		trau_bits[i * 8] = 1;
	trau_bits[19 * 8 + 7] = 1;

	/* C1 .. C5 */
	memcpy(trau_bits + 1 * 8 + 1, fr->c_bits, 5);
	/* D1 .. D2 */
	memcpy(trau_bits + 1 * 8 + 6, fr->d_bits + d_idx, 2);
	d_idx += 2;
	/* D3 .. D8 */
	memcpy(trau_bits + 2 * 8 + 2, fr->d_bits + d_idx, 6);
	d_idx += 6;
	/* D9 .. D63 + D'1 .. D'57 */
	for (i = 3; i < 19; i++) {
		memcpy(trau_bits + i * 8 + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}
	/* D'58 .. D'63 */
	memcpy(trau_bits + 19 * 8 + 1, fr->d_bits + d_idx, 6);
	d_idx += 6;

	return 20 * 8;
}

/* TS 08.61 Section 5.2.3 */
static int decode8_oam(struct osmo_trau_frame *fr, const ubit_t *trau_bits,
		       enum osmo_trau_frame_direction dir)
{
	int i, d_idx = 0;

	/* C1 .. C5 */
	memcpy(fr->c_bits, trau_bits + 1 * 8 + 1, 5);
	/* XC1 .. XC2 */
	memcpy(fr->xc_bits, trau_bits + 1 * 8 + 6, 2);
	/* XC3 .. XC6 */
	memcpy(fr->xc_bits + 2, trau_bits + 2 * 8 + 2, 4);
	/* D1 .. D2 */
	memcpy(fr->d_bits + d_idx, trau_bits + 2 * 8 + 6, 2);
	d_idx += 2;
	/* D3 .. D114 */
	for (i = 3; i < 19; i++) {
		memcpy(fr->d_bits + d_idx, trau_bits + i * 8 + 1, 7);
		d_idx += 7;
	}
	/* D115 .. D120 */
	memcpy(fr->d_bits + d_idx, trau_bits + 19 * 8 + 1, 6);
	d_idx += 7;

	return 0;
}

/* TS 08.61 Section 5.2.3 */
static int encode8_oam(ubit_t *trau_bits, const struct osmo_trau_frame *fr)
{
	int i, d_idx = 0;

	/* sync pattern */
	memset(trau_bits, 0, 8);
	trau_bits[1 * 8 + 0] = 1;
	trau_bits[2 * 8 + 0] = 0;
	trau_bits[2 * 8 + 1] = 1;
	for (i = 3; i < 20; i++)
		trau_bits[i * 8] = 1;

	/* C1 .. C5 */
	memcpy(trau_bits + 1 * 8 + 1, fr->c_bits, 5);
	/* XC1 .. XC2 */
	memcpy(trau_bits + 1 * 8 + 6, fr->xc_bits, 2);
	/* XC3 .. XC6 */
	memcpy(trau_bits + 2 * 8 + 2, fr->xc_bits + 2, 4);
	/* D1 .. D2 */
	memcpy(trau_bits + 2 * 8 + 6, fr->d_bits + d_idx, 2);
	d_idx += 2;
	/* D3 .. D114 */
	for (i = 3; i < 19; i++) {
		memcpy(trau_bits + i * 8 + 1, fr->d_bits + d_idx, 7);
		d_idx += 7;
	}
	/* D115 .. D120 */
	memcpy(trau_bits + 19 * 8 + 1, fr->d_bits + d_idx, 6);
	d_idx += 7;

	return 20 * 8;
}


/*! Encode a TRAU frame from its decoded representation to a sequence of unpacked bits.
 *  \param[out] bits caller-allocated buffer for unpacked outpud bits
 *  \param[in] n_bits size of 'bits' oputput buffer in number of unpacked bits
 *  \param[in] fr decoded representation of TRAU frame to be encoded
 *  \return 0 number of unpacked output bits generated; negative in case of error */
int osmo_trau_frame_encode(ubit_t *bits, size_t n_bits, const struct osmo_trau_frame *fr)
{
	/* check for sufficient space provided by caller in output buffer */
	switch (fr->type) {
	case OSMO_TRAU16_FT_FR:
	case OSMO_TRAU16_FT_EFR:
	case OSMO_TRAU16_FT_HR:
	case OSMO_TRAU16_FT_AMR:
		/* timing alignment may happen: increased space requirement */
		if (n_bits < 2 * 40 * 8 - 1)
			return -ENOSPC;
		break;
	case OSMO_TRAU16_FT_OAM:
	case OSMO_TRAU16_FT_IDLE:
	case OSMO_TRAU16_FT_DATA_HR:
	case OSMO_TRAU16_FT_DATA:
	case OSMO_TRAU16_FT_D145_SYNC:
	case OSMO_TRAU16_FT_EDATA:
		if (n_bits < 1 * 40 * 8)
			return -ENOSPC;
		break;
	case OSMO_TRAU8_SPEECH:
	case OSMO_TRAU8_AMR_LOW:
	case OSMO_TRAU8_AMR_6k7:
	case OSMO_TRAU8_AMR_7k4:
		/* timing alignment may happen: increased space requirement */
		if (n_bits < 2 * 20 * 8 - 1)
			return -ENOSPC;
		break;
	case OSMO_TRAU8_DATA:
	case OSMO_TRAU8_OAM:
		if (n_bits < 1 * 20 * 8)
			return -ENOSPC;
		break;
	case OSMO_TRAU_FT_NONE:
		break;
	default:
		return -EINVAL;
	}

	switch (fr->type) {
	case OSMO_TRAU16_FT_FR:
	case OSMO_TRAU16_FT_EFR:
		return encode16_fr(bits, fr);
	case OSMO_TRAU16_FT_HR:
		return encode16_hr(bits, fr);
	case OSMO_TRAU16_FT_AMR:
		return encode16_amr(bits, fr);
	case OSMO_TRAU16_FT_OAM:
		return encode16_oam(bits, fr);
	case OSMO_TRAU16_FT_IDLE:
		return encode16_idle(bits, fr);
	case OSMO_TRAU16_FT_DATA_HR:
		return encode16_data_hr(bits, fr);
	case OSMO_TRAU16_FT_DATA:
		return encode16_data(bits, fr);
	case OSMO_TRAU16_FT_D145_SYNC:
	case OSMO_TRAU16_FT_EDATA:
		return encode16_edata(bits, fr);
	case OSMO_TRAU8_SPEECH:
		return encode8_hr(bits, fr);
	case OSMO_TRAU8_DATA:
		return encode8_data(bits, fr);
	case OSMO_TRAU8_OAM:
		return encode8_oam(bits, fr);
	case OSMO_TRAU8_AMR_LOW:
		return encode8_amr_low(bits, fr);
	case OSMO_TRAU8_AMR_6k7:
		return encode8_amr_67(bits, fr);
	case OSMO_TRAU8_AMR_7k4:
		return encode8_amr_74(bits, fr);
	case OSMO_TRAU_FT_NONE:
	default:
		return -EINVAL;
	}
}

/*! Decode/parse a 16k TRAU frame from unpacked bits to decoded format.
 *  \param[out] fr caller-allocated output data structure
 *  \param[in] bits unpacked bits containing raw TRAU frame; must be aligned
 *  \param[in] dir direction (uplink/downlink)
 *  \returns 0 in case of success; negative on error */
int osmo_trau_frame_decode_16k(struct osmo_trau_frame *fr, const ubit_t *bits,
			       enum osmo_trau_frame_direction dir)
{
	uint8_t cbits5 = get_bits(bits, 17, 5);

	fr->type = OSMO_TRAU_FT_NONE;
	fr->dir = dir;
	fr->dl_ta_usec = 0;

	switch (cbits5) {
	case TRAU_FT_FR_UP:
	case TRAU_FT_FR_DOWN:
		fr->type = OSMO_TRAU16_FT_FR;
		return decode16_fr(fr, bits, dir);
	case TRAU_FT_IDLE_UP:
	case TRAU_FT_IDLE_DOWN:
		fr->type = OSMO_TRAU16_FT_IDLE;
		return decode16_fr(fr, bits, dir);
	case TRAU_FT_EFR:
		fr->type = OSMO_TRAU16_FT_EFR;
		return decode16_fr(fr, bits, dir);
	case TRAU_FT_AMR:
		fr->type = OSMO_TRAU16_FT_AMR;
		return decode16_amr(fr, bits, dir);
	case TRAU_FT_OM_UP:
	case TRAU_FT_OM_DOWN:
		fr->type = OSMO_TRAU16_FT_OAM;
		return decode16_oam(fr, bits, dir);
	case TRAU_FT_DATA_UP:
	case TRAU_FT_DATA_DOWN:
		fr->type = OSMO_TRAU16_FT_DATA;
		return decode16_data(fr, bits, dir);
	case TRAU_FT_HR_UP:
	case TRAU_FT_HR_DOWN:
		fr->type = OSMO_TRAU16_FT_HR;
		return decode16_hr(fr, bits, dir);
	case TRAU_FT_DATA_UP_HR:
	case TRAU_FT_DATA_DOWN_HR:
		fr->type = OSMO_TRAU16_FT_DATA_HR;
		return decode16_data_hr(fr, bits, dir);
	case TRAU_FT_EDATA:
		fr->type = OSMO_TRAU16_FT_EDATA;
		return decode16_edata(fr, bits, dir);
	case TRAU_FT_D145_SYNC:
		fr->type = OSMO_TRAU16_FT_D145_SYNC;
		return decode16_edata(fr, bits, dir);

	default:
		return -EINVAL;
	}
}

#define TRAU8_FT_AMR_NO_SPEECH_CMI	0x10	/* 1, 0, 0, 0, 0 */
#define TRAU8_FT_AMR_NO_SPEECH_CMR	0x14	/* 1, 0, 1, 0, 0 */
#define TRAU8_FT_AMR_475_515_590	0..7

static const uint8_t bit8_0[8] = { 0,  };

/*!< check sync pattern for hr/data/oam */
static bool is_hr(const ubit_t *bits)
{
	int i;

	/* TS 08.61 Section 6.8.2.1.1 */
	if (memcmp(bits, bit8_0, sizeof(bit8_0)))
		return false;
	if (bits[8] != 1)
		return false;
	if (bits[16] != 0 || bits[17] != 1)
		return false;
	for (i = 24; i < 20 * 8; i += 8) {
		if (bits[i] != 1)
			return false;
	}
	return true;
}


/*!< check sync pattern for AMR No_Speech + low bit rate */
static bool is_amr_low(const ubit_t *bits)
{
	int i;

	/* TS 08.61 Section 6.8.2.1.2 */
	if (memcmp(bits, bit8_0, sizeof(bit8_0)))
		return false;
	if (bits[8] != 1)
		return false;
	if (bits[16] != 1)
		return false;
	if (bits[24] != 0 || bits[25] != 1)
		return false;
	for (i = 32; i < 20 * 8; i += 8) {
		if (bits[i] != 1)
			return false;
	}
	return true;
}

/*!< check sync pattern for AMR 6.7kBit/s */
static bool is_amr_67(const ubit_t *bits)
{
	int i;

	/* TS 08.61 Section 6.8.2.1.3 */
	if (memcmp(bits, bit8_0, sizeof(bit8_0)))
		return false;
	if (bits[8] != 1)
		return false;
	if (bits[16] != 1)
		return false;
	if (bits[24] != 1)
		return false;
	if (bits[32] != 1)
		return false;
	if (bits[40] != 0)
		return false;
	for (i = 48; i < 20 * 8; i+= 16)
		if (bits[i] != 1)
			return false;

	return true;
}

/*!< check sync pattern for AMR 7.4kBit/s */
static bool is_amr_74(const ubit_t *bits)
{
	if (bits[0] != 0 || bits[1] != 0 || bits[2] != 0)
		return false;
	if (bits[8] != 0)
		return false;
	if (bits[16] != 1)
		return false;
	if (bits[24] != 0)
		return false;

	return true;
}

/*! Decode/parse a 8k TRAU frame from unpacked bits to decoded format.
 *  \param[out] fr caller-allocated output data structure
 *  \param[in] bits unpacked bits containing raw TRAU frame; must be aligned
 *  \param[in] dir direction (uplink/downlink)
 *  \returns 0 in case of success; negative on error */
int osmo_trau_frame_decode_8k(struct osmo_trau_frame *fr, const ubit_t *bits,
			      enum osmo_trau_frame_direction dir)
{
	fr->type = OSMO_TRAU_FT_NONE;
	fr->dir = dir;
	fr->dl_ta_usec = 0;

	if (is_hr(bits)) {
		/* normal sync pattern */
		uint8_t cbits5 = get_bits(bits, 9, 5);
		if (dir == OSMO_TRAU_DIR_UL) {
			switch (cbits5) {	/* Section 5.2.4.1.1 */
			case 0x02:
				return decode8_hr(fr, bits, dir);
			case 0x07:
				return decode8_data(fr, bits, dir);
			case 0x0B:
				return decode8_oam(fr, bits, dir);
			}
		} else {
			/* Downlink */
			switch (cbits5 >> 2) {	/* Section 5.2.4.1.2 */
			case 0:
				return decode8_hr(fr, bits, dir);
			case 1:
				return decode8_data(fr, bits, dir);
			case 2:
				return decode8_oam(fr, bits, dir);
			}
		}
	} else if (is_amr_low(bits)) {
		return decode8_amr_low(fr, bits, dir);
	} else if (is_amr_67(bits)) {
		return decode8_amr_67(fr, bits, dir);
	} else if (is_amr_74(bits)) {
		return decode8_amr_74(fr, bits, dir);
	}

	return -EINVAL;
}

/* }@ */

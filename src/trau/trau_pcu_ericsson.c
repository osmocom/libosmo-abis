/*
 * (C) 2022 by sysmocom - s.f.m.c. GmbH
 * Author: Philipp Maier <pmaier@sysmocom.de>
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/bitvec.h>
#include <osmocom/core/crc16gen.h>
#include <osmocom/core/logging.h>
#include <arpa/inet.h>
#include <osmocom/trau/trau_pcu_ericsson.h>
#include <osmocom/gsm/gsm0502.h>

#define E1_TRAU_BITS_MSGB 2048

/* CRC polynom for CS1 TRAU frame protection: X^16+X^12+X^5+1 */
const struct osmo_crc16gen_code cs1_crc16 = { 16, 0x1021, 0, 0xffff };

/* Frame Type C-bits: C1..C5 */
#define PCU_TRAU_ER_FT_PCU_SYNC_IND	0x0F
#define PCU_TRAU_ER_FT_CCU_SYNC_IND	0x15
#define PCU_TRAU_ER_FT_DATA_IND		0x1A
#define PCU_TRAU_ER_FT_DATA9_IND	0x04

const ubit_t T_bits_16[] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,	/* (+8 variable bits) */
};

const ubit_t T_bits_64[] = {
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,	/* (+16 variable bits) */
};

/* Calc an odd parity bit over a given number of bits */
static ubit_t calc_parity(const ubit_t *bits, size_t len)
{
	size_t i;
	ubit_t par = 1;
	for (i = 0; i < len; i++)
		par ^= bits[i];
	return par;
}

/* Put data to TRAU frame, skip T bits */
static int put_trau_data(ubit_t *bits, size_t bits_len,
			 const ubit_t *bits_map, size_t bits_map_len, const ubit_t *bits_in, size_t offs, size_t len)
{
	size_t bit_count = 0;
	size_t i = 0;

	/* The bits map must always cover all bits, so it must not be shorter
	 * then the bits we are about to parse. Also the offset must not be
	 * greater then the length of the bits */
	if (bits_len > bits_map_len)
		return -EINVAL;
	if (bits_len - offs < 0)
		return -EINVAL;

	/* Advance to the position where the data is stored */
	bits += offs;
	bits_len -= offs;
	bits_map += offs;
	bits_map_len -= offs;

	do {
		/* Do not exceed bits or bits_map */
		if (bit_count > bits_len)
			return -EINVAL;
		if (bit_count > bits_map_len)
			return -EINVAL;

		/* Do not exceed output buffer */
		if (i > bits_len)
			return -EINVAL;

		/* skip positions that have already bits set. */
		if (*bits_map == 0) {
			*bits = *bits_in;
			bits_in++;
			i++;
		}

		bit_count++;
		bits++;
		bits_map++;

	} while (i < len);

	return 0;
}

/* Put an uint32 value to TRAU frame */
static int put_trau_uint32(ubit_t *bits, size_t bits_len,
			   const ubit_t *bits_map, size_t bits_map_len, uint32_t value, size_t offs, size_t len)
{
	ubit_t buf[32];

	OSMO_ASSERT(len < 32);

	memset(buf, 0, sizeof(buf));
	value = htonl(value);
	osmo_pbit2ubit_ext(buf, 0, (ubit_t *) &value, 32 - len, len, 0);
	return put_trau_data(bits, bits_len, bits_map, bits_map_len, buf, offs, len);
}

/* Get data from TRAU frame, ignore T bits */
static int get_trau_data(ubit_t *bits_out, size_t bits_out_len,
			 const ubit_t *bits, size_t bits_len,
			 const ubit_t *bits_map, size_t bits_map_len, size_t offs, size_t len)
{
	size_t bit_count = 0;
	size_t i = 0;

	/* (see above) */
	if (bits_len > bits_map_len)
		return -EINVAL;
	if (bits_len - offs < 0)
		return -EINVAL;

	/* Advance to the position where the data is located */
	bits += offs;
	bits_map += offs;
	bits_len -= offs;
	bits_map_len -= offs;

	/* Extract bits from TRAU frame */
	do {
		/* Do not exceed bits or bits_map */
		if (bit_count > bits_len)
			return -EINVAL;
		if (bit_count > bits_map_len)
			return -EINVAL;

		/* Do not exceed output buffer */
		if (i > bits_out_len)
			return -EINVAL;

		if (*bits_map == 0) {
			*bits_out = *bits;
			bits_out++;
			i++;
		}

		bit_count++;
		bits++;
		bits_map++;

	} while (i < len);

	return 0;
}

/* Get an uint32 value from TRAU frame */
static uint32_t get_trau_uint32(const ubit_t *bits, size_t bits_len,
				const ubit_t *bits_map, size_t bits_map_len, size_t offs, uint8_t len)
{
	ubit_t buf[32];
	uint32_t result = 0;
	int rc;

	OSMO_ASSERT(len < 32);

	memset(buf, 0, sizeof(buf));
	rc = get_trau_data(buf, sizeof(buf), bits, bits_len, bits_map, bits_map_len, offs, len);

	if (rc < 0)
		return 0;

	osmo_ubit2pbit_ext((pbit_t *) &result, 32 - len, buf, 0, len, 0);
	result = ntohl(result);

	return result;
}

/* Set Time adjustment bits, add 4 more bits in case of delay */
static int set_timing_ajustment_bits_16(ubit_t *trau_bits, enum time_adj_val tav)
{
	/* Note: This sets the tail bits and returns the final length of the final length of the TRAU frame. The caller
	 * must then make sure that the frame is transitted with the returned length to achieve correct timing
	 * alignment. */

	switch (tav) {
	case TIME_ADJ_NONE:
		return 320;
	case TIME_ADJ_DELAY_250us:
		return 320 + 4;
	case TIME_ADJ_ADVANCE_250us:
		/* Note: the 16 removed bits are not transmitted. */
		return 320 - 4;
	}

	return -EINVAL;
}

/* Set Time adjustment bits, add 16 more bits in case of delay */
static int set_timing_ajustment_bits_64(ubit_t *trau_bits, enum time_adj_val tav)
{
	/* (see comment above) */

	switch (tav) {
	case TIME_ADJ_NONE:
		return 1280;
	case TIME_ADJ_DELAY_250us:
		return 1280 + 16;
	case TIME_ADJ_ADVANCE_250us:
		/* Note: the 16 removed bits are not transmitted. */
		return 1280 - 16;
	}

	return -EINVAL;
}

/* Decode an 8-byte access burst data structure */
static int decode_ab(struct er_gprs_ab *ab, uint8_t *ab_bytes)
{
	memset(ab, 0, sizeof(*ab));

	/* Type 1/2 specific fields */
	if ((ab_bytes[0] & 0x1f) == 0x1f) {
		ab->ab_type = 1;
		ab->u.type_1.crc = ab_bytes[0] >> 5 & 0x07;
		ab->u.type_1.burst_qual = ab_bytes[2];
		ab->u.type_1.frame_qual = ab_bytes[3];
	} else if ((ab_bytes[0] & 0x1f) == 0) {
		ab->ab_type = 2;
		ab->u.type_2.abi = ab_bytes[0] >> 5 & 0x07;
		ab->u.type_2.type = ab_bytes[3] >> 6 & 0x03;
	} else
		return -EINVAL;

	/* Common fields */
	ab->rxlev = ab_bytes[1];
	ab->acc_delay = (ab_bytes[4] << 2) & 0x0300;
	ab->acc_delay |= ab_bytes[5];
	ab->data = (ab_bytes[6] << 3) & 0x0700;
	ab->data |= ab_bytes[7];

	return 0;
}

static int enc_pcu_sync_ind_16(ubit_t *trau_bits, struct er_pcu_sync_ind *ind)
{
	/* 16kbps PCU-SYNC-IND TRAU frame format:
	 * Direction: PCU => CCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    0.    0     0     0     0     0
	 *         1     0     0     0     0     0     0     0
	 *         0     D15   D16   D17   D18   D19   D20   D21   PSEQ (offset 41)
	 *         1     D22   D23   D24   D25   D26   D27   D28
	 *         D29   D30   D31   D32   D33   D34   D35   D36
	 *         1     D37   D38   D39   D40   D41   D42   D43   SS (offset 65)
	 *         D44   D45   D46   D47   D48   D49   D50   D51
	 *         1     1     1     1     1     1     1     1
	 *         1     D60   D61   D62   D63   D64   D65   D66   FN UL (offset 89)
	 *         1     D67   D68   D69   D70   D71   D72   D73
	 *         D74   D75   D76   D77   D78   D79   D80   D81
	 *         1     D82   D83   D84   D85   D86   D87   D88   FN SS (offset 113)
	 *         D89   D90   D91   D92   D93   D94   D95   D96
	 *         1     1     1     1     1     1     1     1
	 *         1     D105  D106  D107  D108  D109  D110  D111  FN DL (offset 137)
	 *         1     D112  D113  D114  D115  D116  D117  D118
	 *         D119  D120  D121  D122  D123  D124  D125  D126
	 *         1     D127  D128  D129  D130  D131  D132  D133  LS (offset 161)
	 *         D134  D135  D136  D137  D138  D139  D140  D141
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     ...
	 *                                 ...   1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     TA1   TA2   TA3   TA4 (bit 319)
	 *         TA5*  TA6*  TA7*  TA8*
	 *
	 * "*" = additional TA bits to delay the frame
	 * C = Control bits
	 * D = Data bits
	 * PC = Parity over C-Bits (odd) */

	pbit_t c_1_5 = PCU_TRAU_ER_FT_PCU_SYNC_IND;
	int rc;

	/* C-Bits */
	osmo_pbit2ubit_ext(trau_bits, 17, &c_1_5, 0, 5, 1);
	osmo_pbit2ubit_ext(trau_bits, 22, (pbit_t *) &ind->tav, 0, 2, 1);
	if (ind->ul_frame_err == false)
		trau_bits[24] = 1;
	trau_bits[25] = calc_parity(trau_bits + 17, 8);

	/* D-Bits */
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->pseq, 41, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->ss, 65, 15);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->fn_ul, 89, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->fn_ss, 113, 15);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->fn_dl, 137, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), ind->ls, 161, 15);
	if (rc < 0)
		return -EINVAL;

	return set_timing_ajustment_bits_16(trau_bits, ind->tav);
}

int enc_pcu_data_ind_16(ubit_t *trau_bits, struct er_pcu_data_ind *ind)
{
	/* 16kbps PCU-DATA-IND TRAU frame format:
	 * Direction: PCU => CCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    E1    E2    E3    E4    E5    E6
	 *         E7    E8    E9    E10   E11   E12   E13   E14
	 *         E15   E16   PE    D1    D2    D3    D4    D5
	 *         D6    D7    D8    D9    D10   D11   D12   D13
	 *         D14   D15   D16   ...
	 *                                 ...   D267  D268  D269
	 *         D270  D271  D272  D273  TA1   TA2   TA3   TA4 (bit 319)
	 *         TA5*  TA6*  TA7*  TA8*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	pbit_t c_1_5 = PCU_TRAU_ER_FT_DATA_IND;

	/* C-Bits */
	osmo_pbit2ubit_ext(trau_bits, 17, &c_1_5, 0, 5, 1);
	osmo_pbit2ubit_ext(trau_bits, 22, (pbit_t *) &ind->tav, 0, 2, 1);
	if (ind->ul_frame_err == false)
		trau_bits[24] = 1;
	trau_bits[25] = calc_parity(trau_bits + 17, 8);

	/* Set coding scheme (E1-E2) */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		trau_bits[26] = 0;
		trau_bits[27] = 0;
		break;
	case CS_OR_HDR_CS2:
		trau_bits[26] = 0;
		trau_bits[27] = 1;
		break;
	default:
		/* NOTE: The 16K TRAU frames do not have enough bandwidth to
		 * support coding schemes other than CS1 and CS2 */
		/* NOTE: Access bursts (AB) are uplink-only. */
		return -EINVAL;
	}

	/* Set demodulation in uplink (E3-E4) */
	switch (ind->ul_chan_mode) {
	case ER_UL_CHMOD_NB_GMSK:
		trau_bits[28] = 0;
		trau_bits[29] = 0;
		break;
	case ER_UL_CHMOD_AB:
		trau_bits[28] = 0;
		trau_bits[29] = 1;
		break;
	case ER_UL_CHMOD_AB_UNKN:
		trau_bits[28] = 1;
		trau_bits[29] = 0;
		break;
	case ER_UL_CHMOD_VOID:
		trau_bits[28] = 1;
		trau_bits[29] = 1;
		break;
	default:
		return -EINVAL;
	}

	/* Timing offset (E5-E12, 8 bit value, MSB first) */
	osmo_pbit2ubit_ext(trau_bits, 30, (pbit_t *) &ind->timing_offset, 0, 8, 0);

	/* Power control (E13-E16, 4 bit value, MSB first, 2dB steps) */
	osmo_pbit2ubit_ext(trau_bits, 38, (pbit_t *) &ind->atten_db, 4, 4, 0);

	/* Parity (odd) over coding scheme, demodulation, timing offset and
	 * power control bits (E1-E16) */
	trau_bits[42] = calc_parity(trau_bits + 26, 16);

	/* Data bits */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		osmo_pbit2ubit_ext(trau_bits, 43, (pbit_t *) ind->data, 0, 184, 1);
		osmo_crc16gen_set_bits(&cs1_crc16, trau_bits + 43, 184, trau_bits + 43 + 184);
		break;
	case CS_OR_HDR_CS2:
		osmo_pbit2ubit_ext(trau_bits, 43, (pbit_t *) ind->data, 0, 271, 1);
		break;
	default:
		/* NOTE: The 16K TRAU frames do not have enough bandwidth to
		 * support coding schemes other than CS1 and CS2 */
		return -EINVAL;
	}

	return set_timing_ajustment_bits_16(trau_bits, ind->tav);
}

/*! encode an 16k Ericsson GPRS (GSL) TRAU frame.
 *  \param[out] bits caller-allocated memory for unpacked output bits (320+4).
 *  \param[in] fr input data structure describing TRAU frame.
 *  \return number of bits encoded. */
int er_gprs_trau_frame_encode_16k(ubit_t *bits, struct er_gprs_trau_frame *fr)
{
	/* Prepare frame: first 16 bits set 0, remaining bits set to 1 */
	memset(bits, 0, 16);
	memset(bits + 16, 1, 320 + 4 - 16);

	switch (fr->type) {
	case ER_GPRS_TRAU_FT_SYNC:
		return enc_pcu_sync_ind_16(bits, &fr->u.pcu_sync_ind);
	case ER_GPRS_TRAU_FT_DATA:
		return enc_pcu_data_ind_16(bits, &fr->u.pcu_data_ind);
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int dec_ccu_sync_ind_16(struct er_ccu_sync_ind *ind, const ubit_t *trau_bits)
{
	/* 16kbps CCU-SYNC-IND TRAU frame format:
	 * Direction: CCU => PCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    E1    PE    1     1     1     1
	 *         1     0     0     0     0     0     0     0
	 *         0     D13   D14   D15   D16   D17   D18   D19   PSEQ (offset 41)
	 *         1     D20   D21   D22   D23   D24   D25   D26
	 *         D27   D28   D29   D30   D31   D32   D33   D34
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     D58   D59   D60   D61   D62   D63   D64   AFN UL (offset 89)
	 *         1     D65   D66   D67   D68   D69   D70   D71
	 *         D72   D73   D74   D75   D76   D77   D78   D79
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     D103  D104  D105  D106  D107  D108  D109  AFN DL (offset 137)
	 *         1     D110  D111  D112  D113  D114  D115  D116
	 *         D117  D118  D119  D120  D121  D122  D123  D124
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     ...
	 *                                 ...   1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     TA1   TA2 (bit 319)
	 *         TA3*  TA4*
	 *
	 * "*" = additional TA bits to delay the frame
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	size_t i;

	/* Validate sync pattern (extended) */
	for (i = 0; i < 19; i++) {
		if (trau_bits[16 + i * 16] != 1) {
			LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-16: invalid sync pattern (T1 at position %zu != 1)\n", i);
			return -EINVAL;
		}
	}

	/* Validate C-Bits */
	if (calc_parity(trau_bits + 17, 8) != trau_bits[25]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-16: invalid parity (C1-C8)\n");
		return -EINVAL;
	}

	/* TAV (C7-C9) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->tav, 0, trau_bits, 22, 2, 1);

	/* Downlink frame error, DFE (C8) */
	if (trau_bits[24] == 0)
		ind->dfe = true;

	/* Check (odd) parity of E1 bit */
	if (trau_bits[26] != ((~trau_bits[27]) & 1)) {
		LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-16: invalid parity (E1)\n");
		return -EINVAL;
	}

	/* Downlink Block error, DBE (E1) */
	if (trau_bits[26] == 0)
		ind->dbe = true;

	/* D bits */
	ind->pseq = get_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), 41, 22);
	ind->afn_ul = get_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), 89, 22);
	ind->afn_dl = get_trau_uint32(trau_bits, 320, T_bits_16, sizeof(T_bits_16), 137, 22);

	return 0;
}

static int dec_ccu_data_ind_16(struct er_ccu_data_ind *ind, const ubit_t *trau_bits)
{
	/* 16kbps CCU-DATA-IND TRAU frame format:
	 * Direction: CCU => PCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         PC    E1    E2    E3    E4    E5    E6    E7
	 *         E8    E9    E10   E11   E12   E13   E14   E15
	 *         E16   E17   E18   E19   PE    D1    D2    D3
	 *         D4    D5    D6    D7    D8    D9    D10   D11
	 *         D12   D13   D14   ...
	 *                                 ...   D267  D268  D269
	 *         D270  D271  D272  D273  TA1   TA2   TA3   TA4 (bit 319)
	 *         TA5*  TA6*  TA7*  TA8*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	uint8_t sfq = 0;
	uint8_t eadd = 0;
	uint8_t e_2_4 = 0;
	int rc;
	int i;

	/* Validate C-Bits */
	if (calc_parity(trau_bits + 17, 7) != trau_bits[24]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: invalid parity (C1-C7)\n");
		return -EINVAL;
	}

	/* TAV (C6-C7) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->tav, 0, trau_bits, 22, 2, 1);

	/* Validate E-Bits */
	if (calc_parity(trau_bits + 25, 19) != trau_bits[44]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: invalid parity (E1-E19)\n");
		return -EINVAL;
	}

	/* Downlink block error (E1) */
	if (trau_bits[25] == 0)
		ind->dbe = true;

	/* Coding scheme (E2-E4) */
	osmo_ubit2pbit_ext((pbit_t *) &e_2_4, 5, trau_bits, 26, 3, 0);
	switch (e_2_4) {
	case 0:
		ind->cs_hdr = CS_OR_HDR_CS1;
		break;
	case 1:
		ind->cs_hdr = CS_OR_HDR_CS2;
		break;
	case 4:
		ind->cs_hdr = CS_OR_HDR_AB;
		break;
	default:
		/* 16kbps timeslots only support CS1, CS2 and AB are supported,
		 * due to bandwidth limitations. */
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: invalid codec status (E2-E4)\n");
		return -EINVAL;
	}

	/* Soft frame quality SFQ (E5-E7, 0 in case of AB) */
	osmo_ubit2pbit_ext((pbit_t *) &sfq, 5, trau_bits, 29, 3, 0);
	ind->u.gprs.block_qual = sfq;

	/* Parity Check (E8, 1 in case of AB) */
	ind->u.gprs.parity_ok = trau_bits[32];

	/* RX-LEV (E9-E14, 63 in case of AB) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->rx_lev, 2, trau_bits, 33, 6, 0);

	/* Estimated access delay (E15-E17, 0 in case of AB) */
	osmo_ubit2pbit_ext((pbit_t *) &eadd, 5, trau_bits, 39, 3, 0);
	switch (eadd) {
	case 0:
		/* <2 or less */
		ind->est_acc_del_dev = -3;
		break;
	case 1:
		ind->est_acc_del_dev = -1;
		break;
	case 2:
		ind->est_acc_del_dev = 1;
		break;
	case 4:
		ind->est_acc_del_dev = 2;
		break;
	case 5:
		/* >2 or more */
		ind->est_acc_del_dev = 3;
		break;
	case 6:
		ind->est_acc_del_dev = 0;
		break;
	case 7:
		ind->est_acc_del_dev = -2;
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: invalid estimated access delay (E15-E17)\n");
		return -EINVAL;
	}

	/* Data bits */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 45, 184, 1);
		rc = osmo_crc16gen_check_bits(&cs1_crc16, trau_bits + 45, 184, trau_bits + 45 + 184);
		if (rc != 0) {
			LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: CRC error in CS1 block\n");
			return -EINVAL;
		}
		ind->data_len = 23;
		break;
	case CS_OR_HDR_CS2:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 45, 271, 1);
		ind->data_len = 34;
		break;
	case CS_OR_HDR_AB:
		/* Note: The useful data starts at D4 and is byte-aligned inside the TRAU frame.
		 * The data string contains 4 items, each 8 bytes long, 32 bytes total. */
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 48, 256, 1);
		ind->data_len = 32;

		for (i = 0; i < 4; i++) {
			rc = decode_ab(&ind->ab[i], ind->data + i * 8);
			if (rc < 0)
				return -EINVAL;
		}
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-16: invalid cs_hdr set\n");
		return -EINVAL;
	}

	return 0;
};

/*! decode an 16k Ericsson GPRS (GSL) TRAU frame.
 *  \param[out] fr caller-allocated output data structure.
 *  \param[in] bits unpacked input bits (320).
 *  \return 0 on success; negative in case of error. */
int er_gprs_trau_frame_decode_16k(struct er_gprs_trau_frame *fr, const ubit_t *bits)
{
	uint8_t c_1_5 = 0;
	const ubit_t expected_sync_pattern[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	int rc;

	memset(fr, 0, sizeof(*fr));

	/* Validate sync pattern */
	if (memcmp(bits, expected_sync_pattern, ARRAY_SIZE(expected_sync_pattern)) != 0) {
		LOGP(DLINP, LOGL_ERROR, "CCU-XXXX-IND-16: invalid sync pattern (T0)\n");
		return -EINVAL;
	}

	/* Determine frame type */
	osmo_ubit2pbit_ext((pbit_t *) &c_1_5, 0, bits, 17, 5, 1);

	switch (c_1_5) {
	case PCU_TRAU_ER_FT_CCU_SYNC_IND:
		fr->type = ER_GPRS_TRAU_FT_SYNC;
		rc = dec_ccu_sync_ind_16(&fr->u.ccu_sync_ind, bits);
		break;
	case PCU_TRAU_ER_FT_DATA_IND:
		fr->type = ER_GPRS_TRAU_FT_DATA;
		rc = dec_ccu_data_ind_16(&fr->u.ccu_data_ind, bits);
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-XXXX-IND-16: invalid frame type (%02x)\n", c_1_5);
		rc = -EINVAL;
	}

	/* Ensure that we exit with predictable data in case of error. */
	if (rc < 0)
		memset(fr, 0, sizeof(*fr));

	return rc;
}

/* Extract the CPS field from a given block, block must have minimum length of 5 bytes.
 * See also: 3GPP TS 44.060, section 10.3a.3 and section 10.3a.4 */
static int cps_from_mcs_block(uint8_t *block, enum er_cs_or_hdr cs_hdr, bool uplink)
{
	uint8_t cps;

	if (uplink) {
		switch (cs_hdr) {
		case CS_OR_HDR_HDR1:
			cps = block[4] & 0x1f;
			break;
		case CS_OR_HDR_HDR2:
			cps = (block[2] >> 6) & 0x3;
			cps |= block[3] << 2 & 0x4;
			break;
		case CS_OR_HDR_HDR3:
			cps = (block[2] >> 6) & 0x3;
			cps |= block[3] << 2 & 0xC;
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (cs_hdr) {
		case CS_OR_HDR_HDR1:
			cps = (block[4] >> 3) & 0x1f;
			break;
		case CS_OR_HDR_HDR2:
			cps = (block[3] >> 1) & 0x07;
			break;
		case CS_OR_HDR_HDR3:
			cps = (block[3] >> 1) & 0x0f;
			break;
		default:
			return -EINVAL;
		}
	}

	return cps;
}

/* Determine the MCS block type from a given CPS value.
 * See also: 3GPP TS 44.060, section 10.4.8a */
static int mcs_from_cps(uint8_t cps, enum er_cs_or_hdr cs_hdr)
{
	switch (cs_hdr) {
	case CS_OR_HDR_HDR1:
		if (cps <= 0x0A)
			return 9;
		if (cps <= 0x13)
			return 8;
		if (cps <= 0x1C)
			return 7;
		return -EINVAL;
		break;
	case CS_OR_HDR_HDR2:
		if (cps <= 0x03 || cps == 0x06 || cps == 0x07)
			return 6;
		if (cps == 4 || cps == 5)
			return 5;
		return -EINVAL;
		break;
	case CS_OR_HDR_HDR3:
		if (cps <= 0x02)
			return 4;
		if (cps <= 0x08)
			return 3;
		if (cps == 0x09 || cps == 0x0A || cps == 0x0D || cps == 0x0E)
			return 2;
		if (cps == 0x0B || cps == 0x0C)
			return 1;
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
}

static int enc_pcu_sync_ind_64(ubit_t *trau_bits, struct er_pcu_sync_ind *ind)
{
	/* 64kbps PCU-SYNC-IND TRAU frame format:
	 * Direction: PCU => CCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D96   D97   D98   D99   D100  D101
	 *         D102  D103  D104  D105  D106  D107  D108  D109
	 *         D110  D111  D112  D113  D114  D115  D116  D117
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         D133  D134  D135  D136  D137  D138  D139  D140
	 *         D141  D142  D143  D144  D145  D146  D147  D148
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D159  D160  D161  D162  D163  D164
	 *         D165  D166  D167  D168  D169  D170  D171  D172
	 *         D173  D174  D175  D176  D177  D178  D179  D180
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         D196  D197  D198  D199  D200  D201  D202  D203
	 *         D204  D205  D206  D207  D208  D209  D210  D211
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D222  D223  D224  D225  D226  D227
	 *         D228  D229  D230  D231  D232  D233  D234  D235
	 *         D236  D237  D238  D239  D340  D241  D242  D243
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         D259  D260  D261  D262  D263  D264  D265  D266
	 *         D267  D268  D269  D270  D271  D272  D273  D274
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     ...
	 *                                 ...   1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * "*" = additional TA bits to delay the frame
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	pbit_t c_1_5 = PCU_TRAU_ER_FT_PCU_SYNC_IND;
	int rc;

	/* C-Bits */
	osmo_pbit2ubit_ext(trau_bits, 65, &c_1_5, 0, 5, 1);
	osmo_pbit2ubit_ext(trau_bits, 70, (pbit_t *) &ind->tav, 0, 2, 1);
	if (ind->ul_frame_err == false)
		trau_bits[73] = 1;
	trau_bits[74] = calc_parity(trau_bits + 65, 8);

	/* Set unused D-Bits to 1 */
	memset(trau_bits + 76, 1, 1280 - 76 - 16);

	/* D-Bits */
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->pseq, 170, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->ss, 208, 16);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->fn_ul, 234, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->fn_ss, 272, 15);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->fn_dl, 298, 22);
	if (rc < 0)
		return -EINVAL;
	rc = put_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), ind->ls, 336, 16);
	if (rc < 0)
		return -EINVAL;

	return set_timing_ajustment_bits_64(trau_bits, ind->tav);
}

int enc_pcu_data_ind_64(ubit_t *trau_bits, struct er_pcu_data_ind *ind, uint8_t mcs)
{
	/* 64kbps PCU-DATA-IND TRAU frame format:
	 * Direction: PCU => CCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    E1    E2    E3    E4    E5    E6
	 *         E7    E8    E9    E10   E11   E12   E13   E14
	 *         E15   E16   E17   E18   E19   E20   E21   E22
	 *         E23   E24   E25   E26   E27   E28   E29   E30
	 *         E31   E32   E33   E34   E35   E36   PE    S1
	 *         S2    S3    S4    S5    S6    S7    S8    S9
	 *         S10   S11   S12   S13   S14   S15   S16   S17
	 *         S18   S19   S20   S21   D1    D2    D3    D4
	 *         D5    D6    D7    D8    D9    D10   D11   D12
	 *         D13   D14   D15   D16   D17   D18   D19   D20
	 *         D21   D22   D23   ...
	 *                                   ... D1130 D1131 D1132
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	pbit_t c_1_5 = PCU_TRAU_ER_FT_DATA_IND;

	/* C-Bits */
	osmo_pbit2ubit_ext(trau_bits, 65, &c_1_5, 0, 5, 1);
	osmo_pbit2ubit_ext(trau_bits, 70, (pbit_t *) &ind->tav, 0, 2, 1);
	if (ind->ul_frame_err == false)
		trau_bits[73] = 1;
	trau_bits[74] = calc_parity(trau_bits + 65, 8);

	/* Set coding scheme (E1-E3) */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		trau_bits[74] = 0;
		trau_bits[75] = 0;
		trau_bits[76] = 1;
		break;
	case CS_OR_HDR_CS2:
		trau_bits[74] = 0;
		trau_bits[75] = 1;
		trau_bits[76] = 0;
		break;
	case CS_OR_HDR_CS3:
		trau_bits[74] = 0;
		trau_bits[75] = 1;
		trau_bits[76] = 1;
		break;
	case CS_OR_HDR_CS4:
		trau_bits[74] = 1;
		trau_bits[75] = 0;
		trau_bits[76] = 0;
		break;
	case CS_OR_HDR_HDR1:
		trau_bits[74] = 1;
		trau_bits[75] = 0;
		trau_bits[76] = 1;
		break;
	case CS_OR_HDR_HDR2:
		trau_bits[74] = 1;
		trau_bits[75] = 1;
		trau_bits[76] = 0;
		break;
	case CS_OR_HDR_HDR3:
		trau_bits[74] = 1;
		trau_bits[75] = 1;
		trau_bits[76] = 1;
		break;
	default:
		/* NOTE: Access bursts (AB) are uplink-only. */
		return -EINVAL;
	}

	/* Set demodulation in uplink (E4-E6) */
	switch (ind->ul_chan_mode) {
	case ER_UL_CHMOD_VOID:
		trau_bits[77] = 0;
		trau_bits[78] = 0;
		trau_bits[79] = 0;
		break;
	case ER_UL_CHMOD_NB_GMSK:
		trau_bits[77] = 0;
		trau_bits[78] = 0;
		trau_bits[79] = 1;
		break;
	case ER_UL_CHMOD_NB_UNKN:
		trau_bits[77] = 0;
		trau_bits[78] = 1;
		trau_bits[79] = 0;
		break;
	case ER_UL_CHMOD_AB:
		trau_bits[77] = 0;
		trau_bits[78] = 1;
		trau_bits[79] = 1;
		break;
	case ER_UL_CHMOD_AB_UNKN:
		trau_bits[77] = 1;
		trau_bits[78] = 0;
		trau_bits[79] = 0;
		break;
	default:
		return -EINVAL;
	}

	/* Timing offset (E7-E14, 8 bit value, MSB first) */
	osmo_pbit2ubit_ext(trau_bits, 80, (pbit_t *) &ind->timing_offset, 0, 8, 0);

	/* Power control (E33-E36, 4 bit value, MSB first, 2dB steps) */
	osmo_pbit2ubit_ext(trau_bits, 106, (pbit_t *) &ind->atten_db, 4, 4, 0);

	/* Parity (odd) over coding scheme, demodulation, timing offset and
	 * power control bits (E1-E36) */
	trau_bits[110] = calc_parity(trau_bits + 74, 36);

	/* Data bits */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 184, 1);
		osmo_crc16gen_set_bits(&cs1_crc16, trau_bits + 132, 184, trau_bits + 132 + 184);
		break;
	case CS_OR_HDR_CS2:
		osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 271, 1);
		break;
	case CS_OR_HDR_CS3:
		osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 315, 1);
		break;
	case CS_OR_HDR_CS4:
		osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 431, 1);
		break;
	case CS_OR_HDR_HDR1:
	case CS_OR_HDR_HDR2:
	case CS_OR_HDR_HDR3:
		switch (mcs) {
		case 1:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 209, 1);
			break;
		case 2:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 257, 1);
			break;
		case 3:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 329, 1);
			break;
		case 4:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 385, 1);
			break;
		case 5:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 478, 1);
			break;
		case 6:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 622, 1);
			break;
		case 7:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 940, 1);
			break;
		case 8:
			osmo_pbit2ubit_ext(trau_bits, 132, (pbit_t *) ind->data, 0, 1132, 1);
			break;
		}
		break;
	default:
		/* NOTE: The 16K TRAU frames do not have enough bandwidth to
		 * support coding schemes other than CS1 and CS2 */
		return -EINVAL;
	}

	return set_timing_ajustment_bits_64(trau_bits, ind->tav);
}

int enc_pcu_data_ind_64_mcs9(ubit_t *trau_bits, struct er_pcu_data_ind *ind)
{
	/* 64kbps PCU-DATA-IND (MCS9) TRAU frame format:
	 * Direction: PCU => CCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     D1    D2    D3    D4    D5    D6    D7
	 *         D8    D9    D10   D11   D12   D13   D14   D15
	 *         D16   D17   D18   D19   D20   D21   D22   D23
	 *         D24   D25   D26   D27   D28   D29   D30   D31
	 *         D32   D33   D34   D35   D36   D37   D38   D39
	 *         D40   D41   D42   D43   D44   D45   D46   D47
	 *         D48   C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    E1    E2    E3    E4    E5    E6
	 *         E7    E8    E9    PE    D49   D50   D51   D52
	 *         D53   D54   D55   D56   D57   D58   D59   D60
	 *         D61   D62   D63   D64   D65   D66   D67   D68
	 *         D69   D70   D71   ...
	 *                                   ... D1226 D1227 D1228
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	pbit_t c_1_5 = PCU_TRAU_ER_FT_DATA9_IND;

	/* NOTE: The ericsson MCS9 64K TRAU format uses a different sync
	 * pattern than the other 64K TRAU frame formats. To make room for
	 * the additional bits in MCS9 only the first 16 bits are T0 bits. */
	trau_bits[16] = 1;

	/* Fill headroom that normally would have T0 and T1 bits with data
	 * bits */
	osmo_pbit2ubit_ext(trau_bits, 17, (pbit_t *) ind->data, 0, 48, 1);

	/* C-Bits */
	osmo_pbit2ubit_ext(trau_bits, 65, &c_1_5, 0, 5, 1);
	osmo_pbit2ubit_ext(trau_bits, 70, (pbit_t *) &ind->tav, 0, 2, 1);
	if (ind->ul_frame_err == false)
		trau_bits[73] = 1;
	trau_bits[74] = calc_parity(trau_bits + 65, 8);

	/* Set demodulation in uplink (E1-E3) */
	switch (ind->ul_chan_mode) {
	case ER_UL_CHMOD_VOID:
		trau_bits[74] = 0;
		trau_bits[75] = 0;
		trau_bits[76] = 0;
		break;
	case ER_UL_CHMOD_NB_GMSK:
		trau_bits[74] = 0;
		trau_bits[75] = 0;
		trau_bits[76] = 1;
		break;
	case ER_UL_CHMOD_NB_UNKN:
		trau_bits[74] = 0;
		trau_bits[75] = 1;
		trau_bits[76] = 0;
		break;
	case ER_UL_CHMOD_AB:
		trau_bits[74] = 0;
		trau_bits[75] = 1;
		trau_bits[76] = 1;
		break;
	case ER_UL_CHMOD_AB_UNKN:
		trau_bits[74] = 1;
		trau_bits[75] = 0;
		trau_bits[76] = 0;
		break;
	default:
		return -EINVAL;
	}

	/* E4-E5 are spare bits? */

	/* Power control (E6-E9, 4 bit value, MSB first, 2dB steps) */
	osmo_pbit2ubit_ext(trau_bits, 79, (pbit_t *) &ind->atten_db, 4, 4, 0);

	/* Parity (odd) over coding scheme, demodulation, timing offset and
	 * power control bits (E1-E9) */
	trau_bits[83] = calc_parity(trau_bits + 74, 9);

	/* Fill the rest of the block with data bits */
	osmo_pbit2ubit_ext(trau_bits, 84, (pbit_t *) ind->data + 6, 0, 1180, 1);

	return set_timing_ajustment_bits_64(trau_bits, ind->tav);
}

/*! encode an 64k Ericsson GPRS (GSL) TRAU frame.
 *  \param[out] bits caller-allocated memory for unpacked output bits (1280+16).
 *  \param[in] fr input data structure describing TRAU frame.
 *  \return number of bits encoded. */
int er_gprs_trau_frame_encode_64k(ubit_t *bits, struct er_gprs_trau_frame *fr)
{
	int cps;
	int mcs = 0;
	enum er_cs_or_hdr cs;

	/* Prepare frame: first 16 bits set 0, remaining bits set to 1 */
	memset(bits, 0, 64);
	memset(bits + 64, 1, 1280 + 16 - 64);

	switch (fr->type) {
	case ER_GPRS_TRAU_FT_SYNC:
		return enc_pcu_sync_ind_64(bits, &fr->u.pcu_sync_ind);
	case ER_GPRS_TRAU_FT_DATA:
		cs = fr->u.pcu_data_ind.cs_hdr;
		if (cs == CS_OR_HDR_HDR1 || cs == CS_OR_HDR_HDR2 || cs == CS_OR_HDR_HDR3) {
			cps = cps_from_mcs_block(fr->u.pcu_data_ind.data, cs, false);
			if (cps < 0)
				return -EINVAL;
			mcs = mcs_from_cps((uint8_t) cps, cs);
			if (mcs < 0)
				return -EINVAL;
		}

		if (mcs < 9)
			return enc_pcu_data_ind_64(bits, &fr->u.pcu_data_ind, mcs);
		else
			return enc_pcu_data_ind_64_mcs9(bits, &fr->u.pcu_data_ind);
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int dec_ccu_sync_ind_64(struct er_ccu_sync_ind *ind, const ubit_t *trau_bits)
{
	/* 64kbps CCU-SYNC-IND TRAU frame format:
	 * Direction: CCU => PCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         C8    PC    E1    PE    1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D94   D95   D96   D97   D98   D99
	 *         D100  D101  D102  D103  D104  D105  D106  D107
	 *         D108  D109  D110  D111  D112  D113  D114  D115
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D157  D158  D159  D160  D161  D162
	 *         D163  D164  D165  D166  D167  D168  D169  D170
	 *         D171  D172  D173  D174  D175  D176  D177  D178
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         1     1     D220  D221  D222  D223  D224  D225
	 *         D226  D227  D228  D229  D230  D231  D232  D233
	 *         D234  D235  D236  D237  D238  D239  D240  D241
	 *         1     1     1     1     1     1     1     1
	 *         1     1     1     ...
	 *                                 ...   1     1     1
	 *         1     1     1     1     1     1     1     1
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * "*" = additional TA bits to delay the frame
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	size_t i;

	/* Validate sync pattern (extended) */
	for (i = 0; i < 19; i++) {
		if (trau_bits[64 + i * 64] != 1) {
			LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-64: invalid sync pattern (T1 at position %zu != 1)\n", i);
			return -EINVAL;
		}
	}

	/* Validate C-Bits */
	if (calc_parity(trau_bits + 65, 8) != trau_bits[73]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-64: invalid parity (C1-C8)\n");
		return -EINVAL;
	}

	/* TAV (C6-C7) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->tav, 0, trau_bits, 70, 2, 1);

	/* Downlink frame error, DFE (C8) */
	if (trau_bits[72] == 0)
		ind->dfe = true;

	/* Check (odd) parity of E1 bit */
	if (trau_bits[74] != ((~trau_bits[75]) & 1)) {
		LOGP(DLINP, LOGL_ERROR, "CCU-SYNC-IND-64: invalid parity (E1)\n");
		return -EINVAL;
	}

	/* Downlink Block error, DBE (E1) */
	if (trau_bits[74] == 0)
		ind->dbe = true;

	/* D bits */
	ind->pseq = get_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), 170, 22);
	ind->afn_ul = get_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), 234, 22);
	ind->afn_dl = get_trau_uint32(trau_bits, 1280, T_bits_64, sizeof(T_bits_64), 298, 22);

	return 0;
}

static int dec_ccu_data_ind_64(struct er_ccu_data_ind *ind, const ubit_t *trau_bits)
{
	/* 64kbps CCU-DATA-IND TRAU frame format:
	 * Direction: CCU => PCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     C1    C2    C3    C4    C5    C6    C7
	 *         PC    E1    E2    E3    E4    E5    E6    E7
	 *         E8    E9    E10   E11   E12   E13   E14   E15
	 *         E16   E17   E18   E19   E20   E21   E22   E23
	 *         E24   E25   E26   E27   E28   E29   E30   E31
	 *         E32   E33   E34   E35   E36   E37   E38   E39
	 *         E40   E41   E42   E43   E44   E45   E46   E47
	 *         E48   E49   E50   E51   E52   E53   E54   E55
	 *         E56   E57   PE    S1    S2    S3    D1    D2
	 *         D3    D4    D5    D6    D7    D8    D9    D10
	 *         D11   D12   D13   D14   D15   D16   D17   D18
	 *         D19   D20   D21   ...
	 *                                 ...   D1136 D1137 D1138
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */
	uint8_t sfq = 0;
	uint8_t eadd = 0;
	uint8_t e_2_4 = 0;
	uint8_t mean_bep = 0;
	uint8_t cv_bep = 0;
	int cps;
	int mcs = 0;
	int rc;
	int i;

	/* Validate C-Bits */
	if (calc_parity(trau_bits + 65, 7) != trau_bits[72]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid parity (C1-C7)\n");
		return -EINVAL;
	}

	/* TAV (C6-C7) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->tav, 0, trau_bits, 70, 2, 1);

	/* Validate E-Bits */
	if (calc_parity(trau_bits + 73, 57) != trau_bits[130]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid parity (E1-E57)\n");
		return -EINVAL;
	}

	/* Downlink block error (E1) */
	if (trau_bits[73] == 0)
		ind->dbe = true;

	/* Coding scheme (E2-E4) */
	osmo_ubit2pbit_ext((pbit_t *) &e_2_4, 5, trau_bits, 74, 3, 0);
	switch (e_2_4) {
	case 0:
		ind->cs_hdr = CS_OR_HDR_AB;
		break;
	case 1:
		ind->cs_hdr = CS_OR_HDR_CS1;
		break;
	case 2:
		ind->cs_hdr = CS_OR_HDR_CS2;
		break;
	case 3:
		ind->cs_hdr = CS_OR_HDR_CS3;
		break;
	case 4:
		ind->cs_hdr = CS_OR_HDR_CS4;
		break;
	case 5:
		ind->cs_hdr = CS_OR_HDR_HDR1;
		break;
	case 6:
		ind->cs_hdr = CS_OR_HDR_HDR2;
		break;
	case 7:
		ind->cs_hdr = CS_OR_HDR_HDR3;
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid codec status (E2-E4)\n");
		return -EINVAL;
	}

	/* RX-LEV (E5-E10, 63 in case of AB) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->rx_lev, 2, trau_bits, 77, 6, 0);

	/* Estimated access delay (E11-E13, 0 in case of AB) */
	osmo_ubit2pbit_ext((pbit_t *) &eadd, 5, trau_bits, 83, 3, 0);
	switch (eadd) {
	case 0:
		/* <2 or less */
		ind->est_acc_del_dev = -3;
		break;
	case 1:
		ind->est_acc_del_dev = -1;
		break;
	case 2:
		ind->est_acc_del_dev = 1;
		break;
	case 4:
		ind->est_acc_del_dev = 2;
		break;
	case 5:
		/* >2 or more */
		ind->est_acc_del_dev = 3;
		break;
	case 6:
		ind->est_acc_del_dev = 0;
		break;
	case 7:
		ind->est_acc_del_dev = -2;
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid estimated access delay (E11-E13)\n");
		return -EINVAL;
	}

	if (ind->cs_hdr == CS_OR_HDR_CS1 || ind->cs_hdr == CS_OR_HDR_CS2 || ind->cs_hdr == CS_OR_HDR_CS3
	    || ind->cs_hdr == CS_OR_HDR_CS4) {
		/* Soft frame quality SFQ (E14-E16, 0 in case of AB) */
		osmo_ubit2pbit_ext((pbit_t *) &sfq, 5, trau_bits, 86, 3, 0);
		ind->u.gprs.block_qual = sfq;

		/* Parity Check (E17, 1 in case of AB) */
		ind->u.gprs.parity_ok = trau_bits[89];

	} else {
		/* Mean BEP (E14-E20) */
		osmo_ubit2pbit_ext((pbit_t *) &mean_bep, 1, trau_bits, 86, 7, 0);
		ind->u.egprs.mean_bep = mean_bep;

		/* CV BEP (E21-E23) */
		osmo_ubit2pbit_ext((pbit_t *) &cv_bep, 5, trau_bits, 93, 3, 0);
		ind->u.egprs.mean_bep = cv_bep;

		/* RLC/MAC header quality (E24) */
		if (trau_bits[96] == 0) {
			ind->u.egprs.hdr_good = true;

			/* Data block quality (E25/E26) */
			if (trau_bits[97] == 0)
				ind->u.egprs.data_good[0] = true;
			if (trau_bits[98] == 0)
				ind->u.egprs.data_good[1] = true;
		} else {
			/* A bad RLC/MAC header always means that the the data blocks
			 * cannot be valid. */
			ind->u.egprs.data_good[0] = false;
			ind->u.egprs.data_good[1] = false;
		}

	}

	/* Data bits */
	switch (ind->cs_hdr) {
	case CS_OR_HDR_CS1:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 184, 1);
		rc = osmo_crc16gen_check_bits(&cs1_crc16, trau_bits + 134, 184, trau_bits + 134 + 184);
		if (rc != 0) {
			LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: CRC error in CS1 block\n");
			return -EINVAL;
		}
		ind->data_len = 23;
		break;
	case CS_OR_HDR_CS2:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 271, 1);
		ind->data_len = 34;
		break;
	case CS_OR_HDR_CS3:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 315, 1);
		ind->data_len = 40;
		break;
	case CS_OR_HDR_CS4:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 431, 1);
		ind->data_len = 54;
		break;
	case CS_OR_HDR_HDR1:
	case CS_OR_HDR_HDR2:
	case CS_OR_HDR_HDR3:
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 40, 1);
		cps = cps_from_mcs_block(ind->data, ind->cs_hdr, true);
		if (cps < 0) {
			LOGP(DLINP, LOGL_NOTICE,
			     "CCU-DATA-IND-64: unable to read CPS from data block, bad data block received?\n");
			break;
		}
		mcs = mcs_from_cps((uint8_t) cps, ind->cs_hdr);
		if (mcs < 0) {
			LOGP(DLINP, LOGL_NOTICE,
			     "CCU-DATA-IND-64: unable to determine coding scheme (MCS) from CPS, bad data block received?\n");
			break;
		}

		/* Note: receiving noise (and eventually bad CPS field, may
		 * happen from time to time and is not an error condition. */

		switch (mcs) {
		case 1:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 209, 1);
			ind->data_len = 27;
			break;
		case 2:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 257, 1);
			ind->data_len = 33;
			break;
		case 3:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 329, 1);
			ind->data_len = 42;
			break;
		case 4:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 385, 1);
			ind->data_len = 49;
			break;
		case 5:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 487, 1);
			ind->data_len = 61;
			break;
		case 6:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 631, 1);
			ind->data_len = 79;
			break;
		case 7:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 946, 1);
			ind->data_len = 119;
			break;
		case 8:
			osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 134, 1138, 1);
			ind->data_len = 143;
			break;
		default:
			LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid cs_hdr set\n");
			return -EINVAL;
		}
		break;
	case CS_OR_HDR_AB:
		/* Note: The useful data starts at D13 and is byte-aligned inside the TRAU frame.
		 * The data string contains 4 items, each 8 bytes long, 32 bytes total. */
		osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, trau_bits, 144, 256, 1);
		ind->data_len = 32;

		for (i = 0; i < 4; i++) {
			rc = decode_ab(&ind->ab[i], ind->data + i * 8);
			if (rc < 0)
				return -EINVAL;
		}
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid cs_hdr set\n");
		return -EINVAL;
	}

	return 0;
};

static int dec_ccu_data_ind_64_mcs9(struct er_ccu_data_ind *ind, const ubit_t *trau_bits)
{
	/* 64kbps CCU-DATA-IND (MCS9) TRAU frame format:
	 * Direction: CCU => PCU
	 *
	 * (bit 0) 0     0     0     0     0     0     0     0
	 *         0     0     0     0     0     0     0     0
	 *         1     D1    D2    D3    D4    D5    D6    D7
	 *         D8    D9    D10   D11   D12   D13   D14   D15
	 *         D16   D17   D18   D19   D20   D21   D22   D23
	 *         D24   D25   D26   D27   D28   D29   D30   D31
	 *         D32   D33   D34   D35   D36   D37   D38   D39
	 *         D40   D41   D42   D43   D44   D45   D46   D47
	 *         D48   C1    C2    C3    C4    C5    C6    C7
	 *         PC    E1    E2    E3    E4    E5    E6    E7
	 *         E8    E9    E10   E11   E12   E13   E14   E15
	 *         E16   E17   E18   E19   E20   E21   E22   E23
	 *         PE    S1    S2    D49   D50   D51   D52   D53
	 *         D54   D55   D56   D57   D58   D59   D60   D61
	 *         D62   D63   D64   ...
	 *                                 ...   D1219 D1220 D1221
	 *         TA1   TA2   TA3   TA4   TA5   TA6   TA7   TA8
	 *         TA9   TA10  TA11  TA12  TA13  TA14  TA15  TA16 (bit 1279)
	 *         TA17* TA18* TA19* TA20* TA21* TA22* TA23* TA24*
	 *         TA25* TA26* TA27* TA28* TA29* TA30* TA31* TA32*
	 *
	 * C = Control bits
	 * D = Data bits
	 * E = Extended control bits
	 * PC = Parity over C-Bits (odd)
	 * PE = Parity over extended control bits (odd) */

	ubit_t block[1234];
	uint8_t eadd = 0;
	uint8_t mean_bep = 0;
	uint8_t cv_bep = 0;

	/* Validate C-Bits */
	if (calc_parity(trau_bits + 65, 7) != trau_bits[72]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid parity (C1-C7)\n");
		return -EINVAL;
	}

	/* TAV (C6-C7) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->tav, 0, trau_bits, 70, 2, 1);

	/* Validate E-Bits */
	if (calc_parity(trau_bits + 73, 23) != trau_bits[96]) {
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid parity (E1-E23)\n");
		return -EINVAL;
	}

	/* Downlink block error (E1) */
	if (trau_bits[73] == 0)
		ind->dbe = true;

	/* MCS9 uses CS header type 1 */
	ind->cs_hdr = CS_OR_HDR_HDR1;

	/* RX-LEV (E2-E7) */
	osmo_ubit2pbit_ext((pbit_t *) &ind->rx_lev, 2, trau_bits, 74, 6, 0);

	/* Estimated access delay (E8-E10) */
	osmo_ubit2pbit_ext((pbit_t *) &eadd, 5, trau_bits, 80, 3, 0);
	switch (eadd) {
	case 0:
		/* <2 or less */
		ind->est_acc_del_dev = -3;
		break;
	case 1:
		ind->est_acc_del_dev = -1;
		break;
	case 2:
		ind->est_acc_del_dev = 1;
		break;
	case 4:
		ind->est_acc_del_dev = 2;
		break;
	case 5:
		/* >2 or more */
		ind->est_acc_del_dev = 3;
		break;
	case 6:
		ind->est_acc_del_dev = 0;
		break;
	case 7:
		ind->est_acc_del_dev = -2;
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid estimated access delay (E11-E13)\n");
		return -EINVAL;
	}

	/* Mean BEP (E11-E17) */
	osmo_ubit2pbit_ext((pbit_t *) &mean_bep, 1, trau_bits, 83, 7, 0);
	ind->u.egprs.mean_bep = mean_bep;

	/* CV BEP (E18-E20) */
	osmo_ubit2pbit_ext((pbit_t *) &cv_bep, 5, trau_bits, 90, 3, 0);
	ind->u.egprs.mean_bep = cv_bep;

	/* RLC/MAC header quality (E21) */
	if (trau_bits[93] == 0) {
		ind->u.egprs.hdr_good = true;

		/* Data block quality (E22/E23) */
		if (trau_bits[94] == 0)
			ind->u.egprs.data_good[0] = true;
		if (trau_bits[95] == 0)
			ind->u.egprs.data_good[1] = true;
	} else {
		/* A bad RLC/MAC header always means that the the data blocks
		 * cannot be valid. */
		ind->u.egprs.data_good[0] = false;
		ind->u.egprs.data_good[1] = false;
	}

	/* For capacity reasons the following fields are stripped from the
	 * header: Spare, RSB, CPS. This means we have to restore those
	 * fields to get a valid MCS9 block. See also: 3GPP TS 44.060,
	 * section 10.3a.4.1 */
	memcpy(block, trau_bits + 17, 32);
	block[32] = 0;		/* CPS 0 */
	block[33] = 0;		/* CPS 1 */
	block[34] = 0;		/* CPS 2 */
	block[35] = 0;		/* CPS 3 */
	block[36] = 0;		/* CPS 4 */
	block[37] = 0;		/* RSB (not a resent block - guessed) */
	block[38] = trau_bits[49];	/* PI */
	block[39] = 0;		/* Spare */
	block[40] = 0;		/* Spare */
	block[41] = 0;		/* Spare */
	block[42] = 0;		/* Spare */
	block[43] = 0;		/* Spare */
	block[44] = 0;		/* Spare */
	block[45] = 0;		/* Spare */
	memcpy(block + 45, trau_bits + 50, 15);
	memcpy(block + 45, trau_bits + 99, 1173);

	osmo_ubit2pbit_ext((pbit_t *) &ind->data, 0, block, 0, sizeof(block), 1);

	return 0;
}

/*! decode an 64k Ericsson GPRS (GSL) TRAU frame.
 *  \param[out] fr caller-allocated output data structure.
 *  \param[in] bits unpacked input bits (1280).
 *  \return 0 on success; negative in case of error. */
int er_gprs_trau_frame_decode_64k(struct er_gprs_trau_frame *fr, const ubit_t *bits)
{
	uint8_t c_1_5 = 0;
	const ubit_t expected_sync_pattern[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1,
	};
	const ubit_t expected_sync_pattern_mcs9[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1,
	};
	int rc;

	memset(fr, 0, sizeof(*fr));

	/* Determine frame type */
	osmo_ubit2pbit_ext((pbit_t *) &c_1_5, 0, bits, 65, 5, 1);

	/* Validate sync patern */
	switch (c_1_5) {
	case PCU_TRAU_ER_FT_CCU_SYNC_IND:
	case PCU_TRAU_ER_FT_DATA_IND:
		if (memcmp(bits, expected_sync_pattern, ARRAY_SIZE(expected_sync_pattern)) != 0) {
			LOGP(DLINP, LOGL_ERROR, "CCU-XXXX-IND-64: invalid sync pattern (T0,T1)\n");
			return -EINVAL;
		}
		break;
	case PCU_TRAU_ER_FT_DATA9_IND:
		if (memcmp(bits, expected_sync_pattern_mcs9, ARRAY_SIZE(expected_sync_pattern_mcs9)) != 0) {
			LOGP(DLINP, LOGL_ERROR, "CCU-DATA-IND-64: invalid sync pattern (T0,T1)\n");
			return -EINVAL;
		}
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-XXXX-IND-64: invalid frame type (%02x)\n", c_1_5);
		rc = -EINVAL;
	}

	/* Decode frame */
	switch (c_1_5) {
	case PCU_TRAU_ER_FT_CCU_SYNC_IND:
		fr->type = ER_GPRS_TRAU_FT_SYNC;
		rc = dec_ccu_sync_ind_64(&fr->u.ccu_sync_ind, bits);
		break;
	case PCU_TRAU_ER_FT_DATA_IND:
		fr->type = ER_GPRS_TRAU_FT_DATA;
		rc = dec_ccu_data_ind_64(&fr->u.ccu_data_ind, bits);
		break;
	case PCU_TRAU_ER_FT_DATA9_IND:
		fr->type = ER_GPRS_TRAU_FT_DATA;
		rc = dec_ccu_data_ind_64_mcs9(&fr->u.ccu_data_ind, bits);
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "CCU-XXXX-IND-64: invalid frame type (%02x)\n", c_1_5);
		rc = -EINVAL;
	}

	/* Ensure that we exit with predictable data in case of error. */
	if (rc < 0)
		memset(fr, 0, sizeof(*fr));

	return rc;
}

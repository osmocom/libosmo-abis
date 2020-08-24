/* TRAU frame to RTP conversion */

/* (C) 2009,2020 by Harald Welte <laforge@gnumonks.org>
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <osmocom/core/crc8gen.h>
#include <osmocom/codec/codec.h>
#include <osmocom/netif/amr.h>

#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>

/* this corresponds to the bit-lengths of the individual codec
 * parameters as indicated in Table 1.1 of TS 46.010 */
static const uint8_t gsm_fr_map[] = {
	6, 6, 5, 5, 4, 4, 3, 3,
	7, 2, 2, 6, 3, 3, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 7, 2, 2, 6, 3, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 7, 2, 2, 6, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 3, 7, 2, 2, 6, 3,
	3, 3, 3, 3, 3, 3, 3, 3,
	3, 3, 3, 3
};


/*
 * EFR TRAU parity
 *
 * g(x) = x^3 + x^1 + 1
 */
static const struct osmo_crc8gen_code gsm0860_efr_crc3 = {
	.bits = 3,
	.poly = 0x3,
	.init = 0x0,
	.remainder = 0x7,
};

/* re-combine EFR parity bits */
static inline void efr_parity_bits_1(ubit_t *check_bits, const ubit_t *d_bits)
{
	memcpy(check_bits + 0 , d_bits + 0, 22);
	memcpy(check_bits + 22 , d_bits + 24, 3);
	check_bits[25] = d_bits[28];
}

static inline void efr_parity_bits_2(ubit_t *check_bits, const ubit_t *d_bits)
{
	memcpy(check_bits + 0 , d_bits + 42, 10);
	memcpy(check_bits + 10 , d_bits + 90, 2);
}

static inline void efr_parity_bits_3(ubit_t *check_bits, const ubit_t *d_bits)
{
	memcpy(check_bits + 0 , d_bits + 98, 5);
	check_bits[5] = d_bits[104];
	memcpy(check_bits + 6 , d_bits + 143, 2);
}

static inline void efr_parity_bits_4(ubit_t *check_bits, const ubit_t *d_bits)
{
	memcpy(check_bits + 0 , d_bits + 151, 10);
	memcpy(check_bits + 10 , d_bits + 199, 2);
}

static inline void efr_parity_bits_5(ubit_t *check_bits, const ubit_t *d_bits)
{
	memcpy(check_bits + 0 , d_bits + 207, 5);
	check_bits[5] = d_bits[213];
	memcpy(check_bits + 6 , d_bits + 252, 2);
}

//static const uint8_t c_bits_check_fr[] = { 0, 0, 0, 1, 0 };
//static const uint8_t c_bits_check_efr[] = { 1, 1, 0, 1, 0 };

/*! Generate the 33 bytes RTP payload for GSM-FR from a decoded TRAU frame.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] fr input TRAU frame in decoded form
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_fr(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf)
{
	int i, j, k, l, o;

	if (tf->type != OSMO_TRAU16_FT_FR)
		return -EINVAL;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	if (tf->c_bits[11]) /* BFI */
		return 0;

	if (out_len < GSM_FR_BYTES)
		return -ENOSPC;

	out[0] = 0xd << 4;
	/* reassemble d-bits */
	i = 0; /* counts bits */
	j = 4; /* counts output bits */
	k = gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset input bits */
	while (i < 260) {
		out[j/8] |= (tf->d_bits[k+o] << (7-(j%8)));
		/* to avoid out-of-bounds access in gsm_fr_map[++l] */
		if (i == 259)
			break;
		if (--k < 0) {
			o += gsm_fr_map[l];
			k = gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}

	return GSM_FR_BYTES;
}

/* See Section 5.2 of RFC5993 */
enum rtp_hr_ietf_ft {
	FT_GOOD_SPEECH  = 0,
	FT_GOOD_SID     = 2,
	FT_NO_DATA      = 7,
};

static const uint8_t rtp_hr_sid[14] = { 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

/*! Generate the 14 bytes ETSI TS 101 318 RTP payload for HR from a decoded 16k TRAU frame.
 *  Note that thsi differs from the IETF RFC5993 format.  However, as OsmoBTS implements
 *  the TS 101 318 format, we also use the same format here. osmo-mgw can convert them.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] tf input TRAU frame in decoded form
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_hr16(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf)
{
	unsigned int i;

	if (tf->type != OSMO_TRAU16_FT_HR)
		return -EINVAL;

	/* HR Data Bits according to TS 48.061 Section 5.1.4.1.1 */

	if (tf->dir == OSMO_TRAU_DIR_UL && tf->c_bits[11]) /* C12: BFI */
		goto bad_frame;

	if (out_len < GSM_HR_BYTES)
		return -ENOSPC;

	/* TS 101 318 Section 5.2: The order of occurrence of the codec parameters in the buffer is
	 * the same as order of occurrence over the Abis as defined in annex B of ETS 300 969
	 * [which is 3GPP TS 46.020 */
	osmo_ubit2pbit(out, tf->d_bits, 112);

	if (tf->c_bits[12] || tf->c_bits[13]) {
		/* Generate SID frame as per TS 101 318 Section 5.2.2 */
		for (i = 0; i < sizeof(rtp_hr_sid); i++)
			out[i] = out[i] | rtp_hr_sid[i];
	}

	return GSM_HR_BYTES;

bad_frame:
	return 0;
}

/*! Generate the 31 bytes RTP payload for GSM-EFR from a decoded TRAU frame.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] fr input TRAU frame in decoded form
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_efr(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf)
{
	int i, j, rc;
	ubit_t check_bits[26];

	if (tf->type != OSMO_TRAU16_FT_EFR)
		return -EINVAL;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	if (tf->c_bits[11]) /* BFI */
		return 0;

	if (out_len < GSM_EFR_BYTES)
		return -ENOSPC;

	if (tf->c_bits[11]) /* BFI */
		goto bad_frame;

	out[0] = 0xc << 4;
	/* reassemble d-bits */
	for (i = 1, j = 4; i < 39; i++, j++)
		out[j/8] |= (tf->d_bits[i] << (7-(j%8)));
	efr_parity_bits_1(check_bits, tf->d_bits);
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, check_bits, 26,
			tf->d_bits + 39);
	if (rc)
		goto bad_frame;
	for (i = 42, j = 42; i < 95; i++, j++)
		out[j/8] |= (tf->d_bits[i] << (7-(j%8)));
	efr_parity_bits_2(check_bits, tf->d_bits);
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, check_bits, 12,
			tf->d_bits + 95);
	if (rc)
		goto bad_frame;
	for (i = 98, j = 95; i < 148; i++, j++)
		out[j/8] |= (tf->d_bits[i] << (7-(j%8)));
	efr_parity_bits_3(check_bits, tf->d_bits);
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, check_bits, 8,
			tf->d_bits + 148);
	if (rc)
		goto bad_frame;
	for (i = 151, j = 145; i < 204; i++, j++)
		out[j/8] |= (tf->d_bits[i] << (7-(j%8)));
	efr_parity_bits_4(check_bits, tf->d_bits);
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, check_bits, 12,
			tf->d_bits + 204);
	if (rc)
		goto bad_frame;
	for (i = 207, j = 198; i < 257; i++, j++)
		out[j/8] |= (tf->d_bits[i] << (7-(j%8)));
	efr_parity_bits_5(check_bits, tf->d_bits);
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, check_bits, 8,
			tf->d_bits + 257);
	if (rc)
		goto bad_frame;

	return GSM_EFR_BYTES;
bad_frame:
	return 0;
}

/* TS 48.060 Section 5.5.1.1.2 */
static int rtp2trau_fr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	int i, j, k, l, o;

	/* data_len == 0 for BFI frame */
	if (data_len < GSM_FR_BYTES && data_len != 0)
		return -EINVAL;

	if (data_len && data[0] >> 4 != 0xd)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_FR;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* FIXME: Generate SID frames? */

	/* set c-bits and t-bits */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* C1 .. C5 */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 0;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 0;
	} else {
		/* C1 .. C5 */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 0;
		tf->c_bits[4] = 0;
	}
	memset(&tf->c_bits[5], 0, 6);	/* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		if (data_len == 0)
			tf->c_bits[11] = 1;	/* C12: BFI */
		else
			tf->c_bits[11] = 0;	/* C12: BFI */
		tf->c_bits[12] = 0; /* C13: SID=0 */
		tf->c_bits[13] = 0; /* C14: SID=0 */
		tf->c_bits[14] = 0; /* C15: TAF (SACCH or not) */
		tf->c_bits[15] = 1; /* C16: spare */
		tf->c_bits[16] = 0; /* C17: DTXd not applied */
	} else {
		memset(&tf->c_bits[11], 1, 10); /* C12 .. C15: spare */
		tf->c_bits[15] = 1; /* C16: SP=1 */
	}
	memset(&tf->c_bits[17], 1, 4); /* C18 .. C12: spare */
	memset(&tf->t_bits[0], 1, 4);

	if (!data_len)
		return 0;

	/* reassemble d-bits */
	i = 0; /* counts bits */
	j = 4; /* counts input bits */
	k = gsm_fr_map[0]-1; /* current number bit in element */
	l = 0; /* counts element bits */
	o = 0; /* offset output bits */
	while (i < 260) {
		tf->d_bits[k+o] = (data[j/8] >> (7-(j%8))) & 1;
		/* to avoid out-of-bounds access in gsm_fr_map[++l] */
		if (i == 259)
			break;
		if (--k < 0) {
			o += gsm_fr_map[l];
			k = gsm_fr_map[++l]-1;
		}
		i++;
		j++;
	}

	return 0;
}

/* does the RTP HR payload resemble a SID frame or not */
static bool is_rtp_hr_sid(const uint8_t *data, const uint8_t data_len)
{
	int i;

	if (data_len < GSM_HR_BYTES)
		return false;

	for (i = 0; i < GSM_HR_BYTES; i++) {
		if ((data[i] & rtp_hr_sid[i]) != rtp_hr_sid[i])
			return false;
	}
	return true;
}

static int rtp2trau_hr16(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	if (data_len < GSM_HR_BYTES && data_len != 0)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_HR;

	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* C1 .. C5 */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 0;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 1;
	} else {
		/* C1 .. C5 */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 0;
		tf->c_bits[4] = 1;
	}
	/* C6.. C11: Time Alignment */
	memset(tf->c_bits + 5, 0, 6);
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* BFI */
		if (data_len == 0)
			tf->c_bits[11] = 1;
		else
			tf->c_bits[11] = 0;
		if (is_rtp_hr_sid(data, data_len)) {
			/* SID=2 is a valid SID frame */
			tf->c_bits[12] = 1;
			tf->c_bits[13] = 0;
		} else {
			tf->c_bits[12] = 0;
			tf->c_bits[13] = 0;
		}
		/* FIXME: C15: TAF */
		tf->c_bits[15] = 0; /* C16: SP */
		tf->c_bits[16] = 0; /* C17: DTXd shall not be applied */
	} else {
		tf->c_bits[11] = 0; /* C12: UFE */
		tf->c_bits[12] = 1; /* C13: spare */
		tf->c_bits[13] = 1; /* C14: spare */
		tf->c_bits[14] = 1; /* C15: spare */
		if (is_rtp_hr_sid(data, data_len))
			tf->c_bits[15] = 0; /* C16: SP */
		else
			tf->c_bits[15] = 1; /* C16: SP */
		tf->c_bits[16] = 1; /* C17: spare */
	}
	memset(tf->c_bits+17, 1, 4); /* C18..C21: spare */
	memset(&tf->t_bits[0], 1, 4);
	if (tf->dir == OSMO_TRAU_DIR_UL)
		tf->ufi = 0;
	else
		tf->ufi = 1;
	/* CRC is computed by TRAU frame encoder */
	if (data_len)
		memcpy(tf->d_bits, data, GSM_HR_BYTES);

	return 0;
}

/* TS 48.060 Section 5.5.1.1.2 */
static int rtp2trau_efr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	int i, j;
	ubit_t check_bits[26];

	/* data_len == 0 for BFI frame */
	if (data_len < GSM_EFR_BYTES && data_len != 0)
		return -EINVAL;

	if (data_len && data[0] >> 4 != 0xc)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_EFR;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* set c-bits and t-bits */
	tf->c_bits[0] = 1;
	tf->c_bits[1] = 1;
	tf->c_bits[2] = 0;
	tf->c_bits[3] = 1;
	tf->c_bits[4] = 0;

	memset(&tf->c_bits[5], 0, 6); /* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		if (data_len == 0)
			tf->c_bits[11] = 1; /* C12: BFI=1 */
		else
			tf->c_bits[11] = 0; /* C12: BFI=1 */
		/* FIXME: Generate SID frames? */
		tf->c_bits[12] = 0; /* C13: SID=0 */
		tf->c_bits[13] = 0; /* C14: SID=0 */
		tf->c_bits[14] = 0; /* C15: TAF (SACCH) */
		tf->c_bits[15] = 1; /* C16: spare */
		tf->c_bits[16] = 0; /* C17: DTXd applied */
	} else {
		tf->c_bits[11] = 1; /* C12: UFE (good uplink) */
		memset(&tf->c_bits[12], 1, 3);	/* C13 .. C15: spare */
		tf->c_bits[15] = 1; /* C16: SP=1 */
		tf->c_bits[16] = 1; /* C17: spare */
	}
	memset(&tf->c_bits[17], 1, 4);	/* C18 .. C21: spare */
	memset(&tf->t_bits[0], 1, 4);

	if (data_len == 0)
		return 0;

	/* reassemble d-bits */
	tf->d_bits[0] = 1;
	for (i = 1, j = 4; i < 39; i++, j++)
		tf->d_bits[i] = (data[j/8] >> (7-(j%8))) & 1;
	efr_parity_bits_1(check_bits, tf->d_bits);
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, check_bits, 26,
			tf->d_bits + 39);
	for (i = 42, j = 42; i < 95; i++, j++)
		tf->d_bits[i] = (data[j/8] >> (7-(j%8))) & 1;
	efr_parity_bits_2(check_bits, tf->d_bits);
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, check_bits, 12,
			tf->d_bits + 95);
	for (i = 98, j = 95; i < 148; i++, j++)
		tf->d_bits[i] = (data[j/8] >> (7-(j%8))) & 1;
	efr_parity_bits_3(check_bits, tf->d_bits);
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, check_bits, 8,
			tf->d_bits + 148);
	for (i = 151, j = 145; i < 204; i++, j++)
		tf->d_bits[i] = (data[j/8] >> (7-(j%8))) & 1;
	efr_parity_bits_4(check_bits, tf->d_bits);
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, check_bits, 12,
			tf->d_bits + 204);
	for (i = 207, j = 198; i < 257; i++, j++)
		tf->d_bits[i] = (data[j/8] >> (7-(j%8))) & 1;
	efr_parity_bits_5(check_bits, tf->d_bits);
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, check_bits, 8,
			tf->d_bits + 257);

	return 0;
}

/* memcpy + increment index */
static inline void memcpy_inc(uint8_t *out, const uint8_t *in, size_t len, unsigned int *idx)
{
	memcpy(out, in, len);
	*idx += len;
}

#define S_FROM_D(sbits, dbits, d_from, d_to, idx)  \
	memcpy_inc(sbits + idx, dbits + (d_from-1), d_to - (d_from-1), &idx)

/* extract the AMR s-bits (codec parameters) from the TRAU frame D-bits */
static int amr_speech_extract_sbits(ubit_t *s_bits, const struct osmo_trau_frame *tf,
				    enum osmo_amr_type mode)
{
	unsigned int s_idx = 0;

	switch (mode) {
	case AMR_4_75:
		S_FROM_D(s_bits, tf->d_bits,  45,  67, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  68,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 108, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 112, 132, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 136, 148, s_idx);
		break;
	case AMR_5_15:
		S_FROM_D(s_bits, tf->d_bits,  47,  69, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  70,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 114, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 118, 136, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 140, 158, s_idx);
		break;
	case AMR_5_90:
		S_FROM_D(s_bits, tf->d_bits,  42,  67, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  68,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 116, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 120, 144, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 148, 168, s_idx);
		break;
	case AMR_6_70:
		S_FROM_D(s_bits, tf->d_bits,  38,  63, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  64,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 120, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 124, 152, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 156, 180, s_idx);
		break;
	case AMR_7_40:
		S_FROM_D(s_bits, tf->d_bits,  35,  60, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  61,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 124, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 128, 159, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 163, 191, s_idx);
		break;
	case AMR_7_95:
		S_FROM_D(s_bits, tf->d_bits,  32,  58, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  59,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 127, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 131, 164, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 168, 199, s_idx);
		break;
	case AMR_10_2:
		S_FROM_D(s_bits, tf->d_bits,  21,  46, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  47,  92, s_idx);
		S_FROM_D(s_bits, tf->d_bits,  96, 138, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 142, 187, s_idx);
		S_FROM_D(s_bits, tf->d_bits, 191, 233, s_idx);
		break;
	case AMR_12_2:
		S_FROM_D(s_bits, tf->d_bits,   1,  38, s_idx);  /* D1..D38 */
		S_FROM_D(s_bits, tf->d_bits,  39,  91, s_idx);	/* D39..D91 */
		S_FROM_D(s_bits, tf->d_bits,  95, 144, s_idx);	/* D95..D144 */
		S_FROM_D(s_bits, tf->d_bits, 148, 200, s_idx);	/* D148..D200 */
		S_FROM_D(s_bits, tf->d_bits, 204, 253, s_idx);	/* D204..D253 */
		break;
	default:
		osmo_panic("unknown AMR speech mode: %u", mode);
		break;
	}

	return s_idx;
}

#define D_FROM_S(dbits, sbits, d_from, d_to, idx)  \
	memcpy_inc(dbits + (d_from-1), sbits + idx, d_to - (d_from-1), &idx)

/* encode the AMR TRAU frame D-bits from the s-bits (codec parameters) */
static int amr_speech_encode_sbits(struct osmo_trau_frame *tf, const ubit_t *s_bits, enum osmo_amr_type mode)
{
	unsigned int s_idx = 0;

	switch (mode) {
	case AMR_4_75:
		D_FROM_S(tf->d_bits, s_bits,  45,  67, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  68,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 108, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 112, 132, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 136, 148, s_idx);
		break;
	case AMR_5_15:
		D_FROM_S(tf->d_bits, s_bits,  47,  69, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  70,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 114, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 118, 136, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 140, 158, s_idx);
		break;
	case AMR_5_90:
		D_FROM_S(tf->d_bits, s_bits,  42,  67, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  68,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 116, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 124, 152, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 156, 180, s_idx);
		break;
	case AMR_6_70:
		D_FROM_S(tf->d_bits, s_bits,  38,  63, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  64,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 120, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 124, 152, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 156, 180, s_idx);
		break;
	case AMR_7_40:
		D_FROM_S(tf->d_bits, s_bits,  35,  60, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  61,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 124, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 128, 159, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 163, 191, s_idx);
		break;
	case AMR_7_95:
		D_FROM_S(tf->d_bits, s_bits,  32,  58, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  59,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 127, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 131, 164, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 168, 199, s_idx);
		break;
	case AMR_10_2:
		D_FROM_S(tf->d_bits, s_bits,  21,  46, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  47,  92, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  96, 138, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 142, 187, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 191, 233, s_idx);
		break;
	case AMR_12_2:
		D_FROM_S(tf->d_bits, s_bits,   1,  38, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  39,  91, s_idx);
		D_FROM_S(tf->d_bits, s_bits,  95, 144, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 148, 200, s_idx);
		D_FROM_S(tf->d_bits, s_bits, 204, 253, s_idx);
	default:
		osmo_panic("unknown AMR speech mode: %u", mode);
		break;
	}
}

/* Frame_Classification as per TS 48.060 Section 5.5.1.2.1 */
enum amr_frame_class {
	AMR_FCLS_SPEECH_GOOD		= 3,
	AMR_FCLS_SPEECH_DEGRADED	= 2,
	AMR_FCLS_SPEECH_BAD		= 1,
	AMR_FCLS_NO_SPEECH		= 0,
};

/* TS 48.060 Section 5.5.1.2.2 */
static int trau2rtp_16(uint8_t *out, const struct osmo_trau_frame *tf, struct osmo_trau2rtp_state *st)
{
	uint8_t frame_class = (tf->c_bits[21-1] << 1) | tf->c_bits[20-1];
	uint8_t cmr_cmi = (tf->c_bits[23-1] << 2) | (tf->c_bits[24-1] << 1) | tf->c_bits[25-1];
	ubit_t rif = tf->c_bits[12-1]; /* Request (1) or Indication (0) Flag */
	uint8_t tac_pac = (tf->c_bits[6-1] << 5) | (tf->c_bits[7-1] << 4) | (tf->c_bits[8-1] << 3) |
			  (tf->c_bits[9-1] << 2) | (tf->c_bits[10-1] << 1) | tf->c_bits[11-1];
	uint8_t no_speech_cls;
	uint8_t s_bits[244];
	uint8_t d_bits[244];
	bool bad_frame = false;
	int n_sbits = 0;

	if (tf->type != OSMO_TRAU16_FT_AMR)
		return -EINVAL;

	if (rif == 0) {
		/* peer (BTS) tells us CMI */
		st->amr.last_cmi = cmr_cmi;
	} else {
		/* peer (BTS) tells us CMR */
		st->amr.last_cmr = cmr_cmi;
	}

	if (tac_pac == 0x3d) {
		/* FIXME: BTS requests us to invert CMI/CMR phase in downlink */
	}

	switch (frame_class) {
	case AMR_FCLS_NO_SPEECH:
		no_speech_cls = (tf->d_bits[32-1] << 2) | (tf->d_bits[33-1] << 1) | tf->d_bits[34-1];
		st->amr.last_cmi = (tf->d_bits[35-1] << 2) | (tf->d_bits[36-1] < 1) || tf->d_bits[37-1];
		st->amr.last_cmr = (tf->d_bits[38-1] << 2) | (tf->d_bits[39-1] < 1) || tf->d_bits[40-1];
		switch (no_speech_cls) {
		case 7: // sid first
			break;
		case 6: // onset
			break;
		case 5: // sid_update
			break;
		case 4: // sid_bad
			break;
		case 0: // no_data
			break;
		/* TOOD: PAB to be treated as PAC */
		/* TODO: Time Alignment Extension (TAE) */
		}
		break;
	case AMR_FCLS_SPEECH_BAD:
		bad_frame = true;
		break;
	case AMR_FCLS_SPEECH_GOOD:
	case AMR_FCLS_SPEECH_DEGRADED:
		/* Extract the s-bits from the TRAU frame */
		n_sbits = amr_speech_extract_sbits(s_bits, tf, st->amr.last_cmi);
		/* Convert the s-bits to d-bits */
		osmo_amr_s_to_d(d_bits, s_bits, n_sbits, st->amr.last_cmi);
		break;
	}

	/* generate octet-aligned RTP AMR header / RFC4867 */
	struct amr_hdr *amrh = (struct amr_hdr *) out;
	amrh->pad1 = 0;
	amrh->cmr = st->amr.last_cmr;
	amrh->pad2 = 0;
	amrh->q = !bad_frame;
	amrh->ft = st->amr.last_cmi;
	amrh->f = 0;

	/* return number of bytes generated */
	return osmo_ubit2pbit(out + sizeof(*amrh), d_bits, n_sbits) + sizeof(*amrh);
}

static int rtp2trau_amr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len,
			struct osmo_trau2rtp_state *st)
{
	struct amr_hdr *amrh = (struct amr_hdr *) data;
	ubit_t s_bits[244];

	if (data_len < sizeof(*amrh))
		return -EIO;

	if (data_len > FIXME)
		data_len = FIXM;

	/* C1..C5: AMR */
	tf->c_bits[1-1] = 0;
	tf->c_bits[2-1] = 0;
	tf->c_bits[3-1] = 1;
	tf->c_bits[4-1] = 1;
	tf->c_bits[5-1] = 0;

	/* C6..C11: TAF: TAC / PAC */
	memset(&tf->c_bits[6-1], 0, 6);


	//tf->c_bits[12-1] = ; /* RIF */
	tf->c_bits[13-1] = 1; /* UFE */
	memset(&tf->c_bits[14-1], 0, 3); /* Config_Prot */
	memset(&tf->c_bits[17-1], 0, 3); /* Message_No */
	tf->c_bits[19-1] = 1; /* spare */
	tf->c_bits[20-1] = 1; /* spare */

#if 0
	tf->c_bits[21-1]
	tf->c_bits[22-1]

	/* CMI (RIF=0) or CMR (RIF=1) */
	tf->c_bits[23-1]
	tf->c_bits[24-1]
	tf->c_bits[25-1]
#endif

	/* TODO: convert s-bits to d-bits */
	osmo_pbit2ubit(s_bits, data + sizeof(*amrh), data_len-sizeof(*amrh));
	amr_speech_encode_sbits(tf, s_bits, mode)

	/* TODO: compute CRC */

	return 0;
}

int trau2rtp_amr(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf, struct osmo_trau2rtp_state *st)
{
	switch (tf->type) {
	case OSMO_TRAU16_FT_AMR:
		return trau2rtp_16(out, tf, st);
	case OSMO_TRAU8_AMR_LOW:
	case OSMO_TRAU8_AMR_6k7:
	case OSMO_TRAU8_AMR_7k4:
	default:
		return -EINVAL;
	}
}

int osmo_trau2rtp(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf,
		  struct osmo_trau2rtp_state *st)
{
	switch (tf->type) {
	case OSMO_TRAU16_FT_FR:
		return trau2rtp_fr(out, out_len, tf);
	case OSMO_TRAU16_FT_EFR:
		return trau2rtp_efr(out, out_len, tf);
	case OSMO_TRAU16_FT_HR:
		return trau2rtp_hr16(out, out_len, tf);
	case OSMO_TRAU16_FT_AMR:
		return trau2rtp_amr(out, out_len, tf, st);
	default:
		return -EINVAL;
	}
}

int osmo_rtp2trau(struct osmo_trau_frame *tf, const uint8_t *rtp, size_t rtp_len,
		  struct osmo_trau2rtp_state *st)
{
	switch (st->type) {
	case OSMO_TRAU16_FT_FR:
		return rtp2trau_fr(tf, rtp, rtp_len);
	case OSMO_TRAU16_FT_EFR:
		return rtp2trau_efr(tf, rtp, rtp_len);
	case OSMO_TRAU16_FT_HR:
		return rtp2trau_hr16(tf, rtp, rtp_len);
	case OSMO_TRAU16_FT_AMR:
		//return rtp2trau_amr(tf, rtp, rtp_len, st);
	default:
		return -EINVAL;
	}
}

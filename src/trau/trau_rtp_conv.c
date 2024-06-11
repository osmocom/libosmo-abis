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

static const uint8_t rtp_hr_sid[14] = { 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

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
	enum osmo_gsm631_sid_class sidc;

	/* data_len == 0 for BFI frame */
	if (data_len < GSM_FR_BYTES && data_len != 0)
		return -EINVAL;

	if (data_len && data[0] >> 4 != 0xd)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_FR;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* set c-bits and t-bits */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* C1 .. C5: FR UL */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 0;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 0;
	} else {			/* DL */
		if (data_len == 0) {
			/* C1 .. C5: idle speech */
			tf->c_bits[0] = 0;
			tf->c_bits[1] = 1;
			tf->c_bits[2] = 1;
			tf->c_bits[3] = 1;
			tf->c_bits[4] = 0;
		} else {
			/* C1 .. C5: FR DL */
			tf->c_bits[0] = 1;
			tf->c_bits[1] = 1;
			tf->c_bits[2] = 1;
			tf->c_bits[3] = 0;
			tf->c_bits[4] = 0;
		}
	}
	memset(&tf->c_bits[5], 0, 6);	/* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		if (data_len == 0) {
			tf->c_bits[11] = 1;	/* C12: BFI */
			tf->c_bits[12] = 0;	/* C13: SID=0 */
			tf->c_bits[13] = 0;	/* C14: SID=0 */
		} else {
			tf->c_bits[11] = 0;	/* C12: BFI */
			/* SID classification per GSM 06.31 section 6.1.1 */
			sidc = osmo_fr_sid_classify(data);
			tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
			tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
		}
		tf->c_bits[14] = 0; /* C15: TAF (SACCH or not) */
		tf->c_bits[15] = 1; /* C16: spare */
		tf->c_bits[16] = 0; /* C17: DTXd not applied */
	} else {
		memset(&tf->c_bits[11], 1, 4);	/* C12 .. C15: spare */
		if (data_len && osmo_fr_check_sid(data, data_len))
			tf->c_bits[15] = 0;	/* C16: SP=0 */
		else
			tf->c_bits[15] = 1;	/* C16: SP=1 */
		tf->c_bits[16] = 1; /* C17: spare */
	}
	memset(&tf->c_bits[17], 1, 4); /* C18 .. C21: spare */
	memset(&tf->t_bits[0], 1, 4);

	if (!data_len) {
		/* idle speech frame if DL, BFI speech frame if UL */
		memset(&tf->d_bits[0], 1, 260);
		return 0;
	}

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
		tf->c_bits[11] = 1; /* C12: UFE */
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
		osmo_pbit2ubit(tf->d_bits, data, GSM_HR_BYTES * 8);
	else
		memset(tf->d_bits, 0, GSM_HR_BYTES * 8);

	return 0;
}

/* TS 48.060 Section 5.5.1.1.2 */
static int rtp2trau_efr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	int i, j;
	ubit_t check_bits[26];
	enum osmo_gsm631_sid_class sidc;

	/* data_len == 0 for BFI frame */
	if (data_len < GSM_EFR_BYTES && data_len != 0)
		return -EINVAL;

	if (data_len && data[0] >> 4 != 0xc)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_EFR;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* set c-bits and t-bits */
	if (data_len == 0 && tf->dir == OSMO_TRAU_DIR_DL) {
		/* C1 .. C5: idle speech */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 0;
	} else {
		/* C1 .. C5: EFR */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 0;
	}

	memset(&tf->c_bits[5], 0, 6); /* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		if (data_len == 0) {
			tf->c_bits[11] = 1;	/* C12: BFI */
			tf->c_bits[12] = 0;	/* C13: SID=0 */
			tf->c_bits[13] = 0;	/* C14: SID=0 */
		} else {
			tf->c_bits[11] = 0;	/* C12: BFI */
			/* SID classification per GSM 06.81 section 6.1.1 */
			sidc = osmo_efr_sid_classify(data);
			tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
			tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
		}
		tf->c_bits[14] = 0; /* C15: TAF (SACCH) */
		tf->c_bits[15] = 1; /* C16: spare */
		tf->c_bits[16] = 0; /* C17: DTXd applied */
	} else {
		tf->c_bits[11] = 1; /* C12: UFE (good uplink) */
		memset(&tf->c_bits[12], 1, 3);	/* C13 .. C15: spare */
		if (data_len && osmo_efr_check_sid(data, data_len))
			tf->c_bits[15] = 0;	/* C16: SP=0 */
		else
			tf->c_bits[15] = 1;	/* C16: SP=1 */
		tf->c_bits[16] = 1; /* C17: spare */
	}
	memset(&tf->c_bits[17], 1, 4);	/* C18 .. C21: spare */
	memset(&tf->t_bits[0], 1, 4);

	if (data_len == 0) {
		/* idle speech frame if DL, BFI speech frame if UL */
		memset(&tf->d_bits[0], 1, 260);
		return 0;
	}

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

#if 0
static inline memcpy_inc(uint8_t *out, const uint8_t *in, size_t len, unsigned int *idx)
{
	memcpy_inc(out, in, len);
	*idx += len;
}

static int amr_speech_extract_sbits(ubit_t *s_bits, const struct osmo_trau_frame *tf,
				    enum osmo_amr_mode mode)
{
	unsigned int s_idx = 0;

	switch (mode) {
	case AMR_4_75:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 44, 67 - 44, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 67, 92 - 67, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 108 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 111, 132 - 111, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 135, 148 - 135, &s_idx);
		break;
	case AMR_5_15:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 46, 96 - 46, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 69, 92 - 69, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 114 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 117, 136 - 117, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 139, 158 - 139, &s_idx);
		break;
	case AMR_5_90:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 41, 67 - 41, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 67, 92 - 67, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 116 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 119, 144 - 119, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 147, 168 - 147, &s_idx);
		break;
	case AMR_6_70:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 37, 63 - 37, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 63, 92 - 63, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 120 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 123, 152 - 123, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 155, 180 - 155, &s_idx);
		break;
	case AMR_7_40:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 34, 60 - 34, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 60, 92 - 60, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 124 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 127, 159 - 127, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 162, 191 - 162, &s_idx);
		break;
	case AMR_7_95:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 31, 58 - 31, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 58, 92 - 58, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 127 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 130, 164 - 130, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 167, 199 - 167, &s_idx);
		break;
	case AMR_10_2:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 20, 46 - 20, &s_idx);	/* D21..D46 */
		memcpy_inc(s_bits + s_idx, tf->d_bits + 46, 92 - 46, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 95, 138 - 95, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 141, 187 - 141, &s_idx);
		memcpy_inc(s_bits + s_idx, tf->d_bits + 190, 233 - 190, &s_idx);
		break;
	case AMR_12_2:
		memcpy_inc(s_bits + s_idx, tf->d_bits + 0, 38 - 0, &s_idx);	/* D1..D38 */
		memcpy_inc(s_bits + s_idx, tf->d_bits + 38, 91 - 38, &s_idx);	/* D39..D91 */
		memcpy_inc(s_bits + s_idx, tf->d_bits + 94, 144 - 94, &s_idx);	/* D95..D144 */
		memcpy_inc(s_bits + s_idx, tf->d_bits + 147, 200 - 147, &s_idx);/* D148..D200 */
		memcpy_inc(s_bits + s_idx, tf->d_bits + 203, 253 - 203, &s_idx);/* D204..D253 */
		break;
	}

	return s_idx;
}

/* TS 48.060 Section 5.5.1.2.2 */
static int trau2rtp_16(uint8_t *out, const struct osmo_trau_frame *tf, enum osmo_amr_mode last_cmi)
{
	enum osmo_amr_mode mode = last_cmi;
	uint8_t frame_class = tf->c_bits[21] << 1 | tf->c_bits[20];
	uint8_t cmr_cmi = tf->c_bits[23] << 2 | tf->c_bits[24] << 1 | tf->cb_bits[25];
	uint8_t no_speech_cls;
	uint8_t s_bits[242];
	uint8_t d_bits[242];
	unsigned int s_idx = 0;
	ubit_t rif = FIXME;

	if (tf->type != OSMO_TRAU16_FT_AMR)
		return -EINVAL;

	if (rif == 0)
		mode = cmr_cmi;

	switch (frame_class) {
	case 0: // no speech
		no_speech_cls = tf->d_bits[32] << 2 | tf->d_bits[33] << 1 | tf->d_bits[34];
		cmi_abs = tf->d_bits[35] << 2 | tf->d_bits[36] < 1 || tf->d_bits[37];
		cmr_abs = tf->d_bits[38] << 2 | tf->d_bits[39] < 1 || tf->d_bits[40];
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
		}
		break;
	case 1: // speech bad
		break;
	case 2:
	case 3:
		/* Extract the s-bits from the TRAU frame */
		amr_speech_extract_sbits(s_bits, tf, mode);
		/* Convert the s-bits to d-bits */
		osmo_amr_s_to_d(d_bits, s_bits, mode);
		break;
	}
}

int trau2rtp_amr(uint8_t *out, const struct osmo_trau_frame *tf, enum osmo_amr_mode last_cmi))
{
	switch (tf->type) {
	case OSMO_TRAU16_FT_AMR:
		return trau2rtp_16(out, tf, last_cmi);
	case OSMO_TRAU8_AMR_LOW:
	case OSMO_TRAU8_AMR_6k7:
	case OSMO_TRAU8_AMR_7k4:
	default:
		return -EINVAL;
	}
}
#endif

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
	default:
		return -EINVAL;
	}
}

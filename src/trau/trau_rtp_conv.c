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
#include <osmocom/gsm/rtp_extensions.h>

#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>

/* RFC4040 "clearmode" RTP payload length */
#define RFC4040_RTP_PLEN 160

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
 * EFR TRAU parity (also used for HR)
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
static int trau2rtp_fr(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf, bool emit_twts001)
{
	size_t req_out_len = emit_twts001 ? GSM_FR_BYTES+1 : GSM_FR_BYTES;
	int i, j, k, l, o;

	if (tf->type != OSMO_TRAU16_FT_FR)
		return -EINVAL;

	if (out_len < req_out_len)
		return -ENOSPC;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* Are we emitting TW-TS-001? If so, emit TRAU-like Extension Header */
	if (emit_twts001) {
		uint8_t hdr_byte = 0xE0;

		if (tf->c_bits[16])	/* DTXd */
			hdr_byte |= 0x08;
		if (tf->c_bits[11])	/* BFI */
			hdr_byte |= 0x02;
		if (tf->c_bits[14])	/* TAF */
			hdr_byte |= 0x01;
		*out++ = hdr_byte;
	}

	if (tf->c_bits[11] && !emit_twts001) /* BFI without TW-TS-001 */
		return 0;

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

	return req_out_len;
}

/* See Section 5.2 of RFC5993 and TW-TS-002 */
enum super5993_ft {
	FT_GOOD_SPEECH		= 0,
	FT_INVALID_SID		= 1,
	FT_GOOD_SID		= 2,
	FT_BFI_WITH_DATA	= 6,
	FT_NO_DATA		= 7,
};

static void twts002_hr16_set_extra_flags(uint8_t *out, const struct osmo_trau_frame *tf)
{
	if (tf->c_bits[16])	/* DTXd */
		out[0] |= 0x08;
	if (tf->ufi)		/* UFI */
		out[0] |= 0x02;
	if (tf->c_bits[14])	/* TAF */
		out[0] |= 0x01;
}

/*! Generate an RFC 5993 RTP payload for HR from a decoded 16k TRAU frame.
 *  We previously emitted TS 101 318 format; however, RFC 5993 is the format
 *  specified in TS 48.103 for AoIP, it can also be extended with TRAU-UL-like
 *  capabilities with TW-TS-002, and we now support RFC 5993 output in OsmoBTS
 *  on all models.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] tf input TRAU frame in decoded form
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_hr16(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf, bool emit_twts002)
{
	enum osmo_gsm631_sid_class sidc;
	int rc;

	if (tf->type != OSMO_TRAU16_FT_HR)
		return -EINVAL;

	if (out_len < GSM_HR_BYTES_RTP_RFC5993)
		return -ENOSPC;

	/* HR Data Bits according to TS 48.061 Section 5.1.4.1.1 */

	/* bad CRC means bad frame, no matter what else is going on */
	rc = osmo_crc8gen_check_bits(&gsm0860_efr_crc3, tf->d_bits, 44,
				     tf->crc_bits);
	if (rc)
		goto bad_frame;

	sidc = (tf->c_bits[12] << 1) | tf->c_bits[13];
	/* both C13 and C14 being set is invalid combination */
	if (sidc > OSMO_GSM631_SID_CLASS_VALID)
		goto bad_frame;

	/* Plain RFC 5993 without TW-TS-002 extensions does not allow
	 * BFI or invalid SID packets. */
	if (!emit_twts002 &&
	    (tf->c_bits[11] || sidc == OSMO_GSM631_SID_CLASS_INVALID))
		goto bad_frame;

	/* BFI turns valid SID into invalid */
	if (tf->c_bits[11] && sidc == OSMO_GSM631_SID_CLASS_VALID)
		sidc = OSMO_GSM631_SID_CLASS_INVALID;

	/* RFC 5993 Frame Type is equal to GSM 06.41 SID classification,
	 * restricted to just speech or valid SID per above.  Or if we
	 * emit TW-TS-002, that restriction is lifted. */
	out[0] = sidc << 4;

	if (emit_twts002) {
		if (tf->c_bits[11] && sidc == OSMO_GSM631_SID_CLASS_SPEECH)
			out[0] = FT_BFI_WITH_DATA << 4;
		twts002_hr16_set_extra_flags(out, tf);
		/* invalid SID frames are truncated in TW-TS-002 */
		if (sidc == OSMO_GSM631_SID_CLASS_INVALID)
			return 1;
	}

	/* TS 101 318 Section 5.2: The order of occurrence of the codec parameters in the buffer is
	 * the same as order of occurrence over the Abis as defined in annex B of ETS 300 969
	 * [which is 3GPP TS 46.020 */
	osmo_ubit2pbit(out + 1, tf->d_bits, 112);

	/* RFC 5993 requires SID frames to be perfect, error-free */
	if (!emit_twts002 && sidc == OSMO_GSM631_SID_CLASS_VALID)
		osmo_hr_sid_reset(out + 1);

	return GSM_HR_BYTES_RTP_RFC5993;

bad_frame:
	if (emit_twts002) {
		out[0] = FT_NO_DATA << 4;
		twts002_hr16_set_extra_flags(out, tf);
		return 1;
	} else
		return 0;
}

static bool hr8_xc_bits_good_parity(const ubit_t *xc_bits)
{
	int i;
	unsigned int sum = 0;

	for (i = 0; i < 6; i++)
		sum += xc_bits[i];
	if (sum & 1)
		return true;	/* odd sum good */
	else
		return false;	/* even sum bad */
}

static void twts002_hr8_set_extra_flags(uint8_t *out, const struct osmo_trau_frame *tf)
{
	if (tf->c_bits[8])	/* DTXd */
		out[0] |= 0x08;
	if (tf->xc_bits[4])	/* UFI */
		out[0] |= 0x02;
}

/*! Generate an RFC 5993 or TW-TS-002 RTP payload for HRv1
 *  from a decoded 8k TRAU frame.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] tf input TRAU frame in decoded form
 *  \param[in] emit_twts002 self-explanatory
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_hr8(uint8_t *out, size_t out_len,
			const struct osmo_trau_frame *tf, bool emit_twts002)
{
	unsigned xc1_4;

	/* function interface preliminaries */
	if (tf->type != OSMO_TRAU8_SPEECH)
		return -EINVAL;
	if (out_len < GSM_HR_BYTES_RTP_RFC5993)
		return -ENOSPC;

	/* Before considering anything else, if we have bad parity in
	 * frame classification bits or bad CRC in payload bits,
	 * we treat this frame as totally invalid: BFI with no data. */
	if (!hr8_xc_bits_good_parity(tf->xc_bits) ||
	    osmo_crc8gen_check_bits(&gsm0860_efr_crc3, tf->d_bits, 44,
				    tf->crc_bits))
		goto bad_frame;

	/* classify this frame per XC1..XC4 */
	xc1_4 = (tf->xc_bits[0] << 3) | (tf->xc_bits[1] << 2) |
		(tf->xc_bits[2] << 1) | (tf->xc_bits[3] << 0);
	switch (xc1_4) {
	case 0:
		/* good speech frame (BFI=0, SID=0) */
		out[0] = FT_GOOD_SPEECH << 4;
		if (emit_twts002)
			twts002_hr8_set_extra_flags(out, tf);
		osmo_ubit2pbit(out + 1, tf->d_bits, 112);
		return GSM_HR_BYTES_RTP_RFC5993;
	case 1:
		/* valid SID frame (BFI=0, SID=2) */
		out[0] = FT_GOOD_SID << 4;
		if (emit_twts002)
			twts002_hr8_set_extra_flags(out, tf);
		osmo_ubit2pbit(out + 1, tf->d_bits, 112);
		/* RFC 5993 requires SID frames to be perfect, error-free */
		if (!emit_twts002)
			osmo_hr_sid_reset(out + 1);
		return GSM_HR_BYTES_RTP_RFC5993;
	case 4:
	case 5:
		/* invalid SID frame (multiple BFI/SID combinations) */
		/* can be represented only in TW-TS-002, not in RFC 5993 */
		if (!emit_twts002)
			return 0;
		out[0] = FT_INVALID_SID << 4;
		twts002_hr8_set_extra_flags(out, tf);
		/* XC4 is TAF with this frame type */
		if (tf->xc_bits[3])
			out[0] |= 0x01;
		return 1;	/* short format per TW-TS-002 */
	case 6:
	case 7:
		/* bad speech frame (BFI=1, SID=0) */
		if (!emit_twts002)
			return 0;
		out[0] = FT_BFI_WITH_DATA << 4;
		twts002_hr8_set_extra_flags(out, tf);
		/* XC4 is TAF with this frame type */
		if (tf->xc_bits[3])
			out[0] |= 0x01;
		osmo_ubit2pbit(out + 1, tf->d_bits, 112);
		return GSM_HR_BYTES_RTP_RFC5993;
	default:
bad_frame:
		/* received garbage, emit BFI with no data */
		if (emit_twts002) {
			out[0] = FT_NO_DATA << 4;
			twts002_hr8_set_extra_flags(out, tf);
			return 1;
		} else
			return 0;
	}
}

/*! Generate the 31 bytes RTP payload for GSM-EFR from a decoded TRAU frame.
 *  \param[out] out caller-provided output buffer
 *  \param[in] out_len length of out buffer in bytes
 *  \param[in] fr input TRAU frame in decoded form
 *  \returns number of bytes generated in 'out'; negative on error. */
static int trau2rtp_efr(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf, bool emit_twts001)
{
	size_t req_out_len = emit_twts001 ? GSM_EFR_BYTES+1 : GSM_EFR_BYTES;
	int i, j, rc;
	ubit_t check_bits[26];
	uint8_t *twts001_hdr;

	if (out_len < req_out_len)
		return -ENOSPC;

	if (tf->type != OSMO_TRAU16_FT_EFR)
		return -EINVAL;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* Are we emitting TW-TS-001? If so, emit TRAU-like Extension Header */
	if (emit_twts001) {
		twts001_hdr = out++;
		*twts001_hdr = 0xE0;
		if (tf->c_bits[16])	/* DTXd */
			*twts001_hdr |= 0x08;
		if (tf->c_bits[11])	/* BFI */
			*twts001_hdr |= 0x02;
		if (tf->c_bits[14])	/* TAF */
			*twts001_hdr |= 0x01;
	}

	if (tf->c_bits[11] && !emit_twts001) /* BFI without TW-TS-001 */
		return 0;

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

	return req_out_len;

bad_frame:
	if (emit_twts001) {
		*twts001_hdr |= 0x06;	/* BFI and No_Data flags */
		return 1;
	} else
		return 0;
}

/* TS 48.060 Section 5.5.1.1.2 */
static int rtp2trau_fr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	int i, j, k, l, o;
	enum osmo_gsm631_sid_class sidc;
	bool bfi = false, taf = false, dtxd = false;

	/* accept TW-TS-001 input */
	if (data_len > 0 && (data[0] & 0xF0) == 0xE0) {
		dtxd = (data[0] & 0x08) != 0;
		bfi  = (data[0] & 0x02) != 0;
		taf  = (data[0] & 0x01) != 0;
		data++;
		data_len--;
	}

	/* now we must have either standard FR or no-data input */
	switch (data_len) {
	case GSM_FR_BYTES:
		if ((data[0] & 0xF0) != 0xD0)
			return -EINVAL;
		break;
	case 0:
		bfi = true;
		break;
	default:
		return -EINVAL;
	}

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
		tf->c_bits[11] = bfi;		/* C12: BFI */
		if (data_len == 0) {
			tf->c_bits[12] = 0;	/* C13: SID=0 */
			tf->c_bits[13] = 0;	/* C14: SID=0 */
		} else {
			/* SID classification per GSM 06.31 section 6.1.1 */
			sidc = osmo_fr_sid_classify(data);
			tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
			tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
		}
		tf->c_bits[14] = taf;  /* C15: TAF (SACCH or not) */
		tf->c_bits[15] = 1;    /* C16: spare */
		tf->c_bits[16] = dtxd; /* C17: DTXd applied or not */
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

static int rtp2trau_hr16(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	/* accept both TS 101 318 and RFC 5993 payloads */
	switch (data_len) {
	case GSM_HR_BYTES:
		break;
	case GSM_HR_BYTES_RTP_RFC5993:
		data++;
		data_len--;
		break;
	case 0:
		/* accept no-data input */
		break;
	default:
		return -EINVAL;
	}

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
		if (osmo_hr_check_sid(data, data_len)) {
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
		if (osmo_hr_check_sid(data, data_len))
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
	if (data_len)
		osmo_pbit2ubit(tf->d_bits, data, GSM_HR_BYTES * 8);
	else
		memset(tf->d_bits, 0, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

	return 0;
}

static int rtp2trau_hr8(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	/* accept both TS 101 318 and RFC 5993 payloads */
	switch (data_len) {
	case GSM_HR_BYTES:
		break;
	case GSM_HR_BYTES_RTP_RFC5993:
		data++;
		data_len--;
		break;
	case 0:
		/* accept no-data input */
		break;
	default:
		return -EINVAL;
	}

	/* FIXME: implement TRAU-UL frame generation if and when
	 * someone actually needs it in a program that uses
	 * this library. */
	if (tf->dir != OSMO_TRAU_DIR_DL)
		return -ENOTSUP;

	tf->type = OSMO_TRAU8_SPEECH;

	/* C1..C5 */
	tf->c_bits[0] = 0;
	tf->c_bits[1] = 0;
	tf->c_bits[2] = 0;
	tf->c_bits[3] = 1;	/* UFE: good uplink */
	tf->c_bits[4] = 0;	/* odd parity */
	/* C6..C9: spare bits */
	memset(tf->c_bits + 5, 1, 4);

	/* XC1..XC6 */
	if (osmo_hr_check_sid(data, data_len)) {
		tf->xc_bits[0] = 0;
		tf->xc_bits[1] = 0;
		tf->xc_bits[2] = 0;
		tf->xc_bits[3] = 1;
		tf->xc_bits[4] = 0;
		tf->xc_bits[5] = 0;	/* odd parity */
	} else {
		tf->xc_bits[0] = 0;
		tf->xc_bits[1] = 0;
		tf->xc_bits[2] = 0;
		tf->xc_bits[3] = 0;
		tf->xc_bits[4] = 0;
		tf->xc_bits[5] = 1;	/* odd parity */
	}

	memset(&tf->t_bits[0], 1, 2);

	if (data_len)
		osmo_pbit2ubit(tf->d_bits, data, GSM_HR_BYTES * 8);
	else
		memset(tf->d_bits, 0, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

	return 0;
}

/* TS 48.060 Section 5.5.1.1.2 */
static int rtp2trau_efr(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	int i, j;
	ubit_t check_bits[26];
	enum osmo_gsm631_sid_class sidc;
	bool bfi = false, taf = false, dtxd = false;

	/* accept TW-TS-001 input */
	if (data_len > 0 && (data[0] & 0xF0) == 0xE0) {
		dtxd = (data[0] & 0x08) != 0;
		bfi  = (data[0] & 0x02) != 0;
		taf  = (data[0] & 0x01) != 0;
		data++;
		data_len--;
	}

	/* now we must have either standard EFR or no-data input */
	switch (data_len) {
	case GSM_EFR_BYTES:
		if ((data[0] & 0xF0) != 0xC0)
			return -EINVAL;
		break;
	case 0:
		bfi = true;
		break;
	default:
		return -EINVAL;
	}

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
		tf->c_bits[11] = bfi;		/* C12: BFI */
		if (data_len == 0) {
			tf->c_bits[12] = 0;	/* C13: SID=0 */
			tf->c_bits[13] = 0;	/* C14: SID=0 */
		} else {
			/* SID classification per GSM 06.81 section 6.1.1 */
			sidc = osmo_efr_sid_classify(data);
			tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
			tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
		}
		tf->c_bits[14] = taf;  /* C15: TAF (SACCH or not) */
		tf->c_bits[15] = 1;    /* C16: spare */
		tf->c_bits[16] = dtxd; /* C17: DTXd applied or not */
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

/*
 * CSD support: converting TRAU frames of type 'data' to 64 kbit/s
 * like the RA part of TRAU, using RTP clearmode representation.
 */

static void trau2v110_bits(ubit_t *out, const ubit_t *in)
{
	int i;

	/* the first 8 bits are sync zeros */
	memset(out, 0, 8);
	out += 8;
	/* for the rest, we have to expand 63 bits into 72 */
	for (i = 0; i < 9; i++) {
		*out++ = 1;
		memcpy(out, in, 7);
		in += 7;
		out += 7;
	}
}

/* intermediate rate 8 kbit/s */
static void trau2v110_ir8(uint8_t *out, const ubit_t *in)
{
	ubit_t ra_bits[80];
	int i;

	trau2v110_bits(ra_bits, in);

	/* RA2: 1 bit per output byte */
	for (i = 0; i < 80; i++)
		out[i] = 0x7F | (ra_bits[i] << 7);
}

/* intermediate rate 16 kbit/s */
static void trau2v110_ir16(uint8_t *out, const ubit_t *in)
{
	ubit_t ra_bits[80];
	int i, o;
	uint8_t b;

	trau2v110_bits(ra_bits, in);

	/* RA2: 2 bits per output byte */
	i = 0;
	for (o = 0; o < 40; o++) {
		b = 0x3F;
		b |= (ra_bits[i++] << 7);
		b |= (ra_bits[i++] << 6);
		out[o] = b;
	}
}

static int trau2rtp_data_fr(uint8_t *out, size_t out_len,
			    const struct osmo_trau_frame *tf)
{
	/* function interface preliminaries */
	if (tf->type != OSMO_TRAU16_FT_DATA)
		return -EINVAL;
	if (out_len < RFC4040_RTP_PLEN)
		return -ENOSPC;

	/* Is it TCH/F9.6 with 16 kbit/s IR,
	 * or TCH/F4.8 or TCH/F2.4 with 8 kbit/s IR? */
	if (tf->c_bits[5]) {
		trau2v110_ir16(out, tf->d_bits);
		trau2v110_ir16(out + 40, tf->d_bits + 63);
		trau2v110_ir16(out + 80, tf->d_bits + 63 * 2);
		trau2v110_ir16(out + 120, tf->d_bits + 63 * 3);
	} else {
		trau2v110_ir8(out, tf->d_bits);
		trau2v110_ir8(out + 80, tf->d_bits + 63 * 2);
	}

	return RFC4040_RTP_PLEN;
}

static int trau2rtp_data_hr16(uint8_t *out, size_t out_len,
			      const struct osmo_trau_frame *tf)
{
	/* function interface preliminaries */
	if (tf->type != OSMO_TRAU16_FT_DATA_HR)
		return -EINVAL;
	if (out_len < RFC4040_RTP_PLEN)
		return -ENOSPC;

	/* Note that Osmocom trau_frame decoding and encoding API
	 * puts the second reduced V.110 frame at d_bits position 63,
	 * unlike 8 kbit/s IR in FR-data frame type where it resides
	 * at d_bits position 63 * 2. */
	trau2v110_ir8(out, tf->d_bits);
	trau2v110_ir8(out + 80, tf->d_bits + 63);

	return RFC4040_RTP_PLEN;
}

static int trau2rtp_data_hr8(uint8_t *out, size_t out_len,
			      const struct osmo_trau_frame *tf)
{
	/* function interface preliminaries */
	if (tf->type != OSMO_TRAU8_DATA)
		return -EINVAL;
	if (out_len < RFC4040_RTP_PLEN)
		return -ENOSPC;

	trau2v110_ir8(out, tf->d_bits);
	trau2v110_ir8(out + 80, tf->d_bits + 63);

	return RFC4040_RTP_PLEN;
}

/*
 * CSD in the opposite direction: from clearmode RTP input
 * to TRAU frame output.
 *
 * Important note about V.110 frame alignment: in the output from trau2rtp
 * process or from OsmoBTS, the 2 or 4 V.110 frames that fit into a 20 ms
 * RTP clearmode packet are always perfectly aligned in that packet.
 * However, there does not seem to be any formally defined policy in
 * Osmocom as to whether this alignment is required in RTP input to
 * OsmoBTS or to the future RTP-to-E1 MGW (OsmoMGW extension or otherwise),
 * or if these processes are required to hunt for arbitrary V.110 frame
 * alignment in their RTP input.
 *
 * Given that the current CSD implementation in OsmoBTS requires perfect
 * V.110 alignment in RTP input, and given that a stateful V.110 alignment
 * hunt implementation cannot be fitted into osmo_rtp2trau API,
 * the approach implemented here is to also require perfect alignment.
 * If our RTP input is not a perfectly aligned pair or quadruple of
 * V.110 frames, we emit an idle data frame (all bits set to 1) per
 * TS 48.060 section 6.5.1 and TS 48.061 section 6.7.4.
 *
 * Also note: in the case of OSMO_TRAU16_FT_DATA output, the caller of
 * osmo_rtp2trau() API must set st->interm_rate_16k (a Boolean flag)
 * appropriately for 8 kbit/s or 16 kbit/s intermediate rate.
 * In the absence of out-of-band information, this flag can be set
 * from control bit C6 of the first received TRAU-UL frame - that is
 * how traditional TRAUs do it.
 */

static bool check_v110_align(const ubit_t *ra_bits)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (ra_bits[i])
			return false;
	}
	for (i = 1; i < 10; i++) {
		if (!ra_bits[i * 8])
			return false;
	}
	return true;
}

static void v110_to_63bits(ubit_t *out, const ubit_t *ra_bits)
{
	memcpy(out, ra_bits + 9, 7);
	memcpy(out + 7, ra_bits + 17, 7);
	memcpy(out + 14, ra_bits + 25, 7);
	memcpy(out + 21, ra_bits + 33, 7);
	memcpy(out + 28, ra_bits + 41, 7);
	memcpy(out + 35, ra_bits + 49, 7);
	memcpy(out + 42, ra_bits + 57, 7);
	memcpy(out + 49, ra_bits + 65, 7);
	memcpy(out + 56, ra_bits + 73, 7);
}

/* intermediate rate 8 kbit/s */
static void rtp2trau_data_ir8(struct osmo_trau_frame *tf, const uint8_t *data,
			      size_t data_len, unsigned second_offset)
{
	ubit_t ra_bits[80 * 2];
	int i;

	if (data_len != RFC4040_RTP_PLEN)
		goto idle_fill;

	/* reverse RA2 first */
	for (i = 0; i < sizeof(ra_bits); i++)
		ra_bits[i] = (data[i] >> 7) & 1;

	/* enforce two properly aligned V.110 frames */
	if (!check_v110_align(ra_bits))
		goto idle_fill;
	if (!check_v110_align(ra_bits + 80))
		goto idle_fill;

	/* all checks passed - copy the payload */
	v110_to_63bits(tf->d_bits, ra_bits);
	v110_to_63bits(tf->d_bits + second_offset, ra_bits + 80);
	return;

idle_fill:
	memset(tf->d_bits, 1, 63);
	memset(tf->d_bits + second_offset, 1, 63);
}

/* intermediate rate 16 kbit/s */
static void rtp2trau_data_ir16(struct osmo_trau_frame *tf, const uint8_t *data,
			       size_t data_len)
{
	ubit_t ra_bits[80 * 4];
	int i, o;

	if (data_len != RFC4040_RTP_PLEN)
		goto idle_fill;

	/* reverse RA2 first */
	o = 0;
	for (i = 0; i < RFC4040_RTP_PLEN; i++) {
		ra_bits[o++] = (data[i] >> 7) & 1;
		ra_bits[o++] = (data[i] >> 6) & 1;
	}

	/* enforce 4 properly aligned V.110 frames */
	if (!check_v110_align(ra_bits))
		goto idle_fill;
	if (!check_v110_align(ra_bits + 80))
		goto idle_fill;
	if (!check_v110_align(ra_bits + 80 * 2))
		goto idle_fill;
	if (!check_v110_align(ra_bits + 80 * 3))
		goto idle_fill;

	/* all checks passed - copy the payload */
	v110_to_63bits(tf->d_bits, ra_bits);
	v110_to_63bits(tf->d_bits + 63, ra_bits + 80);
	v110_to_63bits(tf->d_bits + 63 * 2, ra_bits + 80 * 2);
	v110_to_63bits(tf->d_bits + 63 * 3, ra_bits + 80 * 3);
	return;

idle_fill:
	memset(tf->d_bits, 1, 63 * 4);
}

static int rtp2trau_data_fr(struct osmo_trau_frame *tf, const uint8_t *data,
			    size_t data_len, bool interm_rate_16k)
{
	tf->type = OSMO_TRAU16_FT_DATA;

	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* C1 .. C5: FR data UL */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 0;
		tf->c_bits[4] = 0;
	} else {
		/* C1 .. C5: FR data DL */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 0;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 0;
	}

	if (interm_rate_16k) {
		tf->c_bits[5] = 1;
		rtp2trau_data_ir16(tf, data, data_len);
	} else {
		tf->c_bits[5] = 0;
		rtp2trau_data_ir8(tf, data, data_len, 63 * 2);
		/* remaining data positions are unused in this format */
		memset(tf->d_bits + 63, 1, 63);
		memset(tf->d_bits + 63 * 3, 1, 63);
	}

	/* remaining C bits are unused */
	memset(tf->c_bits + 6, 1, 9);

	return 0;
}

static int rtp2trau_data_hr16(struct osmo_trau_frame *tf, const uint8_t *data,
			      size_t data_len)
{
	tf->type = OSMO_TRAU16_FT_DATA_HR;

	if (tf->dir == OSMO_TRAU_DIR_UL) {
		/* C1 .. C5: HR data UL */
		tf->c_bits[0] = 0;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 0;
		tf->c_bits[3] = 0;
		tf->c_bits[4] = 1;
	} else {
		/* C1 .. C5: HR data DL */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 0;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 1;
		tf->c_bits[4] = 1;
	}

	tf->c_bits[5] = 0;
	/* Note that Osmocom trau_frame decoding and encoding API
	 * puts the second reduced V.110 frame at d_bits position 63,
	 * unlike 8 kbit/s IR in FR-data frame type where it resides
	 * at d_bits position 63 * 2. */
	rtp2trau_data_ir8(tf, data, data_len, 63);

	/* remaining C bits are unused */
	memset(tf->c_bits + 6, 1, 9);

	return 0;
}

static int rtp2trau_data_hr8(struct osmo_trau_frame *tf, const uint8_t *data,
			     size_t data_len)
{
	tf->type = OSMO_TRAU8_DATA;

	/* C1 .. C5: HR data */
	tf->c_bits[0] = 0;
	tf->c_bits[1] = 0;
	tf->c_bits[2] = 1;
	tf->c_bits[3] = 1;
	tf->c_bits[4] = 1;

	rtp2trau_data_ir8(tf, data, data_len, 63);

	return 0;
}

static inline bool check_twts001(struct osmo_trau2rtp_state *st)
{
	if (st->rtp_extensions & OSMO_RTP_EXT_TWTS001)
		return true;
	else
		return false;
}

static inline bool check_twts002(struct osmo_trau2rtp_state *st)
{
	if (st->rtp_extensions & OSMO_RTP_EXT_TWTS002)
		return true;
	else
		return false;
}

int osmo_trau2rtp(uint8_t *out, size_t out_len, const struct osmo_trau_frame *tf,
		  struct osmo_trau2rtp_state *st)
{
	switch (tf->type) {
	case OSMO_TRAU16_FT_FR:
		return trau2rtp_fr(out, out_len, tf, check_twts001(st));
	case OSMO_TRAU16_FT_EFR:
		return trau2rtp_efr(out, out_len, tf, check_twts001(st));
	case OSMO_TRAU16_FT_HR:
		return trau2rtp_hr16(out, out_len, tf, check_twts002(st));
	case OSMO_TRAU16_FT_DATA:
		return trau2rtp_data_fr(out, out_len, tf);
	case OSMO_TRAU16_FT_DATA_HR:
		return trau2rtp_data_hr16(out, out_len, tf);
	case OSMO_TRAU8_SPEECH:
		return trau2rtp_hr8(out, out_len, tf, check_twts002(st));
	case OSMO_TRAU8_DATA:
		return trau2rtp_data_hr8(out, out_len, tf);
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
	case OSMO_TRAU16_FT_DATA:
		return rtp2trau_data_fr(tf, rtp, rtp_len, st->interm_rate_16k);
	case OSMO_TRAU16_FT_DATA_HR:
		return rtp2trau_data_hr16(tf, rtp, rtp_len);
	case OSMO_TRAU8_SPEECH:
		return rtp2trau_hr8(tf, rtp, rtp_len);
	case OSMO_TRAU8_DATA:
		return rtp2trau_data_hr8(tf, rtp, rtp_len);
	default:
		return -EINVAL;
	}
}

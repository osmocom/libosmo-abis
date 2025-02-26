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
#include <osmocom/trau/csd_ra2.h>
#include <osmocom/trau/csd_raa_prime.h>

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

/*
 * This little helper function modifies a marked-bad (BFI=1) GSM-FR payload
 * so it would no longer classify as SID by the bit counting rules
 * of GSM 06.31 section 6.1.1.  It is needed at times when a BTS emits
 * "BFI with no data", out-of-band SID bits (C13 & C14) are set to 0,
 * but the stale bit pattern in the output buffer used by the BTS
 * happens to be (usually invalid) SID.
 *
 * In order to reliably "break the SID", we need to set 16 bits in the
 * SID field to values opposite of SID codeword, i.e., to 1 for GSM-FR.
 * The SID field of GSM-FR includes the msb of every xMc pulse, and also
 * the middle bit of most (but not all) xMc pulses.  The pulses whose
 * middle bit is not part of the SID field are the last 9 pulses of
 * the last subframe.  For simplicity, we produce the desired effect
 * by setting the middle bit of pulses 0, 1, 2 and 3 in every subframe,
 * for a total of 16 bits.
 */
static void make_fr_bfi_nonsid(uint8_t *fr_bytes)
{
	uint8_t *subf_bytes;
	unsigned subn;

	subf_bytes = fr_bytes + 5;	/* skip signature and LARc bits */
	for (subn = 0; subn < 4; subn++) {
		subf_bytes[2] |= 0x24;
		subf_bytes[3] |= 0x90;
		subf_bytes += 7;
	}
}

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
	memset(out + 1, 0, GSM_FR_BYTES - 1);
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

	/*
	 * Many BTS models will emit the previous content of their internal
	 * buffer, perhaps corrupted in some peculiar way, when they need to
	 * emit "BFI with no data" because they received FACCH, or because
	 * they received nothing at all and no channel decoding attempt was
	 * made.  When this situation occurs, the Rx DTX handler in the TRAU
	 * needs to be told "this is a regular BFI, not an invalid SID",
	 * and the BTS makes this indication by setting C12=1 (BFI),
	 * C13=0 and C14=0 (SID=0).  However, the stale or corrupted frame
	 * payload bit content in the Abis output buffer will still sometimes
	 * indicate SID (usually invalid) by the bit counting rules of
	 * GSM 06.31 section 6.1.1!  This situation creates a problem
	 * in the case of TW-TS-001 output: there are no out-of-band bits
	 * for C13 & C14, and when the remote transcoder or TFO transform
	 * calls osmo_fr_sid_classify() or its libgsmfr2 equivalent,
	 * it will detect an invalid SID (always invalid due to BFI=1)
	 * instead of intended-by-BTS "regular BFI".  Solution: because
	 * there is no need to preserve all payload bits that are known
	 * to be bad or dummy, we can afford to corrupt some of them;
	 * as a workaround for the unintentional-SID problem, we "break"
	 * the SID field.
	 *
	 * Note that BFI=1 is a required condition for the logic below.
	 * Because standard RFC 3551 output does not support BFI,
	 * this logic applies only to TW-TS-001 output.
	 */
	if (tf->c_bits[11] && !tf->c_bits[12] && !tf->c_bits[13] &&
	    osmo_fr_is_any_sid(out))
		make_fr_bfi_nonsid(out);

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

/*
 * This little helper function modifies a marked-bad (BFI=1) GSM-EFR payload
 * so it would no longer classify as SID by the bit counting rules
 * of GSM 06.81 section 6.1.1.  It is needed at times when a BTS emits
 * "BFI with no data", out-of-band SID bits (C13 & C14) are set to 0,
 * but the stale bit pattern in the output buffer used by the BTS
 * happens to be (usually invalid) SID.
 *
 * In order to reliably "break the SID", we need to set 16 bits in the
 * SID field to values opposite of SID codeword, i.e., to 0 for GSM-EFR.
 * The SID field of GSM-EFR includes (in every subframe) some bits in
 * LTP lag and LTP gain fields, and many bits in the fixed codebook
 * excitation pulses portion.  The reference decoder from ETSI makes use
 * of the fixed codebook portion even in marked-bad frames, hence
 * we prefer not to corrupt this portion - but we can still reliably
 * break the SID by clearing some bits in LTP lag and gain fields.
 * Let's clear the two lsbs of each LTP lag and LTP gain field,
 * giving us the needed total of 16 bits.
 */
static void make_efr_bfi_nonsid(uint8_t *efr_bytes)
{
	/* subframe 0 */
	efr_bytes[6] &= 0x99;
	/* subframe 1 */
	efr_bytes[12] &= 0xE6;
	efr_bytes[13] &= 0x7F;
	/* subframe 2 */
	efr_bytes[19] &= 0x33;
	/* subframe 3 */
	efr_bytes[25] &= 0xCC;
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
	memset(out + 1, 0, GSM_EFR_BYTES - 1);
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

	/*
	 * Many BTS models will emit the previous content of their internal
	 * buffer, perhaps corrupted in some peculiar way, when they need to
	 * emit "BFI with no data" because they received FACCH, or because
	 * they received nothing at all and no channel decoding attempt was
	 * made.  When this situation occurs, the Rx DTX handler in the TRAU
	 * needs to be told "this is a regular BFI, not an invalid SID",
	 * and the BTS makes this indication by setting C12=1 (BFI),
	 * C13=0 and C14=0 (SID=0).  However, the stale or corrupted frame
	 * payload bit content in the Abis output buffer will still sometimes
	 * indicate SID (usually invalid) by the bit counting rules of
	 * GSM 06.81 section 6.1.1!  This situation creates a problem
	 * in the case of TW-TS-001 output: there are no out-of-band bits
	 * for C13 & C14, and when the remote transcoder or TFO transform
	 * calls osmo_efr_sid_classify() or its libgsmefr equivalent,
	 * it will detect an invalid SID (always invalid due to BFI=1)
	 * instead of intended-by-BTS "regular BFI".  Solution: because
	 * there is no need to preserve all payload bits that are known
	 * to be bad or dummy, we can afford to corrupt some of them;
	 * as a workaround for the unintentional-SID problem, we "break"
	 * the SID field.
	 *
	 * Note that BFI=1 is a required condition for the logic below.
	 * Because standard RFC 3551 output does not support BFI,
	 * this logic applies only to TW-TS-001 output.
	 */
	if (tf->c_bits[11] && !tf->c_bits[12] && !tf->c_bits[13] &&
	    osmo_efr_is_any_sid(out))
		make_efr_bfi_nonsid(out);

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

	/* with or without TW-TS-001 TEH, we require 260-bit GSM-FR payload */
	if (data_len != GSM_FR_BYTES)
		return -EINVAL;
	if ((data[0] & 0xF0) != 0xD0)
		return -EINVAL;

	/* bad frames are only allowed in UL */
	if (bfi && tf->dir != OSMO_TRAU_DIR_UL)
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
		/* C1 .. C5: FR DL */
		tf->c_bits[0] = 1;
		tf->c_bits[1] = 1;
		tf->c_bits[2] = 1;
		tf->c_bits[3] = 0;
		tf->c_bits[4] = 0;
	}
	memset(&tf->c_bits[5], 0, 6);	/* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		tf->c_bits[11] = bfi;		/* C12: BFI */
		/* SID classification per GSM 06.31 section 6.1.1 */
		sidc = osmo_fr_sid_classify(data);
		tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
		tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
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

static int rtp2trau_hr16_dl(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	/* accept both TS 101 318 and RFC 5993 payloads */
	switch (data_len) {
	case GSM_HR_BYTES:
		break;
	case GSM_HR_BYTES_RTP_RFC5993:
		data++;
		data_len--;
		break;
	default:
		return -EINVAL;
	}

	tf->type = OSMO_TRAU16_FT_HR;

	/* C1 .. C5 */
	tf->c_bits[0] = 1;
	tf->c_bits[1] = 1;
	tf->c_bits[2] = 1;
	tf->c_bits[3] = 0;
	tf->c_bits[4] = 1;
	/* C6.. C11: Time Alignment */
	memset(tf->c_bits + 5, 0, 6);
	tf->c_bits[11] = 1; /* C12: UFE */
	tf->c_bits[12] = 1; /* C13: spare */
	tf->c_bits[13] = 1; /* C14: spare */
	tf->c_bits[14] = 1; /* C15: spare */
	if (osmo_hr_check_sid(data, data_len))
		tf->c_bits[15] = 0; /* C16: SP */
	else
		tf->c_bits[15] = 1; /* C16: SP */
	tf->c_bits[16] = 1; /* C17: spare */
	memset(tf->c_bits+17, 1, 4); /* C18..C21: spare */
	memset(&tf->t_bits[0], 1, 4);
	tf->ufi = 1;	/* spare bit in TRAU-DL */
	osmo_pbit2ubit(tf->d_bits, data, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

	return 0;
}

static int rtp2trau_hr16_ul(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	uint8_t ft;
	enum osmo_gsm631_sid_class sidc;
	bool data_bits_req, bfi;

	/* In TRAU-UL direction we require/expect TW-TS-002 RTP payload format;
	 * RFC 5993 is also accepted because it is a subset of TW-TS-002.
	 * TS 101 318 input is not supported for TRAU-UL output! */
	if (data_len < 1)
		return -EINVAL;
	ft = data[0] >> 4;
	switch (ft) {
	case FT_GOOD_SPEECH:
		bfi = false;
		sidc = OSMO_GSM631_SID_CLASS_SPEECH;
		data_bits_req = true;
		break;
	case FT_INVALID_SID:
		/* There are 3 possible BFI/SID combinations in TRAU-16k-UL
		 * format for GSM-HR that all collapse to the same
		 * "invalid SID" code in TRAU-8k-UL and TW-TS-002 formats.
		 * Here we arbitrarily choose BFI=1 SID=1: compared to the
		 * other two options, this one has the greatest Hamming
		 * distance (in C-bits) to good speech or valid SID codes.
		 */
		bfi = true;
		sidc = OSMO_GSM631_SID_CLASS_INVALID;
		data_bits_req = false;
		break;
	case FT_GOOD_SID:
		bfi = false;
		sidc = OSMO_GSM631_SID_CLASS_VALID;
		data_bits_req = true;
		break;
	case FT_BFI_WITH_DATA:
		bfi = true;
		sidc = OSMO_GSM631_SID_CLASS_SPEECH;
		data_bits_req = true;
		break;
	case FT_NO_DATA:
		bfi = true;
		sidc = OSMO_GSM631_SID_CLASS_SPEECH;
		data_bits_req = false;
		break;
	default:
		return -EINVAL;
	}
	/* If the frame type is one that includes data bits, the payload length
	 * per RFC 5993 and TW-TS-002 is 15 bytes.  If the frame type is one
	 * that does not include data bits, then the payload length per the
	 * same specs is only 1 byte - but we also accept 15-byte payloads
	 * in this case to make life easier for applications that pass the
	 * content of a buffer.
	 *
	 * When we make a TRAU-UL frame from FT=1 or FT=7, we fill all Dn bits
	 * with zeros if we got a short (1 byte) payload.  However, if the
	 * application passed us a long (15 byte) payload despite FT being
	 * 1 or 7, we fill Dn bits with application-provided payload.
	 */
	switch (data_len) {
	case GSM_HR_BYTES_RTP_RFC5993:
		break;
	case 1:
		if (data_bits_req)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	tf->type = OSMO_TRAU16_FT_HR;

	/* C1 .. C5 */
	tf->c_bits[0] = 0;
	tf->c_bits[1] = 0;
	tf->c_bits[2] = 0;
	tf->c_bits[3] = 1;
	tf->c_bits[4] = 1;
	/* C6.. C11: Time Alignment */
	memset(tf->c_bits + 5, 0, 6);
	tf->c_bits[11] = bfi;		  /* C12: BFI */
	tf->c_bits[12] = (sidc >> 1) & 1; /* C13: SID msb */
	tf->c_bits[13] = (sidc >> 0) & 1; /* C14: SID lsb */
	tf->c_bits[14] = data[0] & 0x01;  /* C15: TAF */
	tf->c_bits[15] = 1;		  /* C16: spare */
	tf->c_bits[16] = (data[0] & 0x08) >> 3; /* C17: DTXd */
	memset(tf->c_bits+17, 1, 4); /* C18..C21: spare */
	memset(&tf->t_bits[0], 1, 4);
	tf->ufi = (data[0] & 0x02) >> 1;
	if (data_len > 1)
		osmo_pbit2ubit(tf->d_bits, data + 1, GSM_HR_BYTES * 8);
	else
		memset(tf->d_bits, 0, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

	return 0;
}

static int rtp2trau_hr16(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	switch (tf->dir) {
	case OSMO_TRAU_DIR_DL:
		return rtp2trau_hr16_dl(tf, data, data_len);
	case OSMO_TRAU_DIR_UL:
		return rtp2trau_hr16_ul(tf, data, data_len);
	default:
		return -EINVAL;
	}
}

static int rtp2trau_hr8_dl(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	/* accept both TS 101 318 and RFC 5993 payloads */
	switch (data_len) {
	case GSM_HR_BYTES:
		break;
	case GSM_HR_BYTES_RTP_RFC5993:
		data++;
		data_len--;
		break;
	default:
		return -EINVAL;
	}

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

	osmo_pbit2ubit(tf->d_bits, data, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

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
	return !(sum & 1);
}

static int rtp2trau_hr8_ul(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	uint8_t ft, xc1_4;
	bool data_bits_req, have_taf;

	/* In TRAU-UL direction we require/expect TW-TS-002 RTP payload format;
	 * RFC 5993 is also accepted because it is a subset of TW-TS-002.
	 * TS 101 318 input is not supported for TRAU-UL output! */
	if (data_len < 1)
		return -EINVAL;
	ft = data[0] >> 4;
	switch (ft) {
	case FT_GOOD_SPEECH:
		xc1_4 = 0;
		data_bits_req = true;
		have_taf = false;
		break;
	case FT_INVALID_SID:
		xc1_4 = 4;
		data_bits_req = false;
		have_taf = true;
		break;
	case FT_GOOD_SID:
		xc1_4 = 1;
		data_bits_req = true;
		have_taf = false;
		break;
	case FT_BFI_WITH_DATA:
		xc1_4 = 6;
		data_bits_req = true;
		have_taf = true;
		break;
	case FT_NO_DATA:
		xc1_4 = 6;
		data_bits_req = false;
		have_taf = true;
		break;
	default:
		return -EINVAL;
	}
	/* If the frame type is one that includes data bits, the payload length
	 * per RFC 5993 and TW-TS-002 is 15 bytes.  If the frame type is one
	 * that does not include data bits, then the payload length per the
	 * same specs is only 1 byte - but we also accept 15-byte payloads
	 * in this case to make life easier for applications that pass the
	 * content of a buffer.
	 *
	 * When we make a TRAU-UL frame from FT=1 or FT=7, we fill all Dn bits
	 * with zeros if we got a short (1 byte) payload.  However, if the
	 * application passed us a long (15 byte) payload despite FT being
	 * 1 or 7, we fill Dn bits with application-provided payload.
	 */
	switch (data_len) {
	case GSM_HR_BYTES_RTP_RFC5993:
		break;
	case 1:
		if (data_bits_req)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	tf->type = OSMO_TRAU8_SPEECH;

	/* C1..C5 */
	tf->c_bits[0] = 0;
	tf->c_bits[1] = 0;
	tf->c_bits[2] = 0;
	tf->c_bits[3] = 1;
	tf->c_bits[4] = 0;
	/* C6..C8: spare bits */
	memset(tf->c_bits + 5, 1, 3);
	/* C9 is DTXd */
	tf->c_bits[8] = (data[0] & 0x08) >> 3;

	/* XC1..XC6 */
	tf->xc_bits[0] = (xc1_4 >> 3) & 1;
	tf->xc_bits[1] = (xc1_4 >> 2) & 1;
	tf->xc_bits[2] = (xc1_4 >> 1) & 1;
	if (have_taf)
		tf->xc_bits[3] = (data[0] & 0x01) >> 0;
	else
		tf->xc_bits[3] = (xc1_4 >> 0) & 1;
	tf->xc_bits[4] = (data[0] & 0x02) >> 1;		/* UFI */
	tf->xc_bits[5] = compute_odd_parity(tf->xc_bits, 5);

	memset(&tf->t_bits[0], 1, 2);

	if (data_len > 1)
		osmo_pbit2ubit(tf->d_bits, data + 1, GSM_HR_BYTES * 8);
	else
		memset(tf->d_bits, 0, GSM_HR_BYTES * 8);
	/* CRC is *not* computed by TRAU frame encoder - we have to do it */
	osmo_crc8gen_set_bits(&gsm0860_efr_crc3, tf->d_bits, 44, tf->crc_bits);

	return 0;
}

static int rtp2trau_hr8(struct osmo_trau_frame *tf, const uint8_t *data, size_t data_len)
{
	switch (tf->dir) {
	case OSMO_TRAU_DIR_DL:
		return rtp2trau_hr8_dl(tf, data, data_len);
	case OSMO_TRAU_DIR_UL:
		return rtp2trau_hr8_ul(tf, data, data_len);
	default:
		return -EINVAL;
	}
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

	/* with or without TW-TS-001 TEH, we require 244-bit GSM-EFR payload */
	if (data_len != GSM_EFR_BYTES)
		return -EINVAL;
	if ((data[0] & 0xF0) != 0xC0)
		return -EINVAL;

	/* bad frames are only allowed in UL */
	if (bfi && tf->dir != OSMO_TRAU_DIR_UL)
		return -EINVAL;

	tf->type = OSMO_TRAU16_FT_EFR;

	/* FR Data Bits according to TS 48.060 Section 5.5.1.1.2 */

	/* set c-bits and t-bits */
	/* C1 .. C5: EFR */
	tf->c_bits[0] = 1;
	tf->c_bits[1] = 1;
	tf->c_bits[2] = 0;
	tf->c_bits[3] = 1;
	tf->c_bits[4] = 0;
	memset(&tf->c_bits[5], 0, 6); /* C6 .. C11: Time Alignment */
	if (tf->dir == OSMO_TRAU_DIR_UL) {
		tf->c_bits[11] = bfi;		/* C12: BFI */
		/* SID classification per GSM 06.81 section 6.1.1 */
		sidc = osmo_efr_sid_classify(data);
		tf->c_bits[12] = (sidc >> 1) & 1; /* C13: msb */
		tf->c_bits[13] = (sidc >> 0) & 1; /* C14: lsb */
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

	trau2v110_bits(ra_bits, in);
	/* RA2: 1 bit per output byte */
	osmo_csd_ra2_8k_pack(out, ra_bits, 80);
}

/* intermediate rate 16 kbit/s */
static void trau2v110_ir16(uint8_t *out, const ubit_t *in)
{
	ubit_t ra_bits[80];

	trau2v110_bits(ra_bits, in);
	/* RA2: 2 bits per output byte */
	osmo_csd_ra2_16k_pack(out, ra_bits, 40);
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

static int trau2rtp_edata(uint8_t *out, size_t out_len,
			  const struct osmo_trau_frame *tf)
{
	/* function interface preliminaries */
	if (tf->type != OSMO_TRAU16_FT_EDATA)
		return -EINVAL;
	if (out_len < RFC4040_RTP_PLEN)
		return -ENOSPC;

	/* Per TS 48.020 section 11.1:
	 * A-TRAU bit C4 is always 1 in BSS->IWF direction
	 * A-TRAU bit C5 comes from E-TRAU bit C6 */
	osmo_csd144_to_atrau_ra2(out, tf->m_bits, tf->d_bits, 1, tf->c_bits[5]);

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

	if (data_len != RFC4040_RTP_PLEN)
		goto idle_fill;

	/* reverse RA2 first */
	osmo_csd_ra2_8k_unpack(ra_bits, data, RFC4040_RTP_PLEN);

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

	if (data_len != RFC4040_RTP_PLEN)
		goto idle_fill;

	/* reverse RA2 first */
	osmo_csd_ra2_16k_unpack(ra_bits, data, RFC4040_RTP_PLEN);

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

/*
 * For 14.4 kbit/s data, we do _not_ forward A-TRAU bit C5 to E-TRAU bit C6
 * like TS 48.020 section 11.1 Note 1 suggests.  That spec note says:
 * "Bit C5 corresponds to bit C6 of the E-TRAU frame as defined in 3GPP
 * TS 48.060."  However, while the note says (effectively) that the two
 * bits are logically equivalent, it does not prescribe specific forwarding
 * rules for implementations.  When the BTS looks at bit C6 in TRAU-DL,
 * it will want to know whether its TRAU-UL frames were received correctly
 * by the TRAU, hence the TRAU-emulating Abis MGW will need to set this bit
 * based on UL reception status - but it has nothing to do with the incoming
 * RTP stream.  Hence we don't forward this "logically equivalent" bit.
 */

static int rtp2trau_edata(struct osmo_trau_frame *tf, const uint8_t *data,
			  size_t data_len)
{
	ubit_t atrau_c4;

	tf->type = OSMO_TRAU16_FT_EDATA;

	/* set all C bits to 1s: covers frame type, initial C6 & C7,
	 * and the remaining unused bits. */
	memset(tf->c_bits, 1, 13);
	/* set C6=0: indicate good reception of TRAU-UL frames */
	tf->c_bits[5] = 0;

	if (data_len == RFC4040_RTP_PLEN &&
	    osmo_csd144_from_atrau_ra2(tf->m_bits, tf->d_bits, &atrau_c4, NULL,
					data) >= 0) {
		/* We got good A-TRAU input.  3GPP specs define the following
		 * mapping for DL direction only:
		 *
		 * E-TRAU bit C7 is set if idle, inverse of A-TRAU bit C4.
		 * Spec references: TS 48.020 section 11.1, TS 48.060
		 * section 6.5.1. */
		tf->c_bits[6] = !atrau_c4;
	} else {
		/* We did not get good A-TRAU input: generate idle fill. */
		memset(tf->m_bits, 1, 2);
		memset(tf->d_bits, 1, 288);
		/* C7=1 means idle */
		tf->c_bits[6] = 1;
	}

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

/*! convert received TRAU-UL frame to RTP payload
 *  \param[out] out Buffer for the output RTP payload
 *  \param[in] out_len Size of buffer pointed to by \ref out
 *  \param[in] tf Osmo-decoded TRAU frame
 *  \param[in] st State/config structure
 *  \returns length of converted RTP payload if successful; negative on error
 *
 * This function is intended to operate on TRAU-UL or TFO frames, either
 * received from an E1 BTS or extracted from lsbs of PCM samples in TFO.
 * Supported TRAU frame types are FR & EFR speech, HRv1 speech in both
 * 16k and 8k formats, and all defined CSD frame types up to 14.4 kbit/s
 * extended data frames.
 *
 * In the case of FR/EFR speech, the output format is either RFC 3551 or
 * TW-TS-001; in the case of HRv1 speech, the output format is either RFC 5993
 * or TW-TS-002.  st->rtp_extensions field selects the use or non-use of
 * Themyscira RTP extensions; the structure passed in \ref st currently
 * has no other uses in the TRAU->RTP direction.
 *
 * The following TRAU frame types are _not_ supported:
 *
 * - TRAU-DL frames: the direction field of the parsed frame structure is
 *   ignored and all frames are processed as if they were TRAU-UL.  In the
 *   case of FR, EFR, HR-16k and extended data (E-TRAU) frames some control
 *   bits are different between TRAU-UL and TRAU-DL, such that this
 *   misinterpretation will produce invalid results.
 *
 * - D144 sync frames: these special frames are not convertible to RTP;
 *   their synchronization function needs to be handled by the application.
 *
 * - AMR speech frames: not currently implemented.
 */
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
	case OSMO_TRAU16_FT_EDATA:
		return trau2rtp_edata(out, out_len, tf);
	case OSMO_TRAU8_SPEECH:
		return trau2rtp_hr8(out, out_len, tf, check_twts002(st));
	case OSMO_TRAU8_DATA:
		return trau2rtp_data_hr8(out, out_len, tf);
	default:
		return -EINVAL;
	}
}

/*! convert RTP payload to TRAU-UL or TRAU-DL frame
 *  \param[out] tf Osmocom-defined structure for TRAU frame bits
 *  \param[in] rtp Payload to be converted
 *  \param[in] rtp_len Length of payload in \ref rtp
 *  \param[in] st State/config structure
 *  \returns 0 in case of success; negative on error
 *
 * This function can be used to generate both TRAU-UL and TRAU-DL frames.
 * TRAU-DL output is needed when feeding traffic to an E1 BTS; TRAU-UL output
 * is needed in more specialized applications that emulate an E1 BTS or
 * implement in-band TFO.
 *
 * The set of supported codecs and frame types is the same as osmo_trau2rtp();
 * st->type selects the TRAU frame type to be emitted.  Additionally, if the
 * TRAU frame type to be generated is OSMO_TRAU16_FT_DATA, st->interm_rate_16k
 * needs to be set to true for 16 kbit/s IR or false for 8 kbit/s IR.
 *
 * In the output structure pointed to by \ref tf, the caller MUST set dir
 * member prior to calling the present function; additionally, dl_ta_usec
 * member MUST be set (usually to 0) prior to calling osmo_trau_frame_encode().
 * All other required bits are filled correctly by the present function.
 *
 * With FR/HR/EFR speech codecs, semantically appropriate RTP payloads are
 * different between UL and DL output applications.  Considerations for
 * TRAU-DL output:
 *
 * - RTP payload formats of RFC 5993, TW-TS-001 and TW-TS-002 are accepted
 *   by the function - however, all metadata flags carried by the header octet
 *   of these extended formats are ignored/dropped in the DL direction.
 *   BFI frames are not allowed in DL.
 *
 * - The most native RTP input formats for conversion to TRAU-DL are those
 *   defined in ETSI TS 101 318 for FR, HR and EFR; the ones for FR and EFR
 *   are also duplicated in RFC 3551.  In the case of HR codec, RFC 5993 input
 *   is also appropriate as specified in 3GPP TS 48.103 - as long as the user
 *   remembers that the extra header octet is ignored.
 *
 * - The only correct way to implement TrFO for GSM, accepting FR/HR/EFR from
 *   call leg A uplink in TW-TS-001 or TW-TS-002 format and generating TRAU-DL
 *   frames for call leg B, is to apply the TFO transform of TS 28.062 section
 *   C.3.2.1.1, then feed the output of that transform to the present function.
 *
 * Considerations for TRAU-UL output:
 *
 * - For FR and EFR codecs, the only correct RTP format for conversion to
 *   TRAU-UL (TFO) is TW-TS-001 - the basic RTP format of TS 101 318 or
 *   RFC 3551 lacks the necessary metadata flags.
 *
 * - For HRv1 codec, for the same reason as above, the only correct RTP
 *   format for conversion to TRAU-UL is TW-TS-002.  RFC 5993 payloads are
 *   also accepted (because it is a subset of TW-TS-002), but not TS 101 318.
 *
 * - TRAU-UL output for CSD 14.4 kbit/s mode is not currently implemented
 *   (C-bits are always set according to the rules for TRAU-DL) - but the
 *   primary application for TRAU-UL frame output via libosmotrau is TFO,
 *   which does not include CSD.
 */
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
	case OSMO_TRAU16_FT_EDATA:
		return rtp2trau_edata(tf, rtp, rtp_len);
	case OSMO_TRAU8_SPEECH:
		return rtp2trau_hr8(tf, rtp, rtp_len);
	case OSMO_TRAU8_DATA:
		return rtp2trau_data_hr8(tf, rtp, rtp_len);
	default:
		return -EINVAL;
	}
}

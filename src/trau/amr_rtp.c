/*
 * AMR TRAU interworking facility: encoding and decoding of RTP formats.
 *
 * This code was contributed to Osmocom Cellular Network Infrastructure
 * project by Mother Mychaela N. Falconia of Themyscira Wireless.
 * Mother Mychaela's contributions are NOT subject to copyright:
 * no rights reserved, all rights relinquished.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/trau/amr_trau.h>

/*! \addgroup amr_trau
 *  @{
 */

static void decode_twts006_extras(struct osmo_amrt_if *fr,
				  const uint8_t *rtp_pl, unsigned rtp_pl_len,
				  unsigned num_d_bits)
{
	const uint8_t *ptr = rtp_pl + 2 + ((num_d_bits + 7) / 8);
	const uint8_t *endp = rtp_pl + rtp_pl_len;
	uint8_t datum;

	/* simple flags which don't require extra octets */
	if (rtp_pl[0] & 0x08) {	/* RIF presence flag */
		fr->rif = (rtp_pl[0] & 0x04) >> 2;
		fr->rif_valid = true;
	} else {
		fr->rif_valid = false;
	}
	if ((fr->frame_class == OSMO_AMRT_FC_SPEECH_GOOD) && (rtp_pl[1] & 0x02))
		fr->frame_class = OSMO_AMRT_FC_SPEECH_DEGRADED;

	/* set no further info in case of early return */
	fr->ta_dtxd_tfoe_valid = false;
	fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;

	/* decode extra octets in the order given in TW-TS-006 section 5.3 */
	if ((fr->frame_class == OSMO_AMRT_FC_NO_DATA) && (rtp_pl[1] & 0x02)) {
		if (ptr >= endp)
			return;
		datum = *ptr++;
		fr->cmi = datum & 0x07;
		fr->cmi_valid = true;
		if (datum & 0x80)
			fr->frame_class = OSMO_AMRT_FC_ONSET;
	}
	if (rtp_pl[0] & 0x02) {		/* TA+DTXd+TFOE octet */
		if (ptr >= endp)
			return;
		datum = *ptr++;
		fr->ta   = (datum & 0xFC) >> 2;
		fr->dtxd = (datum & 0x02) >> 1;
		fr->tfoe = (datum & 0x01) >> 0;
		fr->ta_dtxd_tfoe_valid = true;
	}

	/* Config_Prot and TFO parameters come last */
	if (!(rtp_pl[0] & 0x01))
		return;
	if (ptr >= endp)
		return;
	datum = *ptr++;
	fr->config_prot = (datum & 0xE0) >> 5;
	fr->message_no  = (datum & 0x18) >> 3;
	fr->param_info  = (datum & 0x06);	/* no shift! */
	if (fr->config_prot == OSMO_AMRT_CFG_PROT_NO_CON)
		return;		/* can't have params for No_Con */
	if (fr->param_info != OSMO_AMRT_TFOP_PRESENT_VALID)
		return;		/* no params included */
	if ((endp - ptr) < 9) {
		/* Received bogon: cut off before TFO parameters!
		 * To avoid messing up TFO protocol by dropping params
		 * from a message that requires them, we clear out
		 * the config message itself.
		 */
		fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
		return;
	}
	osmo_pbit2ubit(fr->tfo_param_bits, ptr, 71);
}

/*! Decode received AMR RTP payload for TRAU interworking
 *
 * \param[out] fr Intermediate representation of the decoded frame, primary
 * output from this function.
 * \param[in] rtp_pl Received RTP payload.
 * \param[in] rtp_pl_len Length of \ref rtp_pl.
 * \param[in] fmt Selection of which AMR RTP payload format is to be decoded.
 * \returns 0 if the payload was decoded successfully, or negative if this
 * received payload is invalid.
 *
 * This function always fills out struct osmo_amrt_if fully, setting all
 * required fields and all validity flags for conditional fields, even when
 * the received payload is deemed invalid.  Therefore, no pre-initialization
 * is needed from the caller, nor does the caller need to do any post-filling
 * except for special application requirements.
 */
int osmo_amrt_decode_rtp(struct osmo_amrt_if *fr, const uint8_t *rtp_pl,
			 unsigned rtp_pl_len, enum osmo_amrt_rtp_format fmt)
{
	uint8_t ft;
	unsigned num_d_bits, total_bits, d_offset;
	ubit_t q, d_bits[244 + 2];

	/* The minimum valid AMR RTP payload length is 2 octets,
	 * for both BWE and OA formats. */
	if (rtp_pl_len < 2)
		goto out_bogon;

	/* CMR bits are in the same position in both BWE and OA -
	 * let's validate received CMR now. */
	fr->cmr = rtp_pl[0] >> 4;
	if (fr->cmr > AMR_12_2 && fr->cmr != AMR_NO_DATA)
		goto out_bogon;
	fr->cmr_valid = true;

	/* Now consider FT, and while we handle BWE vs OA to do so,
	 * also enforce F=0 requirement and extract Q bit. */
	if (fmt == OSMO_AMRT_RTP_FMT_BWE) {
		if (rtp_pl[0] & 0x08)	/* F bit */
			goto out_bogon;
		ft = ((rtp_pl[0] & 0x07) << 1) | ((rtp_pl[1] & 0x80) >> 7);
		q = (rtp_pl[1] & 0x40) >> 6;
	} else {
		if (rtp_pl[1] & 0x80)	/* F bit */
			goto out_bogon;
		ft = (rtp_pl[1] & 0x78) >> 3;
		q  = (rtp_pl[1] & 0x04) >> 2;
	}
	switch (ft) {
	case AMR_NO_DATA:
		num_d_bits = 0;
		break;
	case AMR_SID:
		num_d_bits = 39;
		break;
	default:
		if (ft > AMR_12_2)
			goto out_bogon;
		num_d_bits = gsm690_bitlength[ft];
		break;
	}

	/* Now that we know how many data bits we need for the FT we got,
	 * we can check whether or not the received payload is long enough.
	 */
	if (fmt == OSMO_AMRT_RTP_FMT_BWE)
		total_bits = num_d_bits + 10;
	else
		total_bits = num_d_bits + 16;
	if (rtp_pl_len * 8 < total_bits)
		goto out_bogon;

	/* Now extract all data bits from the received payload */
	if (ft != AMR_NO_DATA) {
		if (fmt == OSMO_AMRT_RTP_FMT_BWE) {
			osmo_pbit2ubit(d_bits, rtp_pl + 1, num_d_bits + 2);
			d_offset = 2;
		} else {
			osmo_pbit2ubit(d_bits, rtp_pl + 2, num_d_bits);
			d_offset = 0;
		}
	}

	/* Now we can do the actual conversion from RTP paradigm to our
	 * intermediate representation that is closer to TRAU paradigm.
	 */
	switch (ft) {
	case AMR_NO_DATA:
		fr->frame_class = OSMO_AMRT_FC_NO_DATA;
		fr->cmi_valid = false;
		break;
	case AMR_SID:
		if (q) {
			if (d_bits[d_offset + 35])
				fr->frame_class = OSMO_AMRT_FC_SID_UPDATE;
			else
				fr->frame_class = OSMO_AMRT_FC_SID_FIRST;
		} else {
			fr->frame_class = OSMO_AMRT_FC_SID_BAD;
		}
		memcpy(fr->s_bits, d_bits + d_offset, 35);
		/* What idiot decided to flip the order of CMI bits? */
		fr->cmi = (d_bits[d_offset+36] << 0) |
			  (d_bits[d_offset+37] << 1) |
			  (d_bits[d_offset+38] << 2);
		fr->cmi_valid = true;
		break;
	default:
		if (q)
			fr->frame_class = OSMO_AMRT_FC_SPEECH_GOOD;
		else
			fr->frame_class = OSMO_AMRT_FC_SPEECH_BAD;
		fr->cmi = ft;
		fr->cmi_valid = true;
		osmo_amr_d_to_s(fr->s_bits, d_bits + d_offset, num_d_bits,
				fr->cmi);
		break;
	}

	if (fmt == OSMO_AMRT_RTP_FMT_OAX) {
		decode_twts006_extras(fr, rtp_pl, rtp_pl_len, num_d_bits);
	} else {
		/* no extra goodies in RFC 4867 formats */
		fr->rif_valid = false;
		fr->ta_dtxd_tfoe_valid = false;
		fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
	}
	return 0;

out_bogon:
	fr->frame_class = OSMO_AMRT_FC_NO_DATA;
	fr->cmi_valid = false;
	fr->cmr_valid = false;
	fr->rif_valid = false;
	fr->ta_dtxd_tfoe_valid = false;
	fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
	return -EINVAL;
}

/*! Encode AMR frame into RTP payload
 *
 * \param[out] pl_buf Caller-provided buffer for generated RTP payload.
 * \param[in] pl_buf_size Available space in \ref pl_buf.
 * \param[in] fr AMR frame to encode.
 * \param[in] fmt Selection of which AMR RTP payload format is to be emitted.
 * \returns number of bytes of generated payload if successful,
 * or negative on errors.
 */
int osmo_amrt_encode_rtp(uint8_t *pl_buf, unsigned pl_buf_size,
			 const struct osmo_amrt_if *fr,
			 enum osmo_amrt_rtp_format fmt)
{
	unsigned num_d_bits, base_bits, base_bytes, req_bytes;
	uint8_t cmr, ft;
	ubit_t q, d_bits[244 + 10];
	unsigned d_offset;
	uint8_t *outptr = pl_buf;

	/* Begin by figuring out how much space we need */
	switch (fr->frame_class) {
	case OSMO_AMRT_FC_SPEECH_GOOD:
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
	case OSMO_AMRT_FC_SPEECH_BAD:
		num_d_bits = gsm690_bitlength[fr->cmi];
		break;
	case OSMO_AMRT_FC_SID_FIRST:
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		num_d_bits = 39;
		break;
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
		num_d_bits = 0;
		break;
	default:
		return -EINVAL;
	}
	if (fmt == OSMO_AMRT_RTP_FMT_BWE)
		base_bits = num_d_bits + 10;
	else
		base_bits = num_d_bits + 16;
	base_bytes = (base_bits + 7) / 8;
	req_bytes = base_bytes;

	/* additional space requirements for TW-TS-006 output */
	if (fmt == OSMO_AMRT_RTP_FMT_OAX) {
		/* FT15 extension octet */
		switch (fr->frame_class) {
		case OSMO_AMRT_FC_NO_DATA:
			if (fr->cmi_valid)
				req_bytes++;
			break;
		case OSMO_AMRT_FC_ONSET:
			req_bytes++;
			break;
		default:
			break;
		}
		/* TA+DTXd+TFOE octet */
		if (fr->ta_dtxd_tfoe_valid)
			req_bytes++;
		/* Config_Prot and TFO parameters */
		if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON) {
			req_bytes++;
			if (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID)
				req_bytes += 9;
		}
	}

	if (pl_buf_size < req_bytes)
		return -ENOSPC;

	/* now generate our output */
	if (fr->cmr_valid)
		cmr = fr->cmr;
	else
		cmr = AMR_NO_DATA;
	switch (fr->frame_class) {
	case OSMO_AMRT_FC_SPEECH_GOOD:
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
		ft = fr->cmi;
		q = 1;
		break;
	case OSMO_AMRT_FC_SPEECH_BAD:
		ft = fr->cmi;
		q = 0;
		break;
	case OSMO_AMRT_FC_SID_FIRST:
	case OSMO_AMRT_FC_SID_UPDATE:
		ft = AMR_SID;
		q = 1;
		break;
	case OSMO_AMRT_FC_SID_BAD:
		ft = AMR_SID;
		q = 0;
		break;
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
		ft = AMR_NO_DATA;
		q = 1;
		break;
	default:
		OSMO_ASSERT(0);
	}

	if (fmt == OSMO_AMRT_RTP_FMT_BWE) {
		d_bits[0] = (cmr >> 3) & 1;
		d_bits[1] = (cmr >> 2) & 1;
		d_bits[2] = (cmr >> 1) & 1;
		d_bits[3] = (cmr >> 0) & 1;
		d_bits[4] = 0;	/* F bit */
		d_bits[5] = (ft >> 3) & 1;
		d_bits[6] = (ft >> 2) & 1;
		d_bits[7] = (ft >> 1) & 1;
		d_bits[8] = (ft >> 0) & 1;
		d_bits[9] = q;
		d_offset = 10;
	} else {
		*outptr++ = (cmr << 4);
		*outptr++ = (ft << 3) | (q << 2);
		d_offset = 0;
	}

	switch (fr->frame_class) {
	case OSMO_AMRT_FC_SPEECH_GOOD:
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
	case OSMO_AMRT_FC_SPEECH_BAD:
		osmo_amr_s_to_d(d_bits + d_offset, fr->s_bits, num_d_bits,
				fr->cmi);
		break;
	case OSMO_AMRT_FC_SID_FIRST:
		/* What is the correct s-bits filler for SID_FIRST in RTP?
		 * RFC 4867 defers to 3GPP TS 26.101 on all such matters,
		 * and the latter spec says (in section 4.2.3) that these
		 * filler bits shall be 0 - the opposite of all-1s filler
		 * in TRAU frames!  Our approach: we obey this indirect
		 * stipulation of RFC 4867 when configured to emit
		 * IETF-compliant (and late-3GPP-compliant) BWE or OA output;
		 * but if we are configured to emit TW-TS-006 format,
		 * we emit whatever s-bits we got, for better network debug
		 * visibility. */
		if (fmt == OSMO_AMRT_RTP_FMT_OAX)
			memcpy(d_bits + d_offset, fr->s_bits, 35);
		else
			memset(d_bits + d_offset, 0, 35);
		d_bits[d_offset + 35] = 0;	/* STI */
		d_bits[d_offset + 36] = (fr->cmi >> 0) & 1;
		d_bits[d_offset + 37] = (fr->cmi >> 1) & 1;
		d_bits[d_offset + 38] = (fr->cmi >> 2) & 1;
		break;
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		memcpy(d_bits + d_offset, fr->s_bits, 35);
		d_bits[d_offset + 35] = 1;	/* STI */
		d_bits[d_offset + 36] = (fr->cmi >> 0) & 1;
		d_bits[d_offset + 37] = (fr->cmi >> 1) & 1;
		d_bits[d_offset + 38] = (fr->cmi >> 2) & 1;
		break;
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
		break;
	default:
		OSMO_ASSERT(0);
	}
	outptr += osmo_ubit2pbit(outptr, d_bits, num_d_bits + d_offset);

	if (fmt != OSMO_AMRT_RTP_FMT_OAX)
		return outptr - pl_buf;

	/* now add TW-TS-006 extensions as needed */
	if (fr->rif_valid)
		pl_buf[0] |= 0x08 | (fr->rif << 2);

	/* FT15 extension octet */
	switch (fr->frame_class) {
	case OSMO_AMRT_FC_NO_DATA:
		if (fr->cmi_valid) {
			pl_buf[1] |= 0x02;
			*outptr++ = fr->cmi;
		}
		break;
	case OSMO_AMRT_FC_ONSET:
		pl_buf[1] |= 0x02;
		*outptr++ = fr->cmi | 0x80;
		break;
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
		pl_buf[1] |= 0x02;
		break;
	default:
		break;
	}

	/* TA+DTXd+TFOE octet */
	if (fr->ta_dtxd_tfoe_valid) {
		pl_buf[0] |= 0x02;
		*outptr++ = (fr->ta << 2) | (fr->dtxd << 1) | (fr->tfoe << 0);
	}

	/* Config_Prot and TFO parameters come last */
	if (fr->config_prot == OSMO_AMRT_CFG_PROT_NO_CON)
		return outptr - pl_buf;

	pl_buf[0] |= 0x01;
	*outptr++ = (fr->config_prot << 5) | (fr->message_no << 3) |
		    fr->param_info;
	if (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID)
		outptr += osmo_ubit2pbit(outptr, fr->tfo_param_bits, 71);
	return outptr - pl_buf;
}

/*! @} */

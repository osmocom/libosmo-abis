/*
 * AMR TRAU interworking facility: encoding and decoding of TRAU frames.
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
#include <osmocom/core/crc8gen.h>
#include <osmocom/core/utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/trau/amr_trau.h>
#include <osmocom/trau/trau_frame.h>

/*! \addgroup amr_trau
 *  @{
 */

/*
 * AMR TRAU parity (same as EFR and HR)
 *
 * g(x) = x^3 + x^1 + 1
 */
static const struct osmo_crc8gen_code gsm0860_amr_crc3 = {
	.bits = 3,
	.poly = 0x3,
	.init = 0x0,
	.remainder = 0x7,
};

/* Please refer to TS 28.062 Table C.6.1.5-1 for the details of how TFO
 * config parameters are mapped to different TRAU frame types.  The parameters
 * are broken down into groups A, B and C, and that last block C is missing
 * in 16k frames for MR102 and 8k frames for SID.  When block C is missing
 * from the transmitted frame, parameters in that block have to assume
 * fixed values listed in that table.  The following datum gives these fixed
 * parameters, in the form in which we need them for TW-TS-006.
 */
static const ubit_t tfo_blockC_fixed_params[26] = {
	1, 1, 1, 1, 1, 1, 1, 1,		/* SCS_2 */
	0,				/* OM_2 */
	1, 0, 0,			/* MACS_2 */
	0,				/* ATVN_2 */
	1, 1, 1, 1, 1, 1, 1, 1,		/* SCS_3 */
	0,				/* OM_3 */
	1, 0, 0,			/* MACS_3 */
	0,				/* ATVN_3 */
};

/*** TRAU frame decoding direction ***/

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

/* This helper function is called when TRAU frame decoding ran into
 * an unrecoverable error, such that the received frame has to be
 * declared totally bad with no useful information content.  In particular,
 * if CRC1 covering control bits is bad, we cannot trust _any_ info
 * from those control bits!
 */
static void decode_handle_error(struct osmo_amrt_decode_state *state,
				struct osmo_amrt_if *fr)
{
	state->cmi_valid = false;
	state->cmr_valid = false;

	fr->frame_class = OSMO_AMRT_FC_NO_DATA;
	fr->cmi_valid = false;
	fr->cmr_valid = false;
	fr->rif_valid = false;
	fr->ta_dtxd_tfoe_valid = false;
	fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
}

static int decode_16k_no_speech(struct osmo_amrt_decode_state *state,
				struct osmo_amrt_if *fr,
				const struct osmo_trau_frame *tf)
{
	ubit_t crc_collect[25 + 61];
	int crc_stat;

	/* Start with CRC verification: if CRC is bad, there is no point
	 * in decoding anything else. */
	memcpy(crc_collect, tf->c_bits, 25);
	memcpy(crc_collect + 25, tf->d_bits + 31, 61);
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, crc_collect,
					   25 + 61, tf->d_bits + 92);
	if (crc_stat)
		return -EINVAL;

	/* CRC is good - now decode the rest */
	fr->frame_class = get_bits(tf->d_bits, 31, 3);
	switch (fr->frame_class) {
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
		/* no need to save s-bits */
		break;
	case OSMO_AMRT_FC_SID_FIRST:
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		/* For all SID variants, we need to extract s-bits,
		 * to be passed into RTP - even for Sid_First, where
		 * these bits are expected to be all ones. */
		memcpy(fr->s_bits, tf->d_bits + 57, 35);
		break;
	default:
		/* invalid No_Speech_Classification code */
		return -EINVAL;
	}

	/* We are past all potential error cases: finish the work. */
	fr->cmi = get_bits(tf->d_bits, 34, 3);
	fr->cmr = get_bits(tf->d_bits, 37, 3);
	fr->cmi_valid = true;
	fr->cmr_valid = true;

	state->cmi = fr->cmi;
	state->cmr = fr->cmr;
	state->cmi_valid = true;
	state->cmr_valid = true;

	return 0;
}

static int decode_16k_speech_per_mode(struct osmo_amrt_if *fr,
				      const struct osmo_trau_frame *tf,
				      bool *bad_crc_234, bool *bad_mr102_crc4)
{
	ubit_t crc_collect[82];		/* largest in mode 6 */
	int crc_stat;

	/* For each mode we follow the same logic:
	 *
	 * - Check CRC1 first: if it is bad, we cannot trust control bits
	 *   and the frame is totally bad.
	 *
	 * - Accept and extract s-bits from the frame.
	 *
	 * - Check the other 3 subframe CRCs.  If any of them is bad,
	 *   we change frame classification to Speech_Bad, but we don't
	 *   have to toss the entire frame.
	 *
	 * - For MR102 only, if CRC4 is bad, we have to note this detail
	 *   for potential impact on TFO config parameters.
	 */
	switch (fr->cmi) {
	case AMR_4_75:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 44, 16);
		memcpy(crc_collect + 41, tf->d_bits + 61, 2);
		memcpy(crc_collect + 43, tf->d_bits + 64, 9);
		memcpy(crc_collect + 52, tf->d_bits + 88, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 56,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 44, 48);
		memcpy(fr->s_bits + 48, tf->d_bits + 95, 13);
		memcpy(fr->s_bits + 61, tf->d_bits + 111, 21);
		memcpy(fr->s_bits + 82, tf->d_bits + 135, 13);
		/* check CRC2 */
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   tf->d_bits + 95, 2,
						   tf->d_bits + 108);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 111, 2);
		memcpy(crc_collect + 2, tf->d_bits + 128, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 6,
						   tf->d_bits + 132);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   tf->d_bits + 135, 2,
						   tf->d_bits + 148);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_5_15:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 46, 16);
		memcpy(crc_collect + 41, tf->d_bits + 64, 11);
		memcpy(crc_collect + 52, tf->d_bits + 87, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 57,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 46, 46);
		memcpy(fr->s_bits + 46, tf->d_bits + 95, 19);
		memcpy(fr->s_bits + 65, tf->d_bits + 117, 19);
		memcpy(fr->s_bits + 84, tf->d_bits + 139, 19);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 2);
		memcpy(crc_collect + 2, tf->d_bits + 109, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 114);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 117, 2);
		memcpy(crc_collect + 2, tf->d_bits + 131, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 136);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 139, 2);
		memcpy(crc_collect + 2, tf->d_bits + 153, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 158);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_5_90:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 41, 17);
		memcpy(crc_collect + 42, tf->d_bits + 67, 8);
		memcpy(crc_collect + 50, tf->d_bits + 88, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 54,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 41, 51);
		memcpy(fr->s_bits + 51, tf->d_bits + 95, 21);
		memcpy(fr->s_bits + 72, tf->d_bits + 119, 25);
		memcpy(fr->s_bits + 97, tf->d_bits + 147, 21);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 3);
		memcpy(crc_collect + 3, tf->d_bits + 112, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 116);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 119, 8);
		memcpy(crc_collect + 8, tf->d_bits + 140, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 12,
						   tf->d_bits + 144);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 147, 3);
		memcpy(crc_collect + 3, tf->d_bits + 164, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 168);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_6_70:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 37, 17);
		crc_collect[42] = tf->d_bits[56];
		crc_collect[43] = tf->d_bits[60];
		memcpy(crc_collect + 44, tf->d_bits + 63, 8);
		memcpy(crc_collect + 52, tf->d_bits + 85, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 57,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 37, 55);
		memcpy(fr->s_bits + 55, tf->d_bits + 95, 25);
		memcpy(fr->s_bits + 80, tf->d_bits + 123, 29);
		memcpy(fr->s_bits + 109, tf->d_bits + 155, 25);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 113, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 9,
						   tf->d_bits + 120);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 123, 8);
		memcpy(crc_collect + 8, tf->d_bits + 145, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 13,
						   tf->d_bits + 152);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 155, 4);
		memcpy(crc_collect + 4, tf->d_bits + 173, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 9,
						   tf->d_bits + 180);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_7_40:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 34, 20);
		memcpy(crc_collect + 45, tf->d_bits + 55, 3);
		memcpy(crc_collect + 48, tf->d_bits + 60, 6);
		memcpy(crc_collect + 54, tf->d_bits + 85, 2);
		memcpy(crc_collect + 56, tf->d_bits + 88, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 59,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 34, 58);
		memcpy(fr->s_bits + 58, tf->d_bits + 95, 29);
		memcpy(fr->s_bits + 87, tf->d_bits + 127, 32);
		memcpy(fr->s_bits + 119, tf->d_bits + 162, 29);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 3);
		memcpy(crc_collect + 3, tf->d_bits + 117, 2);
		memcpy(crc_collect + 5, tf->d_bits + 120, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 8,
						   tf->d_bits + 124);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 127, 6);
		memcpy(crc_collect + 6, tf->d_bits + 152, 2);
		memcpy(crc_collect + 8, tf->d_bits + 155, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 11,
						   tf->d_bits + 159);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 162, 3);
		memcpy(crc_collect + 3, tf->d_bits + 184, 2);
		memcpy(crc_collect + 5, tf->d_bits + 187, 2);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 191);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_7_95:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 31, 35);
		memcpy(crc_collect + 60, tf->d_bits + 83, 2);
		crc_collect[62] = tf->d_bits[87];
		crc_collect[63] = tf->d_bits[90];
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 64,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 31, 61);
		memcpy(fr->s_bits + 61, tf->d_bits + 95, 32);
		memcpy(fr->s_bits + 93, tf->d_bits + 130, 34);
		memcpy(fr->s_bits + 127, tf->d_bits + 167, 32);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 118, 2);
		memcpy(crc_collect + 6, tf->d_bits + 122, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 10,
						   tf->d_bits + 127);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 130, 8);
		memcpy(crc_collect + 8, tf->d_bits + 155, 2);
		memcpy(crc_collect + 10, tf->d_bits + 159, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 14,
						   tf->d_bits + 164);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 167, 4);
		memcpy(crc_collect + 4, tf->d_bits + 190, 2);
		memcpy(crc_collect + 6, tf->d_bits + 194, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 10,
						   tf->d_bits + 199);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	case AMR_10_2:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits, 45);
		memcpy(crc_collect + 70, tf->d_bits + 46, 8);
		memcpy(crc_collect + 78, tf->d_bits + 85, 2);
		memcpy(crc_collect + 80, tf->d_bits + 88, 2);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 82,
						   tf->d_bits + 92);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 20, 72);
		memcpy(fr->s_bits + 72, tf->d_bits + 95, 43);
		memcpy(fr->s_bits + 115, tf->d_bits + 141, 46);
		memcpy(fr->s_bits + 161, tf->d_bits + 190, 43);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 131, 2);
		memcpy(crc_collect + 6, tf->d_bits + 134, 2);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 8,
						   tf->d_bits + 138);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 141, 8);
		memcpy(crc_collect + 8, tf->d_bits + 180, 2);
		memcpy(crc_collect + 10, tf->d_bits + 183, 2);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 12,
						   tf->d_bits + 187);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 190, 4);
		memcpy(crc_collect + 4, tf->d_bits + 226, 2);
		memcpy(crc_collect + 6, tf->d_bits + 229, 2);
		memcpy(crc_collect + 8, tf->d_bits + 233, 20);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 28,
						   tf->d_bits + 253);
		if (crc_stat) {
			*bad_crc_234 = true;
			*bad_mr102_crc4 = true;
		}
		break;
	case AMR_12_2:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits, 29);
		memcpy(crc_collect + 54, tf->d_bits + 38, 12);
		memcpy(crc_collect + 66, tf->d_bits + 86, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 69,
						   tf->d_bits + 91);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits, 91);
		memcpy(fr->s_bits + 91, tf->d_bits + 94, 50);
		memcpy(fr->s_bits + 141, tf->d_bits + 147, 53);
		memcpy(fr->s_bits + 194, tf->d_bits + 203, 50);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 94, 9);
		memcpy(crc_collect + 9, tf->d_bits + 139, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 12,
						   tf->d_bits + 144);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 147, 12);
		memcpy(crc_collect + 12, tf->d_bits + 195, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 15,
						   tf->d_bits + 200);
		if (crc_stat)
			*bad_crc_234 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 203, 5);
		memcpy(crc_collect + 5, tf->d_bits + 209, 3);
		memcpy(crc_collect + 8, tf->d_bits + 248, 3);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 11,
						   tf->d_bits + 253);
		if (crc_stat)
			*bad_crc_234 = true;
		break;
	default:
		OSMO_ASSERT(0);
	}

	fr->cmi_valid = true;
	return 0;
}

static int decode_16k_speech(struct osmo_amrt_decode_state *state,
			     struct osmo_amrt_if *fr,
			     const struct osmo_trau_frame *tf,
			     bool *bad_crc_234, bool *bad_mr102_crc4)
{
	int rc;

	/* The bulk of decoding and CRC verification depends on RIF */
	if (fr->rif) {
		/* CMR frame - saved CMI is required */
		if (!state->cmi_valid)
			return -EINVAL;
		fr->cmi = state->cmi;
		rc = decode_16k_speech_per_mode(fr, tf, bad_crc_234,
						bad_mr102_crc4);
		if (rc < 0)
			return rc;
		/* output and state change specific to CMR frame */
		fr->cmr = get_bits(tf->c_bits, 22, 3);
		fr->cmr_valid = true;
		state->cmr = fr->cmr;
		state->cmr_valid = true;
		state->cmi_valid = false;
	} else {
		/* CMI frame */
		fr->cmi = get_bits(tf->c_bits, 22, 3);
		rc = decode_16k_speech_per_mode(fr, tf, bad_crc_234,
						bad_mr102_crc4);
		if (rc < 0)
			return rc;
		/* output and state change specific to CMI frame */
		if (state->cmr_valid) {
			fr->cmr = state->cmr;
			fr->cmr_valid = true;
		} else {
			fr->cmr_valid = false;
		}
		state->cmi = fr->cmi;
		state->cmi_valid = true;
		state->cmr_valid = false;
	}

	if (*bad_crc_234)
		fr->frame_class = OSMO_AMRT_FC_SPEECH_BAD;
	else
		fr->frame_class = get_bits(tf->c_bits, 20, 2) << 3;
	return 0;
}

static void decode_16k_tfo_mr102(struct osmo_amrt_if *fr,
				 const struct osmo_trau_frame *tf)
{
	/* CRC has already been checked as part of speech frame decoding;
	 * now we just need to extract the parameters.  However, only a
	 * reduced set of params is transmitted in this mode; for the
	 * missing ones, we fill in fixed values as prescribed in TS 28.062
	 * Table C.6.1.5-1. */
	memcpy(fr->tfo_param_bits, tf->d_bits, 20);
	memset(fr->tfo_param_bits + 20, 0, 5);
	memcpy(fr->tfo_param_bits + 25, tf->d_bits + 233, 20);
	memcpy(fr->tfo_param_bits + 45, tfo_blockC_fixed_params, 26);
	fr->param_info = OSMO_AMRT_TFOP_PRESENT_VALID;
}

static void decode_16k_tfo(struct osmo_amrt_if *fr,
			   const struct osmo_trau_frame *tf,
			   bool bad_mr102_crc4)
{
	int crc_stat;

	fr->config_prot = get_bits(tf->c_bits, 13, 3);
	if (fr->config_prot == OSMO_AMRT_CFG_PROT_NO_CON)
		return;
	fr->message_no = get_bits(tf->c_bits, 16, 2);

	/* Is it a speech frame?  To understand the logic that follows,
	 * please refer to TS 28.062 Table C.6.1.5-1. */
	if (osmo_amrt_fc_is_speech(fr->frame_class)) {
		switch (fr->cmi) {
		case AMR_12_2:
			fr->param_info = OSMO_AMRT_TFOP_ABSENT_NO_ROOM;
			return;
		case AMR_10_2:
			if (bad_mr102_crc4) {
				fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
				return;
			}
			decode_16k_tfo_mr102(fr, tf);
			return;
		default:
			break;
		}
	}

	/* We have to check all 3 TFO-specific CRCs */
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, tf->d_bits, 28,
					   tf->d_bits + 28);
	if (crc_stat) {
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
		return;
	}
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, tf->d_bits + 233,
					   20, tf->d_bits + 253);
	if (crc_stat) {
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
		return;
	}
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, tf->d_bits + 202,
					   28, tf->d_bits + 230);
	if (crc_stat) {
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
		return;
	}

	/* CRCs are good - extract the parameters */
	memcpy(fr->tfo_param_bits, tf->d_bits, 25);
	memcpy(fr->tfo_param_bits + 25, tf->d_bits + 233, 20);
	memcpy(fr->tfo_param_bits + 45, tf->d_bits + 202, 26);
	fr->param_info = OSMO_AMRT_TFOP_PRESENT_VALID;
}

static enum osmo_amrt_decode_result
decode_16k(struct osmo_amrt_decode_state *state, struct osmo_amrt_if *fr,
	   const struct osmo_trau_frame *tf, bool allow_config_prot)
{
	bool bad_crc_234 = false, bad_mr102_crc4 = false;
	int rc;

	/* Unless we have to kill it later because of CRC error, RIF is always
	 * valid and readily extracted. */
	fr->rif = tf->c_bits[11];
	fr->rif_valid = true;

	/* speech vs No_Speech frames require different approach to decoding */
	if (tf->c_bits[20] || tf->c_bits[21])
		rc = decode_16k_speech(state, fr, tf, &bad_crc_234,
					&bad_mr102_crc4);
	else
		rc = decode_16k_no_speech(state, fr, tf);
	if (rc < 0) {
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}

	/* CRC1 has been verified as part of decoding, and we know that all
	 * control bits are valid. */
	fr->ta = get_bits(tf->c_bits, 5, 6);
	fr->dtxd = tf->c_bits[18];
	fr->tfoe = tf->c_bits[19];
	fr->ta_dtxd_tfoe_valid = true;

	if (allow_config_prot)
		decode_16k_tfo(fr, tf, bad_mr102_crc4);
	else
		fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;

	if (bad_crc_234)
		return OSMO_AMRT_FRAME_DEC_BAD;
	if (!tf->c_bits[12])
		return OSMO_AMRT_FRAME_DEC_REMOTE_BAD;
	return OSMO_AMRT_FRAME_DEC_GOOD;
}

static void decode_8k_tfo(struct osmo_amrt_if *fr,
			  const struct osmo_trau_frame *tf, bool is_sid)
{
	int crc_stat;

	fr->config_prot = get_bits(tf->d_bits, 54, 3);
	if (fr->config_prot == OSMO_AMRT_CFG_PROT_NO_CON)
		return;
	fr->message_no = get_bits(tf->d_bits, 57, 2);

	/* CRC_A and CRC_B exist whether we are SID or not */
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, tf->d_bits + 64,
					   28, tf->d_bits + 92);
	if (crc_stat) {
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
		return;
	}
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, tf->d_bits + 95,
					   20, tf->d_bits + 115);
	if (crc_stat) {
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
		return;
	}
	/* CRC_C exists only if our No_Speech type is not SID */
	if (!is_sid) {
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   tf->d_bits + 16, 28,
						   tf->d_bits + 44);
		if (crc_stat) {
			fr->param_info = OSMO_AMRT_TFOP_ABSENT_BAD_CRC;
			return;
		}
	}

	/* CRCs are good - extract the parameters */
	memcpy(fr->tfo_param_bits, tf->d_bits + 64, 25);
	memcpy(fr->tfo_param_bits + 25, tf->d_bits + 95, 20);
	if (is_sid)
		memcpy(fr->tfo_param_bits + 45, tfo_blockC_fixed_params, 26);
	else
		memcpy(fr->tfo_param_bits + 45, tf->d_bits + 16, 26);
	fr->param_info = OSMO_AMRT_TFOP_PRESENT_VALID;
}

static enum osmo_amrt_decode_result
decode_8k_low_no_speech(struct osmo_amrt_decode_state *state,
			struct osmo_amrt_if *fr,
			const struct osmo_trau_frame *tf,
			bool allow_config_prot)
{
	ubit_t crc_collect[56];
	int crc_stat;

	/* The definition of TRAU-AMR-8k frame format in TS 48.061 has this
	 * design defect: 8k No_Speech frames include CRC only if the
	 * No_Speech_Classification is either Sid_Update or Sid_Bad, but not
	 * for Sid_First, No_Data or Onset.  In contrast, all TRAU-AMR-16k
	 * frame types feature CRC protection.  I call this aspect of TS 48.061
	 * (the half-rate spec) a design defect because it would not have cost
	 * anything to enable CRC computation and checking for all No_Speech
	 * frame types, whether the 35-bit SID parameters field carries those
	 * or all-1s filler.  But the spec is what it is, and other existing
	 * implementations, whether in E1 BTS or in TFO-interoperating
	 * transcoders anywhere on the global PSTN, emit Sid_First and No_Data
	 * frames without CRC.  If we were to check CRC upfront for all 8k
	 * No_Speech frames, we would reject those spec-valid frames - hence
	 * we have no choice but to accept them without any CRC protection
	 * for control bits.
	 */

	/* If C1,C2 are anything other than 1,0 we got a bogon -
	 * just like bad CRC, if we had one which we could always check. */
	if (tf->c_bits[0] != 1 || tf->c_bits[1] != 0) {
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}
	/* With C1,C2 good, C3 is RIF */
	fr->rif = tf->c_bits[2];
	fr->rif_valid = true;

	fr->frame_class = get_bits(tf->d_bits, 7, 3);
	switch (fr->frame_class) {
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
		if (allow_config_prot)
			decode_8k_tfo(fr, tf, false);
		else
			fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
		break;
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		/* The only 8k No_Speech frames that have CRC! */
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits, 51);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 56,
						   tf->d_bits + 51);
		if (crc_stat) {
			decode_handle_error(state, fr);
			return OSMO_AMRT_FRAME_DEC_BAD;
		}
		memcpy(fr->s_bits, tf->d_bits + 16, 35);
		if (allow_config_prot)
			decode_8k_tfo(fr, tf, true);
		else
			fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
		break;
	case OSMO_AMRT_FC_SID_FIRST:
		if (allow_config_prot)
			decode_8k_tfo(fr, tf, false);
		else
			fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
		/* TS 48.061 section 5.2.4.2.1 allows SID bits to be replaced
		 * with TFO parameters in the case of Sid_First.  In our
		 * "normal" Sid_First handling (8k frame format and no TFO
		 * parameters, or always in 16k format), we prefer to extract
		 * the dummy 35 bits of what would have been SID parameters
		 * and make them available for debug etc - but if we know
		 * that they have been replaced with TFO parameters,
		 * we fill in dummy all-ones here. */
		if (fr->config_prot == OSMO_AMRT_CFG_PROT_NO_CON)
			memcpy(fr->s_bits, tf->d_bits + 16, 35);
		else
			memset(fr->s_bits, 1, 35);
		break;
	default:
		/* invalid No_Speech_Classification code */
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}

	/* We are past all potential error cases: finish the work. */
	fr->cmi = get_bits(tf->d_bits, 10, 3);
	fr->cmr = get_bits(tf->d_bits, 13, 3);
	fr->cmi_valid = true;
	fr->cmr_valid = true;
	fr->ta = get_bits(tf->d_bits, 0, 6);
	fr->dtxd = tf->d_bits[62];
	fr->tfoe = tf->d_bits[63];
	fr->ta_dtxd_tfoe_valid = true;

	state->cmi = fr->cmi;
	state->cmr = fr->cmr;
	state->cmi_valid = true;
	state->cmr_valid = true;

	if (!tf->d_bits[6])
		return OSMO_AMRT_FRAME_DEC_REMOTE_BAD;
	return OSMO_AMRT_FRAME_DEC_GOOD;
}

static int decode_8k_speech_per_mode(struct osmo_amrt_if *fr,
				     const struct osmo_trau_frame *tf,
				     bool *bad_crc_2, bool *bad_crc_34)
{
	ubit_t crc_collect[37];		/* largest in mode 1 */
	int crc_stat;

	/* For each mode we follow the same logic:
	 *
	 * - Check CRC1 first: if it is bad, we cannot trust control bits
	 *   and the frame is totally bad.
	 *
	 * - Accept and extract s-bits from the frame.
	 *
	 * - Check the other 3 subframe CRCs.  If any of them is bad,
	 *   we change frame classification to Speech_Bad, but we don't
	 *   have to toss the entire frame.
	 */
	switch (fr->cmi) {
	case AMR_4_75:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits + 3, 16);
		memcpy(crc_collect + 21, tf->d_bits + 20, 2);
		memcpy(crc_collect + 23, tf->d_bits + 23, 9);
		memcpy(crc_collect + 32, tf->d_bits + 47, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 36,
						   tf->d_bits + 51);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 3, 48);
		memcpy(fr->s_bits + 48, tf->d_bits + 59, 13);
		memcpy(fr->s_bits + 61, tf->d_bits + 75, 21);
		memcpy(fr->s_bits + 82, tf->d_bits + 99, 13);
		/* check CRC2 */
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   tf->d_bits + 54, 7,
						   tf->d_bits + 72);
		if (crc_stat)
			*bad_crc_2 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 75, 2);
		memcpy(crc_collect + 2, tf->d_bits + 92, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 6,
						   tf->d_bits + 96);
		if (crc_stat)
			*bad_crc_34 = true;
		/* check CRC4 */
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   tf->d_bits + 99, 2,
						   tf->d_bits + 112);
		if (crc_stat)
			*bad_crc_34 = true;
		break;
	case AMR_5_15:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits + 5, 16);
		memcpy(crc_collect + 21, tf->d_bits + 23, 11);
		memcpy(crc_collect + 32, tf->d_bits + 46, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 37,
						   tf->d_bits + 51);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits + 5, 46);
		memcpy(fr->s_bits + 46, tf->d_bits + 59, 19);
		memcpy(fr->s_bits + 65, tf->d_bits + 81, 19);
		memcpy(fr->s_bits + 84, tf->d_bits + 103, 19);
		/* check CRC2 */
		memcpy(crc_collect, tf->d_bits + 54, 7);
		memcpy(crc_collect + 7, tf->d_bits + 73, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 12,
						   tf->d_bits + 78);
		if (crc_stat)
			*bad_crc_2 = true;
		/* check CRC3 */
		memcpy(crc_collect, tf->d_bits + 81, 2);
		memcpy(crc_collect + 2, tf->d_bits + 95, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 100);
		if (crc_stat)
			*bad_crc_34 = true;
		/* check CRC4 */
		memcpy(crc_collect, tf->d_bits + 103, 2);
		memcpy(crc_collect + 2, tf->d_bits + 117, 5);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 7,
						   tf->d_bits + 122);
		if (crc_stat)
			*bad_crc_34 = true;
		break;
	case AMR_5_90:
		/* check CRC1 */
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits, 17);
		memcpy(crc_collect + 22, tf->d_bits + 26, 8);
		memcpy(crc_collect + 30, tf->d_bits + 47, 4);
		crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3,
						   crc_collect, 34,
						   tf->d_bits + 51);
		if (crc_stat)
			return -EINVAL;
		/* copy out s-bits */
		memcpy(fr->s_bits, tf->d_bits, 51);
		memcpy(fr->s_bits + 51, tf->d_bits + 59, 21 + 25 + 21);
		/* no CRC2-4 for this mode */
		break;
	default:
		OSMO_ASSERT(0);
	}

	fr->cmi_valid = true;
	return 0;
}

static enum osmo_amrt_decode_result
decode_8k_low_speech(struct osmo_amrt_decode_state *state,
		     struct osmo_amrt_if *fr, const struct osmo_trau_frame *tf,
		     bool allow_config_prot)
{
	uint8_t c1_3 = get_bits(tf->c_bits, 0, 3);
	bool bad_crc_2 = false, bad_crc_34 = false;
	int rc;

	/* The bulk of decoding and CRC verification depends on RIF -
	 * and for 8k, RIF itself is coded indirectly. */
	if (c1_3 >= 3) {
		/* CMR frame - saved CMI is required */
		if (!state->cmi_valid || state->cmi > AMR_5_90) {
			decode_handle_error(state, fr);
			return OSMO_AMRT_FRAME_DEC_BAD;
		}
		fr->cmi = state->cmi;
		rc = decode_8k_speech_per_mode(fr, tf, &bad_crc_2, &bad_crc_34);
		if (rc < 0) {
			decode_handle_error(state, fr);
			return OSMO_AMRT_FRAME_DEC_BAD;
		}
		/* output and state change specific to CMR frame */
		fr->rif = 1;
		fr->rif_valid = true;
		fr->cmr = c1_3 - 3;
		fr->cmr_valid = true;
		state->cmr = fr->cmr;
		state->cmr_valid = true;
		state->cmi_valid = false;
	} else {
		/* CMI frame */
		fr->cmi = c1_3;
		rc = decode_8k_speech_per_mode(fr, tf, &bad_crc_2, &bad_crc_34);
		if (rc < 0) {
			decode_handle_error(state, fr);
			return OSMO_AMRT_FRAME_DEC_BAD;
		}
		/* output and state change specific to CMI frame */
		fr->rif = 0;
		fr->rif_valid = true;
		if (state->cmr_valid) {
			fr->cmr = state->cmr;
			fr->cmr_valid = true;
		} else {
			fr->cmr_valid = false;
		}
		state->cmi = fr->cmi;
		state->cmi_valid = true;
		state->cmr_valid = false;
	}

	if (bad_crc_2 || bad_crc_34)
		fr->frame_class = OSMO_AMRT_FC_SPEECH_BAD;
	else
		fr->frame_class = get_bits(tf->c_bits, 3, 2) << 3;

	/* extra info for TFO etc */
	fr->ta_dtxd_tfoe_valid = false;
	if (allow_config_prot && !bad_crc_2) {
		fr->config_prot = get_bits(tf->d_bits, 54, 3);
		fr->message_no = get_bits(tf->d_bits, 57, 2);
		fr->param_info = OSMO_AMRT_TFOP_ABSENT_NO_ROOM;
	} else {
		fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;
	}

	if (bad_crc_2 || bad_crc_34)
		return OSMO_AMRT_FRAME_DEC_BAD;
	return OSMO_AMRT_FRAME_DEC_GOOD;
}

static enum osmo_amrt_decode_result
decode_8k_low(struct osmo_amrt_decode_state *state, struct osmo_amrt_if *fr,
		const struct osmo_trau_frame *tf, bool allow_config_prot)
{
	if (tf->c_bits[3] || tf->c_bits[4])
		return decode_8k_low_speech(state, fr, tf, allow_config_prot);
	return decode_8k_low_no_speech(state, fr, tf, allow_config_prot);
}

static void decode_8k_67_74_common(struct osmo_amrt_decode_state *state,
				   struct osmo_amrt_if *fr,
				   const struct osmo_trau_frame *tf,
				   unsigned d_offset, bool *bad_cmr)
{
	uint8_t c1_3 = get_bits(tf->c_bits, 0, 3);
	uint8_t d1_3 = get_bits(tf->d_bits, d_offset, 3);

	/* first figure out speech class */
	switch (c1_3) {
	case 0:
	case 2:
		fr->frame_class = OSMO_AMRT_FC_SPEECH_BAD;
		break;
	default:
		fr->frame_class = OSMO_AMRT_FC_SPEECH_GOOD;
		break;
	}

	/* now figure out RIF and CMI/CMR situation */
	switch (c1_3) {
	case 0:
	case 1:
		/* codes for RIF=0 */
		fr->rif = 0;
		fr->rif_valid = true;
		if (state->cmr_valid) {
			fr->cmr = state->cmr;
			fr->cmr_valid = true;
		} else {
			fr->cmr_valid = false;
		}
		state->cmi = fr->cmi;
		state->cmi_valid = true;
		state->cmr_valid = false;
		break;
	case 2:
		/* RIF=1, CMR in Speech_Bad data bits */
		fr->rif = 1;
		fr->rif_valid = true;
		if (d1_3 >= 3) {
			fr->cmr = d1_3 - 3;
			fr->cmr_valid = true;
			state->cmr = fr->cmr;
			state->cmr_valid = true;
		} else {
			*bad_cmr = true;
			fr->cmr_valid = false;
			state->cmr_valid = false;
		}
		state->cmi_valid = false;
		break;
	default:
		/* RIF=1, CMR given like in low speech frames */
		fr->rif = 1;
		fr->rif_valid = true;
		fr->cmr = c1_3 - 3;
		fr->cmr_valid = true;
		state->cmr = fr->cmr;
		state->cmr_valid = true;
		state->cmi_valid = false;
		break;
	}
}

static enum osmo_amrt_decode_result
decode_8k_6k7(struct osmo_amrt_decode_state *state, struct osmo_amrt_if *fr,
		const struct osmo_trau_frame *tf)
{
	ubit_t crc_collect[3 + 17 + 2 + 8 + 5];
	int crc_stat;
	bool bad_cmr = false;

	/* As usual, check CRC before decoding anything else */
	memcpy(crc_collect, tf->c_bits, 3);
	memcpy(crc_collect + 3, tf->d_bits, 17);
	crc_collect[20] = tf->d_bits[19];
	crc_collect[21] = tf->d_bits[23];
	memcpy(crc_collect + 22, tf->d_bits + 26, 8);
	memcpy(crc_collect + 30, tf->d_bits + 48, 5);
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, crc_collect, 35,
					   tf->d_bits + 55);
	if (crc_stat) {
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}

	/* Extract info content from this frame,
	 * without considering C1..C3 yet. */
	fr->cmi = AMR_6_70;
	fr->cmi_valid = true;
	memcpy(fr->s_bits, tf->d_bits, 55);
	memcpy(fr->s_bits + 55, tf->d_bits + 58, 79);

	/* decode speech class and RIF/CMI/CMR */
	decode_8k_67_74_common(state, fr, tf, 0, &bad_cmr);

	/* no extra info */
	fr->ta_dtxd_tfoe_valid = false;
	fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;

	if (bad_cmr)
		return OSMO_AMRT_FRAME_DEC_BAD;
	return OSMO_AMRT_FRAME_DEC_GOOD;
}

static enum osmo_amrt_decode_result
decode_8k_7k4(struct osmo_amrt_decode_state *state, struct osmo_amrt_if *fr,
		const struct osmo_trau_frame *tf)
{
	ubit_t crc_collect[3 + 20 + 3 + 6 + 2 + 3];
	int crc_stat;
	bool bad_cmr = false;

	/* As usual, check CRC before decoding anything else */
	memcpy(crc_collect, tf->c_bits, 3);
	memcpy(crc_collect + 3, tf->d_bits, 20);
	memcpy(crc_collect + 23, tf->d_bits + 21, 3);
	memcpy(crc_collect + 26, tf->d_bits + 26, 6);
	memcpy(crc_collect + 32, tf->d_bits + 51, 2);
	memcpy(crc_collect + 34, tf->d_bits + 54, 3);
	crc_stat = osmo_crc8gen_check_bits(&gsm0860_amr_crc3, crc_collect, 37,
					   tf->d_bits + 58);
	if (crc_stat) {
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}

	/* Extract info content from this frame,
	 * without considering C1..C3 yet. */
	fr->cmi = AMR_7_40;
	fr->cmi_valid = true;
	memcpy(fr->s_bits, tf->d_bits, 58);
	memcpy(fr->s_bits + 58, tf->d_bits + 61, 90);

	/* decode speech class and RIF/CMI/CMR */
	decode_8k_67_74_common(state, fr, tf, 5, &bad_cmr);

	/* no extra info */
	fr->ta_dtxd_tfoe_valid = false;
	fr->config_prot = OSMO_AMRT_CFG_PROT_NO_CON;

	if (bad_cmr)
		return OSMO_AMRT_FRAME_DEC_BAD;
	return OSMO_AMRT_FRAME_DEC_GOOD;
}

/*! High-level decode a received AMR TRAU frame
 *
 * \param[inout] state Decoding state maintained from one frame to the next.
 * \param[out] fr Intermediate representation of the decoded frame, primary
 * output from this function.
 * \param[in] tf Received TRAU frame after low-level decoding by
 * osmo_trau_frame_decode_{8,16}k() functions.
 * \param[in] allow_config_prot Should we accept Rel4 style of TFO config
 * protocol as part of the decoded frame?
 * \returns code indicating status of the decoded frame in the sense of UFE/DFE.
 *
 * This function always fills out struct osmo_amrt_if fully, setting all
 * required fields and all validity flags for conditional fields, even when
 * we return OSMO_AMRT_FRAME_DEC_BAD.  Therefore, no pre-initialization
 * is needed from the caller, nor does the caller need to do any post-filling
 * except for special application requirements.
 *
 * Additional notes regarding \ref allow_config_prot flag: at first glance,
 * it makes the most sense for this function to always decode whatever we
 * received in TRAU frame C-bits and D-bits, put the distilled result into
 * struct osmo_amrt_if, and let the application apply logic that suppresses
 * Config_Prot if necessary.  However, there exist some E1 BTS that implement
 * only Rel5 version of TFO config protocol, not Rel4, and to make it worse
 * (I am pointing at you, Nokia), they put total garbage into bit positions
 * where Rel4 TFO config protocol would go.  Therefore, if an E1 Abis MGW
 * is configured to speak Rel5 version of TFO config protocol, or none at all,
 * it makes sense to skip CPU cycles that would be spent decoding garbage
 * in Rel4 TFO config bits.
 */
enum osmo_amrt_decode_result
osmo_amrt_decode_trau_frame(struct osmo_amrt_decode_state *state,
			    struct osmo_amrt_if *fr,
			    const struct osmo_trau_frame *tf,
			    bool allow_config_prot)
{
	switch (tf->type) {
	case OSMO_TRAU16_FT_AMR:
		return decode_16k(state, fr, tf, allow_config_prot);
	case OSMO_TRAU8_AMR_LOW:
		return decode_8k_low(state, fr, tf, allow_config_prot);
	case OSMO_TRAU8_AMR_6k7:
		return decode_8k_6k7(state, fr, tf);
	case OSMO_TRAU8_AMR_7k4:
		return decode_8k_7k4(state, fr, tf);
	default:
		decode_handle_error(state, fr);
		return OSMO_AMRT_FRAME_DEC_BAD;
	}
}

/*** TRAU frame encoding direction ***/

static void set_bits(ubit_t *bitbuf, int offset, int num, uint32_t val)
{
	uint32_t mask;

	for (mask = 1 << (num - 1); mask; mask >>= 1) {
		if (val & mask)
			bitbuf[offset] = 1;
		else
			bitbuf[offset] = 0;
		offset++;
	}
}

/* This preprocessing function is common between 16k and 8k TRAU frame
 * encoding functions, updating state according to inputs and handling
 * some basic cases of speech frame stealing.
 */
static enum osmo_amrt_fc
encode_common_preproc(struct osmo_amrt_encode_state *state,
			const struct osmo_amrt_if *fr,
			enum osmo_amr_type max_mode_for_channel)
{
	enum osmo_amrt_fc fc = fr->frame_class;

	/* CMR logic per TW-TS-006 section 6.3.4 */
	if (fr->cmr_valid) {
		if (fr->cmr <= max_mode_for_channel)
			state->cmr = fr->cmr;
		else
			state->cmr = max_mode_for_channel;
	}

	/* RIF logic per TW-TS-006 section 6.3.3 */
	if (fr->rif_valid) {
		if (fr->rif && osmo_amrt_fc_is_speech(fc) &&
		    (state->rif || fr->cmi != state->cmi))
			fc = OSMO_AMRT_FC_NO_DATA;
		state->rif = fr->rif;
	} else {
		if (osmo_amrt_fc_is_speech(fc) && !state->rif &&
		    fr->cmi != state->cmi && fr->cmi <= max_mode_for_channel)
			state->rif = 0;		/* two CMI outputs in a row */
		else
			state->rif = !state->rif;
	}

	/* Old state CMI remains only if we got OSMO_AMRT_FC_NO_DATA w/o CMI,
	 * or if we got a bogon in the sense of someone trying to send
	 * MR795, MR102 or MR122 to an 8 kbit/s TRAU channel.
	 */
	if (fr->cmi_valid) {
		if (fr->cmi <= max_mode_for_channel)
			state->cmi = fr->cmi;
		else
			fc = OSMO_AMRT_FC_NO_DATA;
	}

	/* DTXd and TFOE output per TW-TS-006 section 6.3.6 */
	if (fr->ta_dtxd_tfoe_valid) {
		state->dtxd = fr->dtxd;
		state->tfoe = fr->tfoe;
	}

	/* no more commonality between 16k and 8k */
	return fc;
}

static bool encode_16k_steal_for_tfo(const struct osmo_amrt_if *fr)
{
	static const ubit_t five_zeros[5] = {0, 0, 0, 0, 0};
	int rc;

	if (fr->param_info != OSMO_AMRT_TFOP_PRESENT_VALID)
		return false;
	switch (fr->cmi) {
	case AMR_12_2:
		return true;
	case AMR_10_2:
		rc = memcmp(fr->tfo_param_bits + 20, five_zeros, 5);
		if (rc != 0)
			return true;
		rc = memcmp(fr->tfo_param_bits + 45, tfo_blockC_fixed_params,
			    26);
		if (rc != 0)
			return true;
		return false;
	default:
		return false;
	}
}

static void encode_16k_set_cbits(struct osmo_trau_frame *tf,
				 enum osmo_amrt_fc fc,
				 const struct osmo_amrt_if *fr,
				 const struct osmo_amrt_encode_state *state,
				 bool is_tfo)
{
	uint8_t ta;

	set_bits(tf->c_bits, 0, 5, TRAU_FT_AMR);

	if (fr->ta_dtxd_tfoe_valid && fr->ta >= 0x38 && fr->ta <= 0x3C)
		ta = fr->ta;
	else
		ta = is_tfo ? 0x38 : 0;
	set_bits(tf->c_bits, 5, 6, ta);

	tf->c_bits[11] = state->rif;
	tf->c_bits[12] = 1;	/* UFE */

	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON) {
		set_bits(tf->c_bits, 13, 3, fr->config_prot);
		set_bits(tf->c_bits, 16, 2, fr->message_no);
	} else {
		memset(tf->c_bits + 13, 0, 5);
	}

	tf->c_bits[18] = state->dtxd;
	tf->c_bits[19] = state->tfoe;

	tf->c_bits[20] = (fc & 0x10) >> 4;
	tf->c_bits[21] = (fc & 0x08) >> 3;
	if (osmo_amrt_fc_is_speech(fc)) {
		/* speech frame */
		set_bits(tf->c_bits, 22, 3,
			 state->rif ? state->cmr : state->cmi);
	} else {
		/* No_Speech frame */
		memset(tf->c_bits + 22, 0, 3);
	}
}

static void encode_16k_no_speech(struct osmo_trau_frame *tf,
				 enum osmo_amrt_fc fc,
				 const struct osmo_amrt_if *fr,
				 const struct osmo_amrt_encode_state *state)
{
	memset(tf->d_bits, 1, 31);

	set_bits(tf->d_bits, 31, 3, fc);
	set_bits(tf->d_bits, 34, 3, state->cmi);
	set_bits(tf->d_bits, 37, 3, state->cmr);
	/* No PAB or TAE in our pristine output; the application may fill
	 * those in after our encoding but before computing CRC. */
	memset(tf->d_bits + 40, 0, 3);
	/* unused "reserved for TFO" bits */
	memset(tf->d_bits + 43, 1, 14);

	/* D58..D92 depend on No_Speech_Classification */
	switch (fc) {
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		memcpy(tf->d_bits + 57, fr->s_bits, 35);
		break;
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
	case OSMO_AMRT_FC_SID_FIRST:
		memset(tf->d_bits + 57, 1, 35);
		break;
	default:
		OSMO_ASSERT(0);
	}

	memset(tf->d_bits + 95, 1, 256 - 95);
}

static void encode_16k_speech(struct osmo_trau_frame *tf,
				const struct osmo_amrt_if *fr)
{
	switch (fr->cmi) {
	case AMR_4_75:
		memset(tf->d_bits, 1, 44);
		memcpy(tf->d_bits + 44, fr->s_bits, 48);
		memcpy(tf->d_bits + 95, fr->s_bits + 48, 13);
		memcpy(tf->d_bits + 111, fr->s_bits + 61, 21);
		memcpy(tf->d_bits + 135, fr->s_bits + 82, 13);
		memset(tf->d_bits + 151, 1, 256 - 151);
		break;
	case AMR_5_15:
		memset(tf->d_bits, 1, 46);
		memcpy(tf->d_bits + 46, fr->s_bits, 46);
		memcpy(tf->d_bits + 95, fr->s_bits + 46, 19);
		memcpy(tf->d_bits + 117, fr->s_bits + 65, 19);
		memcpy(tf->d_bits + 139, fr->s_bits + 84, 19);
		memset(tf->d_bits + 161, 1, 256 - 161);
		break;
	case AMR_5_90:
		memset(tf->d_bits, 1, 41);
		memcpy(tf->d_bits + 41, fr->s_bits, 51);
		memcpy(tf->d_bits + 95, fr->s_bits + 51, 21);
		memcpy(tf->d_bits + 119, fr->s_bits + 72, 25);
		memcpy(tf->d_bits + 147, fr->s_bits + 97, 21);
		memset(tf->d_bits + 171, 1, 256 - 171);
		break;
	case AMR_6_70:
		memset(tf->d_bits, 1, 37);
		memcpy(tf->d_bits + 37, fr->s_bits, 55);
		memcpy(tf->d_bits + 95, fr->s_bits + 55, 25);
		memcpy(tf->d_bits + 123, fr->s_bits + 80, 29);
		memcpy(tf->d_bits + 155, fr->s_bits + 109, 25);
		memset(tf->d_bits + 183, 1, 256 - 183);
		break;
	case AMR_7_40:
		memset(tf->d_bits, 1, 34);
		memcpy(tf->d_bits + 34, fr->s_bits, 58);
		memcpy(tf->d_bits + 95, fr->s_bits + 58, 29);
		memcpy(tf->d_bits + 127, fr->s_bits + 87, 32);
		memcpy(tf->d_bits + 162, fr->s_bits + 119, 29);
		memset(tf->d_bits + 194, 1, 256 - 194);
		break;
	case AMR_7_95:
		memset(tf->d_bits, 1, 31);
		memcpy(tf->d_bits + 31, fr->s_bits, 61);
		memcpy(tf->d_bits + 95, fr->s_bits + 61, 32);
		memcpy(tf->d_bits + 130, fr->s_bits + 93, 34);
		memcpy(tf->d_bits + 167, fr->s_bits + 127, 32);
		memset(tf->d_bits + 202, 1, 256 - 202);
		break;
	case AMR_10_2:
		memset(tf->d_bits, 1, 20);
		memcpy(tf->d_bits + 20, fr->s_bits, 72);
		memcpy(tf->d_bits + 95, fr->s_bits + 72, 43);
		memcpy(tf->d_bits + 141, fr->s_bits + 115, 46);
		memcpy(tf->d_bits + 190, fr->s_bits + 161, 43);
		memset(tf->d_bits + 233, 1, 20);
		break;
	case AMR_12_2:
		memcpy(tf->d_bits, fr->s_bits, 91);
		memcpy(tf->d_bits + 94, fr->s_bits + 91, 50);
		memcpy(tf->d_bits + 147, fr->s_bits + 141, 53);
		memcpy(tf->d_bits + 203, fr->s_bits + 194, 50);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void encode_16k_tfo_regular(struct osmo_trau_frame *tf,
				   const struct osmo_amrt_if *fr)
{
	if (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID) {
		memcpy(tf->d_bits, fr->tfo_param_bits, 25);
		memset(tf->d_bits + 25, 0, 3);
		memcpy(tf->d_bits + 233, fr->tfo_param_bits + 25, 20);
		memcpy(tf->d_bits + 202, fr->tfo_param_bits + 45, 26);
		memset(tf->d_bits + 228, 0, 2);
	} else {
		/* Par_Type=0,0; the rest is don't-care fill */
		memset(tf->d_bits, 0, 28);
		memset(tf->d_bits + 233, 0, 20);
		memset(tf->d_bits + 202, 0, 28);
	}
	/* all 3 CRCs need to be set correctly no matter how we set the bits */
	osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits, 28,
				tf->d_bits + 28);
	osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 233, 20,
				tf->d_bits + 253);
	osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 202, 28,
				tf->d_bits + 230);
}

static void encode_16k_tfo_mr102(struct osmo_trau_frame *tf,
				 const struct osmo_amrt_if *fr)
{
	if (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID) {
		memcpy(tf->d_bits, fr->tfo_param_bits, 20);
		memcpy(tf->d_bits + 233, fr->tfo_param_bits + 25, 20);
	} else {
		/* Par_Type=0,0; the rest is don't-care fill */
		memset(tf->d_bits, 0, 20);
		memset(tf->d_bits + 233, 0, 20);
	}
}

static void encode_16k_tfo(struct osmo_trau_frame *tf,
			   const struct osmo_amrt_if *fr, enum osmo_amrt_fc fc)
{
	if (!osmo_amrt_fc_is_speech(fc)) {
		encode_16k_tfo_regular(tf, fr);
		return;
	}
	switch (fr->cmi) {
	case AMR_12_2:
		break;
	case AMR_10_2:
		encode_16k_tfo_mr102(tf, fr);
		break;
	default:
		encode_16k_tfo_regular(tf, fr);
		break;
	}
}

/*! High-level encode an AMR TRAU frame into TRAU-AMR-16k format
 *
 * \param[inout] state Encoding state maintained from one frame to the next.
 * \param[out] tf TRAU frame to be generated.
 * \param[in] fr Frame content to be encoded.
 * \param[in] is_tfo TFO frame is to be emitted, rather than regular TRAU.
 * \returns code indicating the type of TRAU frame CRC to be computed later.
 *
 * All necessary fields of struct osmo_trau_frame are filled out by this
 * function (followed by osmo_amrt_set_trau_crc()) with the exception of
 * .dir and .dl_ta_usec - those two fields must be set by the application,
 * either before or after calling the present function, but before passing
 * the frame to osmo_trau_frame_encode() or osmo_trau_frame_encode_tfo()
 * for final low-level encoding.
 */
enum osmo_amrt_crc_type
osmo_amrt_encode_trau_frame_16k(struct osmo_amrt_encode_state *state,
				struct osmo_trau_frame *tf,
				const struct osmo_amrt_if *fr, bool is_tfo)
{
	enum osmo_amrt_fc fc;
	enum osmo_amrt_crc_type crc_type;

	fc = encode_common_preproc(state, fr, AMR_12_2);

	/* Additional case of speech frame stealing for TFO params */
	if (osmo_amrt_fc_is_speech(fc) && fr->config_prot) {
		if (encode_16k_steal_for_tfo(fr))
			fc = OSMO_AMRT_FC_NO_DATA;
	}

	/* Proceed with output */
	tf->type = OSMO_TRAU16_FT_AMR;
	encode_16k_set_cbits(tf, fc, fr, state, is_tfo);
	if (osmo_amrt_fc_is_speech(fc)) {
		encode_16k_speech(tf, fr);
		crc_type = OSMO_AMRT_CRC_16k_SPEECH_MR475 + fr->cmi;
	} else {
		encode_16k_no_speech(tf, fc, fr, state);
		crc_type = OSMO_AMRT_CRC_16k_NO_SPEECH;
	}
	memset(tf->t_bits, 1, 4);

	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON)
		encode_16k_tfo(tf, fr, fc);

	return crc_type;
}

static bool encode_8k_steal_for_tfo(const struct osmo_amrt_if *fr,
				    enum osmo_amrt_fc fc)
{
	int rc;

	switch (fc) {
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
	case OSMO_AMRT_FC_SID_FIRST:
		return false;
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		if (fr->param_info != OSMO_AMRT_TFOP_PRESENT_VALID)
			return false;
		rc = memcmp(fr->tfo_param_bits + 45, tfo_blockC_fixed_params,
			    26);
		return (rc != 0);
	case OSMO_AMRT_FC_SPEECH_GOOD:
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
	case OSMO_AMRT_FC_SPEECH_BAD:
		switch (fr->cmi) {
		case AMR_4_75:
		case AMR_5_15:
		case AMR_5_90:
			return (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID);
		case AMR_6_70:
		case AMR_7_40:
			return true;
		default:
			OSMO_ASSERT(0);
		}
	default:
		OSMO_ASSERT(0);
	}
}

static enum osmo_amrt_crc_type
encode_8k_no_speech(struct osmo_trau_frame *tf, enum osmo_amrt_fc fc,
		    const struct osmo_amrt_if *fr,
		    const struct osmo_amrt_encode_state *state, bool is_tfo)
{
	uint8_t ta;
	enum osmo_amrt_crc_type crc_type;

	if (fr->ta_dtxd_tfoe_valid && fr->ta >= 0x38 && fr->ta <= 0x3C)
		ta = fr->ta;
	else
		ta = is_tfo ? 0x38 : 0;

	tf->type = OSMO_TRAU8_AMR_LOW;
	tf->c_bits[0] = 1;
	tf->c_bits[1] = 0;
	tf->c_bits[2] = state->rif;
	tf->c_bits[3] = 0;
	tf->c_bits[4] = 0;

	set_bits(tf->d_bits, 0, 6, ta);
	tf->d_bits[6] = 1;	/* UFE/DFE */
	set_bits(tf->d_bits, 7, 3, fc);
	set_bits(tf->d_bits, 10, 3, state->cmi);
	set_bits(tf->d_bits, 13, 3, state->cmr);

	/* D17..D51 depend on No_Speech_Classification */
	switch (fc) {
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		crc_type = OSMO_AMRT_CRC_8k_SID_UPDATE;
		memcpy(tf->d_bits + 16, fr->s_bits, 35);
		break;
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
	case OSMO_AMRT_FC_SID_FIRST:
		/* Per TS 48.061 section 5.2.4.2.1, these frame types
		 * (unlike their TRAU-AMR-16k counterparts) have no CRC.
		 * Not only the 35 bits of SID parameters, but also the
		 * following 3 bits for CRC, "shall be set to all '1' in
		 * these cases".  Because it is "shall" and not "should",
		 * we comply and resist the natural urge to fill in CRC
		 * for all No_Speech frame types.
		 */
		crc_type = OSMO_AMRT_CRC_NONE;
		memset(tf->d_bits + 16, 1, 38);
		break;
	default:
		OSMO_ASSERT(0);
	}

	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON) {
		set_bits(tf->d_bits, 54, 3, fr->config_prot);
		set_bits(tf->d_bits, 57, 2, fr->message_no);
	} else {
		memset(tf->d_bits + 54, 0, 5);
	}
	/* No PAB or TAE in our pristine output; the application may fill
	 * those in after our encoding but before computing CRC. */
	memset(tf->d_bits + 59, 0, 3);
	tf->d_bits[62] = state->dtxd;
	tf->d_bits[63] = state->tfoe;

	memset(tf->d_bits + 64, 1, 126 - 64);
	tf->t_bits[0] = 1;

	return crc_type;
}

static void encode_8k_sp_common_low(struct osmo_trau_frame *tf,
				    enum osmo_amrt_fc fc,
				    const struct osmo_amrt_if *fr,
				    const struct osmo_amrt_encode_state *state)
{
	uint8_t c1_3;

	tf->type = OSMO_TRAU8_AMR_LOW;

	if (state->rif)
		c1_3 = state->cmr + 3;
	else
		c1_3 = state->cmi;
	set_bits(tf->c_bits, 0, 3, c1_3);
	tf->c_bits[3] = (fc & 0x10) >> 4;
	tf->c_bits[4] = (fc & 0x08) >> 3;

	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON) {
		set_bits(tf->d_bits, 54, 3, fr->config_prot);
		set_bits(tf->d_bits, 57, 2, fr->message_no);
	} else {
		memset(tf->d_bits + 54, 0, 5);
	}

	tf->t_bits[0] = 1;
}

static void encode_8k_sp_common_high(struct osmo_trau_frame *tf,
				     enum osmo_amrt_fc fc,
				     const struct osmo_amrt_encode_state *state,
				     unsigned d_offset)
{
	bool is_bad;
	uint8_t cmr_shifted = state->cmr + 3;

	switch (fc) {
	case OSMO_AMRT_FC_SPEECH_GOOD:
	case OSMO_AMRT_FC_SPEECH_DEGRADED:
		is_bad = false;
		break;
	case OSMO_AMRT_FC_SPEECH_BAD:
		is_bad = true;
		break;
	default:
		OSMO_ASSERT(0);
	}

	if (state->rif) {
		if (is_bad) {
			set_bits(tf->c_bits, 0, 3, 2);
			set_bits(tf->d_bits, d_offset, 3, cmr_shifted);
		} else {
			set_bits(tf->c_bits, 0, 3, cmr_shifted);
		}
	} else {
		set_bits(tf->c_bits, 0, 3, is_bad ? 0 : 1);
	}
}

static void encode_8k_speech(struct osmo_trau_frame *tf, enum osmo_amrt_fc fc,
			     const struct osmo_amrt_if *fr,
			     const struct osmo_amrt_encode_state *state)
{
	switch (fr->cmi) {
	case AMR_4_75:
		encode_8k_sp_common_low(tf, fc, fr, state);
		memset(tf->d_bits, 1, 3);
		memcpy(tf->d_bits + 3, fr->s_bits, 48);
		memcpy(tf->d_bits + 59, fr->s_bits + 48, 13);
		memcpy(tf->d_bits + 75, fr->s_bits + 61, 21);
		memcpy(tf->d_bits + 99, fr->s_bits + 82, 13);
		memset(tf->d_bits + 115, 1, 11);
		break;
	case AMR_5_15:
		encode_8k_sp_common_low(tf, fc, fr, state);
		memset(tf->d_bits, 1, 5);
		memcpy(tf->d_bits + 5, fr->s_bits, 46);
		memcpy(tf->d_bits + 59, fr->s_bits + 46, 19);
		memcpy(tf->d_bits + 81, fr->s_bits + 65, 19);
		memcpy(tf->d_bits + 103, fr->s_bits + 84, 19);
		tf->d_bits[125] = 1;
		break;
	case AMR_5_90:
		encode_8k_sp_common_low(tf, fc, fr, state);
		memcpy(tf->d_bits, fr->s_bits, 51);
		memcpy(tf->d_bits + 59, fr->s_bits + 51, 21 + 25 + 21);
		break;
	case AMR_6_70:
		tf->type = OSMO_TRAU8_AMR_6k7;
		memcpy(tf->d_bits, fr->s_bits, 55);
		memcpy(tf->d_bits + 58, fr->s_bits + 55, 79);
		encode_8k_sp_common_high(tf, fc, state, 0);
		break;
	case AMR_7_40:
		tf->type = OSMO_TRAU8_AMR_7k4;
		memcpy(tf->d_bits, fr->s_bits, 58);
		memcpy(tf->d_bits + 61, fr->s_bits + 58, 90);
		encode_8k_sp_common_high(tf, fc, state, 5);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void encode_8k_tfo(struct osmo_trau_frame *tf,
			  const struct osmo_amrt_if *fr, enum osmo_amrt_fc fc)
{
	bool is_sid;

	switch (fc) {
	case OSMO_AMRT_FC_NO_DATA:
	case OSMO_AMRT_FC_ONSET:
	case OSMO_AMRT_FC_SID_FIRST:
		is_sid = false;
		break;
	case OSMO_AMRT_FC_SID_UPDATE:
	case OSMO_AMRT_FC_SID_BAD:
		is_sid = true;
		break;
	default:
		return;
	}

	if (fr->param_info == OSMO_AMRT_TFOP_PRESENT_VALID) {
		memcpy(tf->d_bits + 64, fr->tfo_param_bits, 25);
		memset(tf->d_bits + 89, 0, 3);
		memcpy(tf->d_bits + 95, fr->tfo_param_bits + 25, 20);
		if (!is_sid) {
			memcpy(tf->d_bits + 16, fr->tfo_param_bits + 45, 26);
			memset(tf->d_bits + 42, 0, 2);
		}
	} else {
		/* Par_Type=0,0; the rest is don't-care fill */
		memset(tf->d_bits + 64, 0, 28);
		memset(tf->d_bits + 95, 0, 20);
		if (!is_sid)
			memset(tf->d_bits + 16, 0, 28);
	}
	/* CRC_A and CRC_B are always present */
	osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 64, 28,
				tf->d_bits + 92);
	osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 95, 20,
				tf->d_bits + 115);
	if (!is_sid) {
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 16, 28,
					tf->d_bits + 44);
	}
}

/*! High-level encode an AMR TRAU frame into TRAU-AMR-8k format
 *
 * \param[inout] state Encoding state maintained from one frame to the next.
 * \param[out] tf TRAU frame to be generated.
 * \param[in] fr Frame content to be encoded.
 * \param[in] is_tfo TFO frame is to be emitted, rather than regular TRAU.
 * \param[in] restr Restrictions on the type of frame that can be emitted.
 * \returns code indicating the type of TRAU frame CRC to be computed later.
 *
 * All necessary fields of struct osmo_trau_frame are filled out by this
 * function (followed by osmo_amrt_set_trau_crc()) with the exception of
 * .dir and .dl_ta_usec - those two fields must be set by the application,
 * either before or after calling the present function, but before passing
 * the frame to osmo_trau_frame_encode() or osmo_trau_frame_encode_tfo()
 * for final low-level encoding.
 */
enum osmo_amrt_crc_type
osmo_amrt_encode_trau_frame_8k(struct osmo_amrt_encode_state *state,
				struct osmo_trau_frame *tf,
				const struct osmo_amrt_if *fr, bool is_tfo,
				enum osmo_amrt_8k_restrict restr)
{
	enum osmo_amrt_fc fc;
	enum osmo_amrt_crc_type crc_type;

	fc = encode_common_preproc(state, fr, AMR_7_40);

	/* Apply requested restrictions */
	switch (restr) {
	case OSMO_AMRT_OUT8k_NO_RESTRICT:
		break;
	case OSMO_AMRT_OUT8k_REQUIRE_LOW:
		if (osmo_amrt_fc_is_speech(fc) && fr->cmi >= AMR_5_90)
			fc = OSMO_AMRT_FC_NO_DATA;
		break;
	case OSMO_AMRT_OUT8k_REQUIRE_NO_SPEECH:
		if (osmo_amrt_fc_is_speech(fc))
			fc = OSMO_AMRT_FC_NO_DATA;
		break;
	default:
		/* Don't want to assert just for this...
		 * Treat it as no restriction. */
		break;
	}

	/* Frame stealing for TFO Config_Prot or parameters */
	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON) {
		if (encode_8k_steal_for_tfo(fr, fc))
			fc = OSMO_AMRT_FC_NO_DATA;
	}

	/* Frame stealing for messages in TA field */
	if (fr->ta_dtxd_tfoe_valid && fr->ta >= 0x38 && fr->ta <= 0x3C &&
	    osmo_amrt_fc_is_speech(fc))
		fc = OSMO_AMRT_FC_NO_DATA;

	/* Proceed with output */
	if (osmo_amrt_fc_is_speech(fc)) {
		encode_8k_speech(tf, fc, fr, state);
		crc_type = OSMO_AMRT_CRC_8k_SPEECH_MR475 + fr->cmi;
	} else {
		crc_type = encode_8k_no_speech(tf, fc, fr, state, is_tfo);
	}

	if (fr->config_prot != OSMO_AMRT_CFG_PROT_NO_CON)
		encode_8k_tfo(tf, fr, fc);

	return crc_type;
}

/*! Set CRC fields in prepared AMR TRAU frame
 *
 * \param[inout] tf TRAU frame to be completed with CRC bits.
 * \param[in] crc_type CRC type code returned earlier by
 * osmo_amrt_encode_trau_frame_16k() or osmo_amrt_encode_trau_frame_8k().
 * \returns 0 if successful, or negative if \ref crc_type is invalid.
 *
 * This function should be called after initial encoding with
 * osmo_amrt_encode_trau_frame_{8,16}k() and after any application-specific
 * modifications to that encoded frame, just before passing the finished
 * frame to osmo_trau_frame_encode() or osmo_trau_frame_encode_tfo()
 * for final low-level encoding.
 */
int osmo_amrt_set_trau_crc(struct osmo_trau_frame *tf,
			   enum osmo_amrt_crc_type crc_type)
{
	ubit_t crc_collect[86];		/* 16k No_Speech is the longest */

	switch (crc_type) {
	case OSMO_AMRT_CRC_NONE:
		break;
	case OSMO_AMRT_CRC_16k_NO_SPEECH:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 31, 61);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 86,
					tf->d_bits + 92);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR475:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 44, 16);
		memcpy(crc_collect + 41, tf->d_bits + 61, 2);
		memcpy(crc_collect + 43, tf->d_bits + 64, 9);
		memcpy(crc_collect + 52, tf->d_bits + 88, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 56,
					tf->d_bits + 92);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 95, 2,
					tf->d_bits + 108);
		memcpy(crc_collect, tf->d_bits + 111, 2);
		memcpy(crc_collect + 2, tf->d_bits + 128, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 6,
					tf->d_bits + 132);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 135, 2,
					tf->d_bits + 148);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR515:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 46, 16);
		memcpy(crc_collect + 41, tf->d_bits + 64, 11);
		memcpy(crc_collect + 52, tf->d_bits + 87, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 57,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 2);
		memcpy(crc_collect + 2, tf->d_bits + 109, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 114);
		memcpy(crc_collect, tf->d_bits + 117, 2);
		memcpy(crc_collect + 2, tf->d_bits + 131, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 136);
		memcpy(crc_collect, tf->d_bits + 139, 2);
		memcpy(crc_collect + 2, tf->d_bits + 153, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 158);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR59:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 41, 17);
		memcpy(crc_collect + 42, tf->d_bits + 67, 8);
		memcpy(crc_collect + 50, tf->d_bits + 88, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 54,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 3);
		memcpy(crc_collect + 3, tf->d_bits + 112, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 116);
		memcpy(crc_collect, tf->d_bits + 119, 8);
		memcpy(crc_collect + 8, tf->d_bits + 140, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 12,
					tf->d_bits + 144);
		memcpy(crc_collect, tf->d_bits + 147, 3);
		memcpy(crc_collect + 3, tf->d_bits + 164, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 168);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR67:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 37, 17);
		crc_collect[42] = tf->d_bits[56];
		crc_collect[43] = tf->d_bits[60];
		memcpy(crc_collect + 44, tf->d_bits + 63, 8);
		memcpy(crc_collect + 52, tf->d_bits + 85, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 57,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 113, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 9,
					tf->d_bits + 120);
		memcpy(crc_collect, tf->d_bits + 123, 8);
		memcpy(crc_collect + 8, tf->d_bits + 145, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 13,
					tf->d_bits + 152);
		memcpy(crc_collect, tf->d_bits + 155, 4);
		memcpy(crc_collect + 4, tf->d_bits + 173, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 9,
					tf->d_bits + 180);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR74:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 34, 20);
		memcpy(crc_collect + 45, tf->d_bits + 55, 3);
		memcpy(crc_collect + 48, tf->d_bits + 60, 6);
		memcpy(crc_collect + 54, tf->d_bits + 85, 2);
		memcpy(crc_collect + 56, tf->d_bits + 88, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 59,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 3);
		memcpy(crc_collect + 3, tf->d_bits + 117, 2);
		memcpy(crc_collect + 5, tf->d_bits + 120, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 8,
					tf->d_bits + 124);
		memcpy(crc_collect, tf->d_bits + 127, 6);
		memcpy(crc_collect + 6, tf->d_bits + 152, 2);
		memcpy(crc_collect + 8, tf->d_bits + 155, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 11,
					tf->d_bits + 159);
		memcpy(crc_collect, tf->d_bits + 162, 3);
		memcpy(crc_collect + 3, tf->d_bits + 184, 2);
		memcpy(crc_collect + 5, tf->d_bits + 187, 2);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 191);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR795:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits + 31, 35);
		memcpy(crc_collect + 60, tf->d_bits + 83, 2);
		crc_collect[62] = tf->d_bits[87];
		crc_collect[63] = tf->d_bits[90];
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 64,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 118, 2);
		memcpy(crc_collect + 6, tf->d_bits + 122, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 10,
					tf->d_bits + 127);
		memcpy(crc_collect, tf->d_bits + 130, 8);
		memcpy(crc_collect + 8, tf->d_bits + 155, 2);
		memcpy(crc_collect + 10, tf->d_bits + 159, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 14,
					tf->d_bits + 164);
		memcpy(crc_collect, tf->d_bits + 167, 4);
		memcpy(crc_collect + 4, tf->d_bits + 190, 2);
		memcpy(crc_collect + 6, tf->d_bits + 194, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 10,
					tf->d_bits + 199);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR102:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits, 45);
		memcpy(crc_collect + 70, tf->d_bits + 46, 8);
		memcpy(crc_collect + 78, tf->d_bits + 85, 2);
		memcpy(crc_collect + 80, tf->d_bits + 88, 2);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 82,
					tf->d_bits + 92);
		memcpy(crc_collect, tf->d_bits + 95, 4);
		memcpy(crc_collect + 4, tf->d_bits + 131, 2);
		memcpy(crc_collect + 6, tf->d_bits + 134, 2);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 8,
					tf->d_bits + 138);
		memcpy(crc_collect, tf->d_bits + 141, 8);
		memcpy(crc_collect + 8, tf->d_bits + 180, 2);
		memcpy(crc_collect + 10, tf->d_bits + 183, 2);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 12,
					tf->d_bits + 187);
		memcpy(crc_collect, tf->d_bits + 190, 4);
		memcpy(crc_collect + 4, tf->d_bits + 226, 2);
		memcpy(crc_collect + 6, tf->d_bits + 229, 2);
		memcpy(crc_collect + 8, tf->d_bits + 233, 20);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 28,
					tf->d_bits + 253);
		break;
	case OSMO_AMRT_CRC_16k_SPEECH_MR122:
		memcpy(crc_collect, tf->c_bits, 25);
		memcpy(crc_collect + 25, tf->d_bits, 29);
		memcpy(crc_collect + 54, tf->d_bits + 38, 12);
		memcpy(crc_collect + 66, tf->d_bits + 86, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 69,
					tf->d_bits + 91);
		memcpy(crc_collect, tf->d_bits + 94, 9);
		memcpy(crc_collect + 9, tf->d_bits + 139, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 12,
					tf->d_bits + 144);
		memcpy(crc_collect, tf->d_bits + 147, 12);
		memcpy(crc_collect + 12, tf->d_bits + 195, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 15,
					tf->d_bits + 200);
		memcpy(crc_collect, tf->d_bits + 203, 5);
		memcpy(crc_collect + 5, tf->d_bits + 209, 3);
		memcpy(crc_collect + 8, tf->d_bits + 248, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 11,
					tf->d_bits + 253);
		break;
	case OSMO_AMRT_CRC_8k_SID_UPDATE:
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits, 51);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 56,
					tf->d_bits + 51);
		break;
	case OSMO_AMRT_CRC_8k_SPEECH_MR475:
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits + 3, 16);
		memcpy(crc_collect + 21, tf->d_bits + 20, 2);
		memcpy(crc_collect + 23, tf->d_bits + 23, 9);
		memcpy(crc_collect + 32, tf->d_bits + 47, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 36,
					tf->d_bits + 51);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 54, 7,
					tf->d_bits + 72);
		memcpy(crc_collect, tf->d_bits + 75, 2);
		memcpy(crc_collect + 2, tf->d_bits + 92, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 6,
					tf->d_bits + 96);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, tf->d_bits + 99, 2,
					tf->d_bits + 112);
		break;
	case OSMO_AMRT_CRC_8k_SPEECH_MR515:
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits + 5, 16);
		memcpy(crc_collect + 21, tf->d_bits + 23, 11);
		memcpy(crc_collect + 32, tf->d_bits + 46, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 37,
					tf->d_bits + 51);
		memcpy(crc_collect, tf->d_bits + 54, 7);
		memcpy(crc_collect + 7, tf->d_bits + 73, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 12,
					tf->d_bits + 78);
		memcpy(crc_collect, tf->d_bits + 81, 2);
		memcpy(crc_collect + 2, tf->d_bits + 95, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 100);
		memcpy(crc_collect, tf->d_bits + 103, 2);
		memcpy(crc_collect + 2, tf->d_bits + 117, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 7,
					tf->d_bits + 122);
		break;
	case OSMO_AMRT_CRC_8k_SPEECH_MR59:
		memcpy(crc_collect, tf->c_bits, 5);
		memcpy(crc_collect + 5, tf->d_bits, 17);
		memcpy(crc_collect + 22, tf->d_bits + 26, 8);
		memcpy(crc_collect + 30, tf->d_bits + 47, 4);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 34,
					tf->d_bits + 51);
		break;
	case OSMO_AMRT_CRC_8k_SPEECH_MR67:
		memcpy(crc_collect, tf->c_bits, 3);
		memcpy(crc_collect + 3, tf->d_bits, 17);
		crc_collect[20] = tf->d_bits[19];
		crc_collect[21] = tf->d_bits[23];
		memcpy(crc_collect + 22, tf->d_bits + 26, 8);
		memcpy(crc_collect + 30, tf->d_bits + 48, 5);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 35,
					tf->d_bits + 55);
		break;
	case OSMO_AMRT_CRC_8k_SPEECH_MR74:
		memcpy(crc_collect, tf->c_bits, 3);
		memcpy(crc_collect + 3, tf->d_bits, 20);
		memcpy(crc_collect + 23, tf->d_bits + 21, 3);
		memcpy(crc_collect + 26, tf->d_bits + 26, 6);
		memcpy(crc_collect + 32, tf->d_bits + 51, 2);
		memcpy(crc_collect + 34, tf->d_bits + 54, 3);
		osmo_crc8gen_set_bits(&gsm0860_amr_crc3, crc_collect, 37,
					tf->d_bits + 58);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*! @} */

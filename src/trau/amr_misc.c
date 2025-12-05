/*
 * AMR TRAU interworking facility: miscellaneous functions.
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

/*! Fill AMR frame buffer with constant DHF
 *
 * \param[out] fr AMR frame buffer in intermediate format of the present
 * library.
 * \param[in] mode One of 8 AMR codec modes, AMR_4_75 through AMR_12_2.
 * \returns 0 if successful, or negative if \ref mode is invalid.
 *
 * IMPORTANT NOTE: unlike osmo_amrt_decode_trau_frame() and
 * osmo_amrt_decode_rtp(), this function does NOT fully fill out
 * struct osmo_amrt_if - instead it operates as follows:
 *
 * - frame_class, cmi, cmi_valid and s_bits are set by this function.
 *
 * - Fields related to CMR, RIF, TA+DTXd+TFOE and 3GPP Rel4 TFO configuration
 *   protocol are the responsibility of the application.  If they were
 *   already filled before calling this function, they will not be overwritten;
 *   alternatively the application may fill them after calling this function.
 *
 * This function is needed for implementation of an MGW that interfaces to
 * E1 Abis in the place of a TRAU: during prolonged absence of RTP input
 * (not yet connected through, call on hold etc), the BTS needs to be
 * supplied with valid traffic other than endless No_Data frames; filling
 * with constant DHFs, with mode selected per CMR from the BTS/CCU, is
 * the simplest solution.  The resulting effect is also indistinguishable
 * from a real TRAU operating with G.711 A-law (although not mu-law)
 * when that G.711 A-law input is pure 0xD5 silence.
 */
int osmo_amrt_fill_with_dhf(struct osmo_amrt_if *fr, enum osmo_amr_type mode)
{
	const uint16_t *param;

	switch (mode) {
	case AMR_4_75:
		param = osmo_amr_dhf_4_75;
		break;
	case AMR_5_15:
		param = osmo_amr_dhf_5_15;
		break;
	case AMR_5_90:
		param = osmo_amr_dhf_5_90;
		break;
	case AMR_6_70:
		param = osmo_amr_dhf_6_70;
		break;
	case AMR_7_40:
		param = osmo_amr_dhf_7_40;
		break;
	case AMR_7_95:
		param = osmo_amr_dhf_7_95;
		break;
	case AMR_10_2:
		param = osmo_amr_dhf_10_2;
		break;
	case AMR_12_2:
		param = osmo_amr_dhf_12_2;
		break;
	default:
		return -EINVAL;
	}

	fr->frame_class = OSMO_AMRT_FC_SPEECH_GOOD;
	fr->cmi = mode;
	fr->cmi_valid = true;
	osmo_amr_param_to_sbits(fr->s_bits, param, mode);

	return 0;
}

/*! @} */

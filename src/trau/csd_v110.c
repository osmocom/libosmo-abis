/*
 * This C module contains the implementation of CSD V.110 helper functions
 * for CLEARMODE defined in <osmocom/trau/csd_v110.h>.
 *
 * Author: Mychaela N. Falconia <falcon@freecalypso.org>, 2025 - however,
 * Mother Mychaela's contributions are NOT subject to copyright.
 * No rights reserved, all rights relinquished.
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
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/trau/csd_ra2.h>
#include <osmocom/trau/csd_v110.h>

/*! Convert V.110 frame from distilled 63-bit form (as it appears in TRAU
 *  frames) to full 80-bit form by adding sync pattern bits.
 *  \param[out] out buffer of size 80 ubit_t
 *  \param[in] in 63-bit distilled V.110 frame
 */
void osmo_csd_63bits_to_v110_bits(ubit_t *out, const ubit_t *in)
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

/*! Convert V.110 frame from distilled 63-bit form (as it appears in TRAU
 *  frames) to external octet form (RA2 with 8 kbit/s IR) form by adding
 *  sync pattern bits and applying RA2.
 *  \param[out] out buffer of size 80 octets
 *  \param[in] in 63-bit distilled V.110 frame
 */
void osmo_csd_63bits_to_v110_ir8(uint8_t *out, const ubit_t *in)
{
	ubit_t ra_bits[80];

	osmo_csd_63bits_to_v110_bits(ra_bits, in);
	/* RA2: 1 bit per output byte */
	osmo_csd_ra2_8k_pack(out, ra_bits, 80);
}

/*! Convert V.110 frame from distilled 63-bit form (as it appears in TRAU
 *  frames) to external octet form (RA2 with 16 kbit/s IR) form by adding
 *  sync pattern bits and applying RA2.
 *  \param[out] out buffer of size 40 octets
 *  \param[in] in 63-bit distilled V.110 frame
 */
void osmo_csd_63bits_to_v110_ir16(uint8_t *out, const ubit_t *in)
{
	ubit_t ra_bits[80];

	osmo_csd_63bits_to_v110_bits(ra_bits, in);
	/* RA2: 2 bits per output byte */
	osmo_csd_ra2_16k_pack(out, ra_bits, 40);
}

/*! Check alignment of unpacked 80-bit V.110 frame.
 *  \param[in] ra_bits candidate V.110 frame in standard 80-bit form
 *  \returns true if V.110 alignment pattern is correct, false otherwise
 */
bool osmo_csd_check_v110_align(const ubit_t *ra_bits)
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

/*! Convert V.110 frame from standard 80-bit form (unpacked) to
 *  our distilled 63-bit form (as it appears in TRAU frames).
 *  \param[out] out buffer of size 63 ubit_t
 *  \param[in] in 80-bit standard V.110 frame
 *
 * This function ignores all 17 sync pattern bits and assumes that the
 * V.110 frame is correctly aligned.  Therefore, this function should
 * be called after osmo_csd_check_v110_align().
 */
void osmo_csd_v110_to_63bits(ubit_t *out, const ubit_t *ra_bits)
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

/*
 * This C module contains the implementation of RAA' (RAA-prime) function
 * in the decoding direction, from an A-TRAU frame to user bits.
 * The interface is defined in <osmocom/trau/csd_raa_prime.h>.
 *
 * Author: Mychaela N. Falconia <falcon@freecalypso.org>, 2024 - however,
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
#include <string.h>
#include <errno.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/csd_raa_prime.h>
#include <osmocom/trau/csd_ra2.h>

static int decode_zsp(const ubit_t *atrau_bits, unsigned *pos)
{
	unsigned pos1, n;

	/* guard bits must be set */
	if (!atrau_bits[0] || !atrau_bits[7])
		return -EINVAL;
	pos1 = 0;
	for (n = 0; n < 5; n++) {
		pos1 <<= 1;
		pos1 |= atrau_bits[2 + n];
	}
	if (pos1 < 1 || pos1 > 29)
		return -EINVAL;
	*pos = pos1 - 1;
	return 0;
}

static int decode_subframe(ubit_t *d_bits, const ubit_t *atrau_bits)
{
	ubit_t zi, cbit;
	unsigned pos, z_pos, numd;
	int rc;

	/* consume Zi bit and weed out the trivial case */
	zi = *atrau_bits++;
	if (zi) {
		memcpy(d_bits, atrau_bits, 36);
		return 0;
	}
	pos = 0;
	for (;;) {
		if (pos > 28)
			return -EINVAL;
		rc = decode_zsp(atrau_bits + pos, &z_pos);
		if (rc < 0)
			return rc;
		cbit = atrau_bits[pos + 1];
		if (z_pos < pos)
			return -EINVAL;
		numd = z_pos - pos;
		memcpy(d_bits + pos, atrau_bits + 8 + pos, numd);
		memset(d_bits + pos + numd, 0, 8);
		pos += numd + 8;
		if (cbit)
			break;
	}
	OSMO_ASSERT(pos <= 36);
	numd = 36 - pos;
	memcpy(d_bits + pos, atrau_bits + pos, numd);
	return 0;
}

int osmo_csd144_from_atrau_bits(ubit_t *m_bits, ubit_t *d_bits,
				ubit_t *atrau_c4, ubit_t *atrau_c5,
				const ubit_t *atrau_bits)
{
	static const ubit_t header_bits[20] =  {0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0,
						1, 0, 1, 1};
	int subf, rc;

	if (memcmp(atrau_bits, header_bits, 20) != 0)
		return -EINVAL;
	if (atrau_c4)
		*atrau_c4 = atrau_bits[20];
	if (atrau_c5)
		*atrau_c5 = atrau_bits[21];
	m_bits[0] = atrau_bits[22];
	m_bits[1] = atrau_bits[23];

	/* done with the header, do the 8 subframes */
	atrau_bits += 24;
	for (subf = 0; subf < 8; subf++) {
		rc = decode_subframe(d_bits, atrau_bits);
		if (rc < 0)
			return rc;
		atrau_bits += 37;
		d_bits += 36;
	}
	return 0;
}

int osmo_csd144_from_atrau_ra2(ubit_t *m_bits, ubit_t *d_bits,
				ubit_t *atrau_c4, ubit_t *atrau_c5,
				const uint8_t *atrau_bytes)
{
	ubit_t atrau_bits[OSMO_ATRAU_FRAME_BITS];

	osmo_csd_ra2_16k_unpack(atrau_bits, atrau_bytes, OSMO_ATRAU_RA2_OCTETS);
	return osmo_csd144_from_atrau_bits(m_bits, d_bits, atrau_c4, atrau_c5,
					   atrau_bits);
}

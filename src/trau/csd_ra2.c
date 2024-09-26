/*
 * This C module contains the implementation of CSD RA2 functions
 * defined in <osmocom/trau/csd_ra2.h>.
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

#include <osmocom/core/bits.h>
#include <osmocom/trau/csd_ra2.h>

void osmo_csd_ra2_8k_pack(uint8_t *out_bytes, const ubit_t *in_bits,
			  unsigned n_out_bytes)
{
	unsigned i;

	/* RA2: 1 bit per output byte */
	for (i = 0; i < n_out_bytes; i++)
		out_bytes[i] = 0x7F | (in_bits[i] << 7);
}

void osmo_csd_ra2_8k_unpack(ubit_t *out_bits, const uint8_t *in_bytes,
			    unsigned n_in_bytes)
{
	unsigned i;

	for (i = 0; i < n_in_bytes; i++)
		out_bits[i] = (in_bytes[i] >> 7) & 1;
}

void osmo_csd_ra2_16k_pack(uint8_t *out_bytes, const ubit_t *in_bits,
			   unsigned n_out_bytes)
{
	unsigned i, o;
	uint8_t b;

	/* RA2: 2 bits per output byte */
	i = 0;
	for (o = 0; o < n_out_bytes; o++) {
		b = 0x3F;
		b |= (in_bits[i++] << 7);
		b |= (in_bits[i++] << 6);
		out_bytes[o] = b;
	}
}

void osmo_csd_ra2_16k_unpack(ubit_t *out_bits, const uint8_t *in_bytes,
			     unsigned n_in_bytes)
{
	unsigned i, o;

	o = 0;
	for (i = 0; i < n_in_bytes; i++) {
		out_bits[o++] = (in_bytes[i] >> 7) & 1;
		out_bits[o++] = (in_bytes[i] >> 6) & 1;
	}
}

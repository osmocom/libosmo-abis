/*
 * This C module contains the implementation of RAA' (RAA-prime) function
 * in the encoding direction, from user bits to A-TRAU frame.
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
#include <stdbool.h>
#include <string.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/csd_raa_prime.h>
#include <osmocom/trau/csd_ra2.h>

/* max possible number of Z-sequences per 36-bit subframe */
#define	MAX_ZSEQ_CNT	4

static bool is_z_seq(const ubit_t *d_bits)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (d_bits[i])
			return false;
	}
	return true;
}

static unsigned find_z_seqs(const ubit_t *d_bits, uint8_t *zseq_pos)
{
	unsigned idx, count;

	count = 0;
	for (idx = 0; idx <= 28; ) {
		if (is_z_seq(d_bits + idx)) {
			zseq_pos[count++] = idx;
			idx += 8;
		} else
			idx++;
	}
	return count;
}

static void encode_zsp(ubit_t *out_bits, uint8_t pos_0based, bool is_final)
{
	uint8_t pos_1based = pos_0based + 1;

	*out_bits++ = 1;
	*out_bits++ = is_final;
	*out_bits++ = (pos_1based >> 4) & 1;
	*out_bits++ = (pos_1based >> 3) & 1;
	*out_bits++ = (pos_1based >> 2) & 1;
	*out_bits++ = (pos_1based >> 1) & 1;
	*out_bits++ = (pos_1based >> 0) & 1;
	*out_bits++ = 1;
}

static void atrau_pack_subframe(ubit_t *out_bits, const ubit_t *d_bits)
{
	uint8_t zseq_pos[MAX_ZSEQ_CNT];
	unsigned zseq_count, n;
	unsigned in_pos, z_pos, numd;

	zseq_count = find_z_seqs(d_bits, zseq_pos);
	/* weed out the trivial case of no substitution needed */
	if (zseq_count == 0) {
		out_bits[0] = 1;	/* Zi bit */
		memcpy(out_bits + 1, d_bits, 36);
		return;
	}
	/* we do need substitution: set Zi accordingly */
	*out_bits++ = 0;
	in_pos = 0;
	for (n = 0; n < zseq_count; n++) {
		z_pos = zseq_pos[n];
		encode_zsp(out_bits, z_pos, n == zseq_count - 1);
		out_bits += 8;
		OSMO_ASSERT(z_pos >= in_pos);
		numd = z_pos - in_pos;
		memcpy(out_bits, d_bits + in_pos, numd);
		out_bits += numd;
		in_pos += numd + 8;
	}
	OSMO_ASSERT(in_pos <= 36);
	numd = 36 - in_pos;
	memcpy(out_bits, d_bits + in_pos, numd);
}

void osmo_csd144_to_atrau_bits(ubit_t *out_bits, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5)
{
	int subf;

	/* sync pattern: 16 zeros followed by a single one */
	memset(out_bits, 0, 16);
	out_bits[16] = 1;
	/* C1..C5 per TS 48.020 section 11.1 */
	out_bits[17] = 0;
	out_bits[18] = 1;
	out_bits[19] = 1;
	out_bits[20] = atrau_c4;
	out_bits[21] = atrau_c5;
	/* the two M-bits are pass-through */
	out_bits[22] = m_bits[0];
	out_bits[23] = m_bits[1];
	/* done with the header, do the 8 subframes */
	out_bits += 24;
	for (subf = 0; subf < 8; subf++) {
		atrau_pack_subframe(out_bits, d_bits);
		d_bits += 36;
		out_bits += 37;
	}
}

void osmo_csd144_to_atrau_ra2(uint8_t *out_bytes, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5)
{
	ubit_t atrau_bits[OSMO_ATRAU_FRAME_BITS];

	osmo_csd144_to_atrau_bits(atrau_bits, m_bits, d_bits, atrau_c4,
				  atrau_c5);
	osmo_csd_ra2_16k_pack(out_bytes, atrau_bits, OSMO_ATRAU_RA2_OCTETS);
}

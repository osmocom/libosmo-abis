/*
 * This header file defines the public API for some helper functions that
 * work with CSD V.110 frames represented in the standard 64 kbit/s CLEARMODE
 * format of 3GPP TS 48.103, _without_ compression per TW-TS-007, but _with_
 * Osmocom-introduced constraint that the pair or quadruple of V.110 frames
 * in RA2 encoding has to be naturally aligned within each 20 ms RTP packet.
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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <osmocom/core/bits.h>

/* conversion from distilled 63-bit format (as used in TRAU frames) to full
 * 80-bit V.110 format, with or without RA2. */

void osmo_csd_63bits_to_v110_bits(ubit_t *out, const ubit_t *in);
void osmo_csd_63bits_to_v110_ir8(uint8_t *out, const ubit_t *in);
void osmo_csd_63bits_to_v110_ir16(uint8_t *out, const ubit_t *in);

/* functions to verify V.110 frame alignment after RA2 decoding, and to convert
 * to our distilled 63-bit format. */

bool osmo_csd_check_v110_align(const ubit_t *ra_bits);
void osmo_csd_v110_to_63bits(ubit_t *out, const ubit_t *ra_bits);

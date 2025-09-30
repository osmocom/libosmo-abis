/*
 * This header file defines the public API for working with compressed-CSD
 * RTP format of TW-TS-007:
 *
 * https://www.freecalypso.org/specs/tw-ts-007-v010001.txt
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
#include <osmocom/core/bits.h>

/* RTP payload lengths for different modes of compressed CSD */
#define	OSMO_CCSD_PL_LEN_4k8	16	/* TW-TS-007 section 7.2 */
#define	OSMO_CCSD_PL_LEN_9k6	32	/* TW-TS-007 section 7.3 */
#define	OSMO_CCSD_PL_LEN_14k4	37	/* TW-TS-007 section 7.4 */

/* low-level CSD frame packing and unpacking functions */

void osmo_ccsd_pack_v110_frame(uint8_t *out_bytes, const ubit_t *bits_63);
void osmo_ccsd_pack_atrau_frame(uint8_t *out_bytes, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5);

int osmo_ccsd_unpack_v110_frame(ubit_t *bits_63, const uint8_t *ccsd_bytes);
int osmo_ccsd_unpack_atrau_frame(ubit_t *m_bits, ubit_t *d_bits,
				 ubit_t *atrau_c4, ubit_t *atrau_c5,
				 const uint8_t *ccsd_bytes);

/* compression and decompression functions: conversion between CLEARMODE
 * and TW-TS-007 formats */

int osmo_ccsd_compress(uint8_t *outbuf, size_t outbuf_size,
			const uint8_t *input_pl, size_t input_pl_len);
int osmo_ccsd_decompress(uint8_t *outbuf, size_t outbuf_size,
			 const uint8_t *input_pl, size_t input_pl_len);

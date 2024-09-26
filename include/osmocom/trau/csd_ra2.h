/*
 * Rate adaption function RA2 for GSM CSD calls is defined in 3GPP TS
 * 48.020 chapter 13.  While it is nothing but one special case of I.460
 * submultiplexing and we do have i460_mux infrastructure in Osmocom,
 * that fully-general I.460 infra is too heaviweight for CSD encoding
 * and decoding functions that need simple, single-input and single-output
 * RA2 packing and unpacking functions.
 *
 * This header file defines the API to CSD-oriented RA2 packing and unpacking
 * functions that support intermediate rates of 8 and 16 kbit/s: the two IRs
 * that occur in "plain" single-slot CSD without EDGE.
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

#pragma once

#include <stdint.h>
#include <osmocom/core/bits.h>

/*! pack bits into octets per RA2 with 8 kbit/s IR
 *  param[out] out_bytes caller-provided buffer of size nbytes
 *  param[in] in_bits source bits to be packed, size nbytes
 *  param[in] n_out_bytes number of output bytes to pack, equals
 *            number of source bits with 8 kbit/s IR
 */
void osmo_csd_ra2_8k_pack(uint8_t *out_bytes, const ubit_t *in_bits,
			  unsigned n_out_bytes);

/*! unpack bits from octets per RA2 with 8 kbit/s IR
 *  param[out] out_bits caller-provided buffer of size nbytes
 *  param[in] in_bytes received 64 kbit/s octets to unpack, size nbytes
 *  param[in] n_in_bytes number of received octets to unpack, equals
 *            number of output bits with 8 kbit/s IR
 */
void osmo_csd_ra2_8k_unpack(ubit_t *out_bits, const uint8_t *in_bytes,
			    unsigned n_in_bytes);

/*! pack bits into octets per RA2 with 16 kbit/s IR
 *  param[out] out_bytes caller-provided buffer of size nbytes
 *  param[in] in_bits source bits to be packed, size nbytes*2
 *  param[in] n_out_bytes number of output bytes to pack; the number
 *            of source bits is double this number
 */
void osmo_csd_ra2_16k_pack(uint8_t *out_bytes, const ubit_t *in_bits,
			   unsigned n_out_bytes);

/*! unpack bits from octets per RA2 with 16 kbit/s IR
 *  param[out] out_bits caller-provided buffer of size nbytes*2
 *  param[in] in_bytes received 64 kbit/s octets to unpack, size nbytes
 *  param[in] n_in_bytes number of received octets to unpack; the number
 *            of output bits is double this number
 */
void osmo_csd_ra2_16k_unpack(ubit_t *out_bits, const uint8_t *in_bytes,
			     unsigned n_in_bytes);

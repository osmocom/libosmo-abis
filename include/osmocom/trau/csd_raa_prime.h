/*
 * This header file defines the API to the RAA' (RAA-prime) function of
 * 3GPP TS 48.020 chapter 11, which is a transform applicable to
 * circuit-switched data calls at air interface user rate of 14.4 kbit/s.
 * RAA' converts between the "user" form of 290 bits (288 payload bits
 * plus two M-bits) that appears at the radio channel coder interface
 * and in E-TRAU frames on one side, and external PCM interface A-TRAU
 * frames on the other side.
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

#define	OSMO_ATRAU_FRAME_BITS	320
#define	OSMO_ATRAU_RA2_OCTETS	160

/*! Convert one 20 ms frame of csd144 from user form to A-TRAU frame;
 *  the output is in the form of unpacked bits.
 *
 *  \param[out] out_bits caller-provided buffer of size OSMO_ATRAU_FRAME_BITS
 *  \param[in] m_bits 2 M-bits associated with this 20 ms CSD frame
 *  \param[in] d_bits 288 user data bits in this 20 ms CSD frame
 *  \param[in] atrau_c4 value to be put into A-TRAU frame bit C4
 *  \param[in] atrau_c5 value to be put into A-TRAU frame bit C5
 */
void osmo_csd144_to_atrau_bits(ubit_t *out_bits, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5);

/*! Convert one 20 ms frame of csd144 from user form to A-TRAU frame;
 *  the output is the external output form with RA2 applied.
 *
 *  \param[out] out_bytes caller-provided buffer of size OSMO_ATRAU_RA2_OCTETS
 *  \param[in] m_bits 2 M-bits associated with this 20 ms CSD frame
 *  \param[in] d_bits 288 user data bits in this 20 ms CSD frame
 *  \param[in] atrau_c4 value to be put into A-TRAU frame bit C4
 *  \param[in] atrau_c5 value to be put into A-TRAU frame bit C5
 */
void osmo_csd144_to_atrau_ra2(uint8_t *out_bytes, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5);

/*! Decode aligned A-TRAU frame into user form of 288 D-bits and
 *  two M-bits; the A-TRAU frame input is given in unpacked bit form.
 *
 *  \param[out] m_bits caller-provided buffer for 2 M-bits
 *  \param[out] d_bits caller-provided buffer for 288 user data bits
 *  \param[out] atrau_c4 return of A-TRAU frame bit C4 (NULL pointer OK)
 *  \param[out] atrau_c5 return of A-TRAU frame bit C5 (NULL pointer OK)
 *  \param[in] atrau_bits A-TRAU frame as 320 unpacked bits
 *  \return 0 if A-TRAU frame is good; negative if it is invalid
 */
int osmo_csd144_from_atrau_bits(ubit_t *m_bits, ubit_t *d_bits,
				ubit_t *atrau_c4, ubit_t *atrau_c5,
				const ubit_t *atrau_bits);

/*! Decode aligned A-TRAU frame into user form of 288 D-bits and
 *  two M-bits; the A-TRAU frame input is given in RA2 octet form.
 *
 *  \param[out] m_bits caller-provided buffer for 2 M-bits
 *  \param[out] d_bits caller-provided buffer for 288 user data bits
 *  \param[out] atrau_c4 return of A-TRAU frame bit C4 (NULL pointer OK)
 *  \param[out] atrau_c5 return of A-TRAU frame bit C5 (NULL pointer OK)
 *  \param[in] atrau_bytes A-TRAU frame as 160 RA2 octets
 *  \return 0 if A-TRAU frame is good; negative if it is invalid
 */
int osmo_csd144_from_atrau_ra2(ubit_t *m_bits, ubit_t *d_bits,
				ubit_t *atrau_c4, ubit_t *atrau_c5,
				const uint8_t *atrau_bytes);

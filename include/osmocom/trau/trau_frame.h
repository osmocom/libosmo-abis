#pragma once
/* TRAU frame handling according to GSM TS 48.060 / 48.061 */

/* (C) 2020 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
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
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>

/*! \defgroup trau_frame TRAU frame handling
 *  @{
 *
 *  \file trau_frame.h
 */

/*! \brief Maximum number of C-bits in a TRAU frame:
 * 21 for FR/EFR, 25 for AMR, 15 for OM, 15 for data, 13 for E-data, 21 idle */
#define MAX_C_BITS	25
/*! \brief Maximum number of D-bits in a TRAU frame:
 * 260 for FR/EFR, 256 for AMR, 264 for OM, 288 for E-data */
#define MAX_D_BITS	288
/*! \brief Maximum number of T-bits in a TRAU frame for all speech frames */
#define MAX_T_BITS	4
/*! \brief Maximum number of S-bits in a TRAU frame for OM */
#define MAX_S_BITS	6
/*! \brief Maximum number of M-bits in a TRAU frame  for E-data */
#define MAX_M_BITS	2
/*! \brief Maximum number of XC-bits in a TRAU frame for 8k */
#define MAX_XC_BITS	6

/*! 16k sub-slot types: Bits C1..C5 */
#define TRAU_FT_FR_UP		0x02	/* 0 0 0 1 0 - 3.5.1.1.1 */
#define TRAU_FT_HR_UP		0x03	/* 0 0 0 1 1 - TS 08.61 5.1.4.1.1 */
#define TRAU_FT_FR_DOWN		0x1c	/* 1 1 1 0 0 - 3.5.1.1.1 */
#define TRAU_FT_HR_DOWN		0x1d	/* 1 1 1 0 1 - TS 08.61 5.1.4.1.1 */
#define TRAU_FT_EFR		0x1a	/* 1 1 0 1 0 - 3.5.1.1.1 */
#define TRAU_FT_AMR		0x06	/* 0 0 1 1 0 - 3.5.1.2 */
#define TRAU_FT_OM_UP		0x05	/* 0 0 1 0 1 - 3.5.2 */
#define TRAU_FT_OM_DOWN		0x1b	/* 1 1 0 1 1 - 3.5.2 */
#define TRAU_FT_DATA_UP		0x08	/* 0 1 0 0 0 - 3.5.3 */
#define TRAU_FT_DATA_UP_HR	0x09	/* 0 1 0 0 1 - TS 08.61 5.1.4.2 */
#define TRAU_FT_DATA_DOWN	0x16	/* 1 0 1 1 0 - 3.5.3 */
#define TRAU_FT_DATA_DOWN_HR	0x17	/* 1 0 1 1 1 - TS 08.61 5.1.4.2 */
#define TRAU_FT_D145_SYNC	0x14	/* 1 0 1 0 0 - 3.5.3 */
#define TRAU_FT_EDATA		0x1f	/* 1 1 1 1 1 - 3.5.4 */
#define TRAU_FT_IDLE_UP		0x10	/* 1 0 0 0 0 - 3.5.5 */
#define TRAU_FT_IDLE_DOWN	0x0e	/* 0 1 1 1 0 - 3.5.5 */

/* 8k sub-slot types: Bits C1..C5*/
#define TRAU8_FT_SPEECH_UP	0x02	/* 0 0 0 1 P - TS 08.61 5.2.4.1.1 */
#define TRAU8_FT_DATA_UP	0x06	/* 0 0 1 1 P - TS 08.61 5.2.4.1.1 */
#define TRAU8_FT_OM_UP		0x0a	/* 0 1 0 1 P - TS 08.61 5.2.4.1.1 */
#define TRAU8_FT_SPEECH_DOWN	0x00	/* 0 0 0 U P - TS 08.61 5.2.4.1.2 */
#define TRAU8_FT_DATA_DOWN	0x04	/* 0 0 1 U P - TS 08.61 5.2.4.1.2 */
#define TRAU8_FT_OM_DOWN	0x08	/* 0 1 0 U P - TS 08.61 5.2.4.1.2 */

/*********************************************************************************
 * New API
 *********************************************************************************/

enum osmo_trau_frame_type {
	OSMO_TRAU_FT_NONE,
	OSMO_TRAU16_FT_FR = 1,
	OSMO_TRAU16_FT_HR,
	OSMO_TRAU16_FT_EFR,
	OSMO_TRAU16_FT_AMR,
	OSMO_TRAU16_FT_OAM,
	OSMO_TRAU16_FT_DATA,
	OSMO_TRAU16_FT_EDATA,
	OSMO_TRAU16_FT_D145_SYNC,
	OSMO_TRAU16_FT_DATA_HR,
	OSMO_TRAU16_FT_IDLE,

	OSMO_TRAU8_SPEECH,
	OSMO_TRAU8_DATA,
	OSMO_TRAU8_OAM,
	OSMO_TRAU8_AMR_LOW,
	OSMO_TRAU8_AMR_6k7,
	OSMO_TRAU8_AMR_7k4,
};

extern const struct value_string osmo_trau_frame_type_names[];
static inline const char *osmo_trau_frame_type_name(enum osmo_trau_frame_type ft) {
	return get_value_string(osmo_trau_frame_type_names, ft);
}

enum osmo_trau_frame_direction {
	OSMO_TRAU_DIR_UL	= 0,
	OSMO_TRAU_DIR_DL	= 1,
};

struct osmo_trau_frame {
	enum osmo_trau_frame_type type;
	enum osmo_trau_frame_direction dir;

	ubit_t c_bits[MAX_C_BITS];
	ubit_t d_bits[MAX_D_BITS];
	ubit_t t_bits[MAX_T_BITS];
	ubit_t s_bits[MAX_S_BITS];
	ubit_t m_bits[MAX_M_BITS];
	ubit_t ufi;			/*!< Unreliable Frame Indication (HR) */
	ubit_t crc_bits[3];		/*!< CRC0..CRC2 (HR) */
	ubit_t xc_bits[MAX_XC_BITS];	/*!< XC1..XC6 (8k) */
};

/*! decode an 8k TRAU frame
 *  \param[out] fr caller-allocated output data structure
 *  \param[in] bits unpacked input bits
 *  \param[in] dir direction
 *  \return 0 on success; negative in case of error */
int osmo_trau_frame_decode_8k(struct osmo_trau_frame *fr, const ubit_t *bits,
			      enum osmo_trau_frame_direction dir);

/*! decode an 16k TRAU frame
 *  \param[out] fr caller-allocated output data structure
 *  \param[in] bits unpacked input bits
 *  \param[in] dir direction
 *  \return 0 on success; negative in case of error */
int osmo_trau_frame_decode_16k(struct osmo_trau_frame *fr, const ubit_t *bits,
			       enum osmo_trau_frame_direction dir);

/*! encode a TRAU frame
 *  \param[out] bits caller-allocated memory for unpacked output bits
 *  \param[out] fr input data structure describing TRAU frame
 *  \return number of bits encoded */
int osmo_trau_frame_encode(ubit_t *bits, size_t n_bits, const struct osmo_trau_frame *fr);


/* }@ */

/*
 * This C module contains the implementation of TFO frame functions
 * defined in <osmocom/trau/tfo_frame.h>.
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
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/tfo_frame.h>

#define	NUM_PCM_BYTES		160
#define	NUM_FRAME_BITS_16k	320
#define	NUM_FRAME_BITS_8k	160

/*! Insert TFO-FR/EFR or AMR_TFO_16k frame into PCM sample block
 *  \param[out] pcm Block of 160 PCM samples into which the TFO frame is to be
 *  inserted
 *  \param[in] fr Decoded representation of TRAU-UL/TFO frame to be encoded
 *  \returns 0 if successful, negative on errors
 *
 * For each of the 160 PCM samples in the buffer pointed to by \ref pcm,
 * the two least significant bits are replaced with TFO frame bits, while
 * the upper 6 bits remain unchanged.  The application is expected to fill
 * this buffer with its intended G.711 A-law or mu-law output just prior
 * to calling this function to add TFO.
 *
 * If the application is emitting not only TFO frames, but also embedded
 * TFO messages, the proper sequence is: (1) set bit C5 (fr->c_bits[4])
 * in the frame to be encoded, (2) compute and fill in CRC (only for AMR),
 * (3) call the present function, and (4) insert the TFO message into the
 * same PCM sample block (overwriting the lsb of samples 0, 16, 32, ..., 144)
 * in the same way how it would be done in the absence of TFO frame output.
 * The bits of TFO frame overwritten in the last step will be part of the
 * sync pattern, following GSM 08.62 section 6.2 (renumbered to 7.2 in
 * TS 28.062).
 */
int osmo_tfo_insert_frame_16k(uint8_t *pcm, const struct osmo_trau_frame *fr)
{
	ubit_t bits[NUM_FRAME_BITS_16k];
	unsigned i, o;
	int rc;

	rc = osmo_trau_frame_encode_tfo(bits, sizeof(bits), fr);
	if (rc < 0)
		return rc;
	if (rc != NUM_FRAME_BITS_16k)
		return -EINVAL;
	/* See TS 28.062 section 5.2.3 for bit packing order: it is the
	 * opposite of the bit order in 16 kbit/s subslots on Abis! */
	i = 0;
	for (o = 0; o < NUM_PCM_BYTES; o++) {
		pcm[o] &= 0xFC;
		pcm[o] |= bits[i++] << 0;
		pcm[o] |= bits[i++] << 1;
	}
	return 0;
}

/*! Insert TFO-HRv1 frame into PCM sample block
 *  \param[out] pcm Block of 160 PCM samples into which the TFO frame is to be
 *  inserted
 *  \param[in] fr Decoded representation of TRAU-UL/TFO frame to be encoded
 *  \returns 0 if successful, negative on errors
 *
 * This functions is just like osmo_tfo_insert_frame_16k() except that
 * it operates on the 1 bit wide TFO frame format of GSM 08.62 section 5.2
 * or TS 28.062 section 5.3, which is used only for HRv1 codec.
 * For each of the 160 PCM samples in the buffer pointed to by \ref pcm,
 * the least significant bit is replaced with TFO frame bits, while
 * the upper 7 bits remain unchanged.
 */
int osmo_tfo_insert_frame_hr1(uint8_t *pcm, const struct osmo_trau_frame *fr)
{
	ubit_t bits[NUM_FRAME_BITS_8k];
	unsigned i;
	int rc;

	rc = osmo_trau_frame_encode_tfo(bits, sizeof(bits), fr);
	if (rc < 0)
		return rc;
	if (rc != NUM_FRAME_BITS_8k)
		return -EINVAL;
	for (i = 0; i < NUM_PCM_BYTES; i++) {
		pcm[i] &= 0xFE;
		pcm[i] |= bits[i];
	}
	return 0;
}

/*! Extract TFO-FR/EFR or AMR_TFO_16k frame from PCM samples
 *  \param[out] fr Caller-allocated output data structure
 *  \param[in] pcm Block of 160 PCM samples containing an aligned TFO frame
 *  \returns 0 if successful, negative on errors
 *
 * The application is responsible for detecting the presence of TFO frames
 * in the incoming G.711 PCM sample stream and locating their alignment
 * (both functions are based on the known sync pattern); once an aligned
 * TFO frame is located, call this function to extract and parse it into
 * struct osmo_trau_frame.
 */
int osmo_tfo_extract_frame_16k(struct osmo_trau_frame *fr, const uint8_t *pcm)
{
	ubit_t bits[NUM_FRAME_BITS_16k];
	unsigned i, o;

	o = 0;
	for (i = 0; i < NUM_PCM_BYTES; i++) {
		bits[o++] = (pcm[i] >> 0) & 1;
		bits[o++] = (pcm[i] >> 1) & 1;
	}
	return osmo_trau_frame_decode_tfo_16k(fr, bits);
}

/*! Extract TFO-HRv1 frame from PCM samples
 *  \param[out] fr Caller-allocated output data structure
 *  \param[in] pcm Block of 160 PCM samples containing an aligned TFO frame
 *  \returns 0 if successful, negative on errors
 *
 * The application is responsible for detecting the presence of TFO frames
 * in the incoming G.711 PCM sample stream and locating their alignment
 * (both functions are based on the known sync pattern); once an aligned
 * TFO frame is located, call this function to extract and parse it into
 * struct osmo_trau_frame.
 */
int osmo_tfo_extract_frame_hr1(struct osmo_trau_frame *fr, const uint8_t *pcm)
{
	ubit_t bits[NUM_FRAME_BITS_8k];
	unsigned i;

	for (i = 0; i < NUM_PCM_BYTES; i++)
		bits[i] = pcm[i] & 1;
	return osmo_trau_frame_decode_tfo_hr1(fr, bits);
}

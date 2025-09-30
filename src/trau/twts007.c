/*
 * This C module contains the implementation of TW-TS-007 compressed CSD
 * functions defined in <osmocom/trau/twts007.h>.
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
#include <string.h>
#include <errno.h>

#include <osmocom/core/bits.h>
#include <osmocom/trau/clearmode.h>
#include <osmocom/trau/csd_ra2.h>
#include <osmocom/trau/csd_raa_prime.h>
#include <osmocom/trau/csd_v110.h>
#include <osmocom/trau/twts007.h>

/*! Pack V.110 frame content (distilled 63-bit form) into compressed form
 *  of TW-TS-007 section 7.1.
 *  \param[out] out_bytes buffer of size 8 bytes
 *  \param[in] bits_63 63-bit distilled V.110 frame
 *
 * This function may seem wasteful, doing a memcpy of 63 ubit_t only to
 * add a single constant bit and then call osmo_ubit2pbit().  However,
 * this data movement path is necessary when converting from TRAU frames
 * (struct osmo_trau_frame) to TW-TS-007 compressed format: the 63-bit
 * frame has to be mem'copied out of TRAU d_bits into a staging buffer
 * of 64 ubit_t before it can be packed into octets.
 */
void osmo_ccsd_pack_v110_frame(uint8_t *out_bytes, const ubit_t *bits_63)
{
	ubit_t bits_64[64];

	bits_64[0] = 1;
	memcpy(bits_64 + 1, bits_63, 63);
	osmo_ubit2pbit(out_bytes, bits_64, 64);
}

/*! Pack A-TRAU frame content (M-bits, D-bits, C4 and C5 metadata flags)
 *  into compressed form of TW-TS-007 section 7.4.
 *  \param[out] out_bytes buffer of size 37 bytes
 *  \param[in] m_bits 2 M-bits associated with this 20 ms CSD frame
 *  \param[in] d_bits 288 user data bits in this 20 ms CSD frame
 *  \param[in] atrau_c4 value to be put into A-TRAU frame bit C4
 *  \param[in] atrau_c5 value to be put into A-TRAU frame bit C5
 *
 * This function is API-symmetric with osmo_csd144_to_atrau_ra2(),
 * presenting two alternate ways to represent the same CSD frame
 * in RTP, markedly contrasting in efficiency of both payload size
 * and encoding/decoding logic.
 */
void osmo_ccsd_pack_atrau_frame(uint8_t *out_bytes, const ubit_t *m_bits,
				const ubit_t *d_bits, ubit_t atrau_c4,
				ubit_t atrau_c5)
{
	uint8_t head_byte;

	head_byte = 0xB0;
	head_byte |= (atrau_c4 << 3);
	head_byte |= (atrau_c5 << 2);
	head_byte |= (m_bits[0] << 1);
	head_byte |= (m_bits[1] << 0);
	out_bytes[0] = head_byte;
	osmo_ubit2pbit(out_bytes + 1, d_bits, 288);
}

/*! Extract V.110 frame content (distilled 63-bit form) from compressed form
 *  of TW-TS-007 section 7.1.
 *  \param[out] bits_63 buffer of size 63 ubit_t
 *  \param[in] ccsd_bytes 8-octet compressed V.110 frame of TW-TS-007
 *  section 7.1
 *  \returns 0 if compressed V.110 frame is good; negative if it is invalid
 *
 * The same considerations and questions of potential wastefulness that are
 * covered above for osmo_ccsd_pack_v110_frame() also apply here.  In the
 * opposite direction of RTP input to TRAU frames, when a TW-TS-007 compressed
 * representation of a V.110 frame is to be unpacked into its corresponding
 * 63-bit slot in d_bits of struct osmo_trau_frame, an intermediate unpacking
 * step into a temporary buffer of 64 ubit_t, followed by memcpy into TRAU
 * d_bits, is unavoidably necessary.
 */
int osmo_ccsd_unpack_v110_frame(ubit_t *bits_63, const uint8_t *ccsd_bytes)
{
	ubit_t bits_64[64];

	osmo_pbit2ubit(bits_64, ccsd_bytes, 64);
	if (!bits_64[0])
		return -EINVAL;
	memcpy(bits_63, bits_64 + 1, 63);
	return 0;
}

/*! Extract A-TRAU frame content from compressed form of TW-TS-007 section 7.4.
 *  \param[out] m_bits caller-provided buffer for 2 M-bits
 *  \param[out] d_bits caller-provided buffer for 288 user data bits
 *  \param[out] atrau_c4 return of A-TRAU frame bit C4 (NULL pointer OK)
 *  \param[out] atrau_c5 return of A-TRAU frame bit C5 (NULL pointer OK)
 *  \param[in] ccsd_bytes 37-octet compressed V.110 frame of TW-TS-007
 *  section 7.4
 *  \returns 0 if compressed A-TRAU frame is good; negative if it is invalid
 *
 * This function is API-symmetric with osmo_csd144_from_atrau_ra2();
 * see comments for osmo_ccsd_pack_atrau_frame().
 */
int osmo_ccsd_unpack_atrau_frame(ubit_t *m_bits, ubit_t *d_bits,
				 ubit_t *atrau_c4, ubit_t *atrau_c5,
				 const uint8_t *ccsd_bytes)
{
	if ((ccsd_bytes[0] & 0xF0) != 0xB0)
		return -EINVAL;
	if (atrau_c4)
		*atrau_c4 = (ccsd_bytes[0] >> 3) & 1;
	if (atrau_c5)
		*atrau_c5 = (ccsd_bytes[0] >> 2) & 1;
	m_bits[0] = (ccsd_bytes[0] >> 1) & 1;
	m_bits[1] = (ccsd_bytes[0] >> 0) & 1;
	osmo_pbit2ubit(d_bits, ccsd_bytes + 1, 288);
	return 0;
}

static void compress_v110_frame(uint8_t *out, const ubit_t *ra_bits)
{
	ubit_t bits_64[64];

	/* Do an inline equivalent of osmo_ccsd_pack_v110_frame()
	 * to save the memcpy of 63 ubit_t. */
	bits_64[0] = 1;
	osmo_csd_v110_to_63bits(bits_64 + 1, ra_bits);
	osmo_ubit2pbit(out, bits_64, 64);
}

static int compress_v110_ir8(uint8_t *outbuf, size_t outbuf_size,
			     const uint8_t *input_pl)
{
	ubit_t ra_bits[80 * 2];

	if (outbuf_size < OSMO_CCSD_PL_LEN_4k8)
		return -ENOSPC;

	/* reverse RA2 first */
	osmo_csd_ra2_8k_unpack(ra_bits, input_pl, OSMO_CLEARMODE_20MS);

	/* enforce two properly aligned V.110 frames */
	if (!osmo_csd_check_v110_align(ra_bits))
		return -EINVAL;
	if (!osmo_csd_check_v110_align(ra_bits + 80))
		return -EINVAL;

	/* all checks passed - perform the conversion */
	compress_v110_frame(outbuf, ra_bits);
	compress_v110_frame(outbuf + 8, ra_bits + 80);
	return OSMO_CCSD_PL_LEN_4k8;
}

static int compress_v110_ir16(uint8_t *outbuf, size_t outbuf_size,
			      const uint8_t *input_pl)
{
	ubit_t ra_bits[80 * 4];

	if (outbuf_size < OSMO_CCSD_PL_LEN_9k6)
		return -ENOSPC;

	/* reverse RA2 first */
	osmo_csd_ra2_16k_unpack(ra_bits, input_pl, OSMO_CLEARMODE_20MS);

	/* enforce four properly aligned V.110 frames */
	if (!osmo_csd_check_v110_align(ra_bits))
		return -EINVAL;
	if (!osmo_csd_check_v110_align(ra_bits + 80))
		return -EINVAL;
	if (!osmo_csd_check_v110_align(ra_bits + 80 * 2))
		return -EINVAL;
	if (!osmo_csd_check_v110_align(ra_bits + 80 * 3))
		return -EINVAL;

	/* all checks passed - perform the conversion */
	compress_v110_frame(outbuf, ra_bits);
	compress_v110_frame(outbuf + 8, ra_bits + 80);
	compress_v110_frame(outbuf + 8 * 2, ra_bits + 80 * 2);
	compress_v110_frame(outbuf + 8 * 3, ra_bits + 80 * 3);
	return OSMO_CCSD_PL_LEN_9k6;
}

static int compress_atrau(uint8_t *outbuf, size_t outbuf_size,
			  const uint8_t *input_pl)
{
	ubit_t d_bits[288], m_bits[2], atrau_c4, atrau_c5;
	int rc;

	if (outbuf_size < OSMO_CCSD_PL_LEN_14k4)
		return -ENOSPC;
	rc = osmo_csd144_from_atrau_ra2(m_bits, d_bits, &atrau_c4, &atrau_c5,
					input_pl);
	if (rc < 0)
		return rc;
	osmo_ccsd_pack_atrau_frame(outbuf, m_bits, d_bits, atrau_c4, atrau_c5);
	return OSMO_CCSD_PL_LEN_14k4;
}

/*! Compression function of TW-TS-007 section 8.1: convert RTP payload
 *  from standard-but-aligned CLEARMODE to compressed CSD.
 *  \param[out] outbuf Caller-provided buffer for compressed CSD payload
 *  \param[in] outbuf_size Space available in \ref outbuf
 *  \param[in] input_pl Received CLEARMODE payload
 *  \param[in] input_pl_len Length of received payload
 *  \returns output payload length if successful, negative on errors
 */
int osmo_ccsd_compress(uint8_t *outbuf, size_t outbuf_size,
			const uint8_t *input_pl, size_t input_pl_len)
{
	if (input_pl_len != OSMO_CLEARMODE_20MS)
		return -EINVAL;
	/* format autodetection per TW-TS-007 section 8.1.1 */
	if (input_pl[0] & 0x40)
		return compress_v110_ir8(outbuf, outbuf_size, input_pl);
	if (input_pl[4] & 0x80)
		return compress_v110_ir16(outbuf, outbuf_size, input_pl);
	return compress_atrau(outbuf, outbuf_size, input_pl);
}

static int decompress_v110_ir8(uint8_t *outbuf, const uint8_t *input_pl)
{
	int i;
	ubit_t bits_64[64];

	for (i = 0; i < 2; i++) {
		/* Do an inline equivalent of osmo_ccsd_unpack_v110_frame()
		 * to save the memcpy of 63 ubit_t. */
		osmo_pbit2ubit(bits_64, input_pl, 64);
		if (!bits_64[0])
			return -EINVAL;
		osmo_csd_63bits_to_v110_ir8(outbuf, bits_64 + 1);
		input_pl += 8;
		outbuf += 80;
	}
	return OSMO_CLEARMODE_20MS;
}

static int decompress_v110_ir16(uint8_t *outbuf, const uint8_t *input_pl)
{
	int i;
	ubit_t bits_64[64];

	for (i = 0; i < 4; i++) {
		/* Do an inline equivalent of osmo_ccsd_unpack_v110_frame()
		 * to save the memcpy of 63 ubit_t. */
		osmo_pbit2ubit(bits_64, input_pl, 64);
		if (!bits_64[0])
			return -EINVAL;
		osmo_csd_63bits_to_v110_ir16(outbuf, bits_64 + 1);
		input_pl += 8;
		outbuf += 40;
	}
	return OSMO_CLEARMODE_20MS;
}

static int decompress_atrau(uint8_t *outbuf, const uint8_t *input_pl)
{
	ubit_t d_bits[288], m_bits[2], atrau_c4, atrau_c5;
	int rc;

	rc = osmo_ccsd_unpack_atrau_frame(m_bits, d_bits, &atrau_c4, &atrau_c5,
					  input_pl);
	if (rc < 0)
		return rc;
	osmo_csd144_to_atrau_ra2(outbuf, m_bits, d_bits, atrau_c4, atrau_c5);
	return OSMO_CLEARMODE_20MS;
}

/*! Decompression function of TW-TS-007 section 8.2: convert RTP payload
 *  from compressed CSD to standard CLEARMODE.
 *  \param[out] outbuf Caller-provided buffer for CLEARMODE payload
 *  \param[in] outbuf_size Space available in \ref outbuf
 *  \param[in] input_pl Received compressed CSD payload
 *  \param[in] input_pl_len Length of received payload
 *  \returns output payload length if successful, negative on errors
 */
int osmo_ccsd_decompress(uint8_t *outbuf, size_t outbuf_size,
			 const uint8_t *input_pl, size_t input_pl_len)
{
	if (outbuf_size < OSMO_CLEARMODE_20MS)
		return -ENOSPC;
	switch (input_pl_len) {
	case OSMO_CCSD_PL_LEN_4k8:
		return decompress_v110_ir8(outbuf, input_pl);
	case OSMO_CCSD_PL_LEN_9k6:
		return decompress_v110_ir16(outbuf, input_pl);
	case OSMO_CCSD_PL_LEN_14k4:
		return decompress_atrau(outbuf, input_pl);
	default:
		return -EINVAL;
	}
}

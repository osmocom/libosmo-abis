/*
 * This program is an exerciser for AMR TRAU frame interworking facility
 * of <osmocom/trau/amr_trau.h> in the direction from RTP input decoding
 * to TRAU frame encoding.  It reads RTP payloads (BWE, OA or OAX) from
 * a TW-TS-005 Annex C hex file and converts them to either TRAU-AMR-16k
 * or TRAU-AMR-8k frames as specified on the command line, exercising
 * a chain of libosmotrau functions in the process.
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/amr_trau.h>

#include "../trau_conv/tw5reader.h"

static enum osmo_trau_frame_direction direction;
static bool trau_8k_mode;
static enum osmo_amrt_rtp_format rtp_format;
static struct osmo_amrt_encode_state enc_state;
static FILE *out_file;

static void emit_hex_frame(const uint8_t *frame, unsigned nbytes)
{
	unsigned n;

	for (n = 0; n < nbytes; n++)
		fprintf(out_file, "%02x", frame[n]);
	putc('\n', out_file);
}

static void output_stage_16k(const struct osmo_amrt_if *fr,
			     const char *filename, unsigned lineno)
{
	struct osmo_trau_frame tf;
	ubit_t tf_bits[320];
	uint8_t tf_bytes[40];
	enum osmo_amrt_crc_type crc_type;
	int rc;

	crc_type = osmo_amrt_encode_trau_frame_16k(&enc_state, &tf, fr, false);
	osmo_amrt_set_trau_crc(&tf, crc_type);
	tf.dir = direction;
	tf.dl_ta_usec = 0;
	rc = osmo_trau_frame_encode(tf_bits, sizeof(tf_bits), &tf);
	if (rc != 320) {
		fprintf(stderr,
			"%s line %u: osmo_trau_frame_encode() returned %d\n",
			filename, lineno, rc);
		exit(1);
	}
	osmo_ubit2pbit(tf_bytes, tf_bits, sizeof(tf_bytes) * 8);
	emit_hex_frame(tf_bytes, sizeof(tf_bytes));
}

static void output_stage_8k(const struct osmo_amrt_if *fr,
			    const char *filename, unsigned lineno)
{
	struct osmo_trau_frame tf;
	ubit_t tf_bits[160];
	uint8_t tf_bytes[20];
	enum osmo_amrt_crc_type crc_type;
	int rc;

	crc_type = osmo_amrt_encode_trau_frame_8k(&enc_state, &tf, fr, false,
						  OSMO_AMRT_OUT8k_NO_RESTRICT);
	osmo_amrt_set_trau_crc(&tf, crc_type);
	tf.dir = direction;
	tf.dl_ta_usec = 0;
	rc = osmo_trau_frame_encode(tf_bits, sizeof(tf_bits), &tf);
	if (rc != 160) {
		fprintf(stderr,
			"%s line %u: osmo_trau_frame_encode() returned %d\n",
			filename, lineno, rc);
		exit(1);
	}
	osmo_ubit2pbit(tf_bytes, tf_bits, sizeof(tf_bytes) * 8);
	emit_hex_frame(tf_bytes, sizeof(tf_bytes));
}

static void process_record(const uint8_t *rtp_pl, unsigned rtp_pl_len,
			   const char *filename, unsigned lineno)
{
	struct osmo_amrt_if fr;
	int rc;

	rc = osmo_amrt_decode_rtp(&fr, rtp_pl, rtp_pl_len, rtp_format);
	if (rc < 0) {
		fprintf(out_file, "# input line %u: RTP decode error\n",
			lineno);
	}
	if (trau_8k_mode)
		output_stage_8k(&fr, filename, lineno);
	else
		output_stage_16k(&fr, filename, lineno);
}

static void process_file(const char *infname, const char *outfname)
{
	FILE *inf;
	unsigned lineno;
	uint8_t frame[TWTS005_MAX_FRAME];
	unsigned frame_len;
	int rc;

	inf = fopen(infname, "r");
	if (!inf) {
		perror(infname);
		exit(1);
	}
	if (outfname) {
		out_file = fopen(outfname, "w");
		if (!out_file) {
			perror(outfname);
			exit(1);
		}
	} else {
		out_file = stdout;
	}

	lineno = 0;
	for (;;) {
		rc = twts005_read_frame(inf, &lineno, frame, &frame_len);
		if (rc < 0) {
			fprintf(stderr, "%s line %u: not valid TW-TS-005\n",
				infname, lineno);
			exit(1);
		}
		if (!rc)
			break;
		process_record(frame, frame_len, infname, lineno);
	}

	fclose(inf);
	if (outfname) {
		fclose(out_file);
		out_file = NULL;
	}
}

int main(int argc, char **argv)
{
	int opt;
	extern int optind;

	trau_8k_mode = false;
	direction = OSMO_TRAU_DIR_DL;
	rtp_format = OSMO_AMRT_RTP_FMT_OA;
	while ((opt = getopt(argc, argv, "8bux")) != EOF) {
		switch (opt) {
		case '8':
			trau_8k_mode = true;
			break;
		case 'b':
			rtp_format = OSMO_AMRT_RTP_FMT_BWE;
			break;
		case 'u':
			direction = OSMO_TRAU_DIR_UL;
			break;
		case 'x':
			rtp_format = OSMO_AMRT_RTP_FMT_OAX;
			break;
		default:
			goto usage;
		}
	}
	if (argc < optind + 1 || argc > optind + 2)
		goto usage;

	/* encoder initial state */
	enc_state.cmi = AMR_4_75;
	enc_state.cmr = trau_8k_mode ? AMR_7_40 : AMR_12_2;
	enc_state.rif = 1;
	enc_state.dtxd = 1;
	enc_state.tfoe = 1;
	process_file(argv[optind], argv[optind + 1]);
	exit(0);

usage:	fprintf(stderr, "usage: %s [options] input-file [output-file]\n",
		argv[0]);
	exit(1);
}

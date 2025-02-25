/*
 * This program reads RTP payloads for FR/HR/EFR speech from a TW-TS-005
 * hex file and converts them to either TRAU-DL or TRAU-UL frames
 * as specified on the command line, exercising osmo_rtp2trau() and
 * osmo_trau_frame_encode() functions in the process.
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

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>

#include "tw5reader.h"

static enum osmo_trau_frame_direction direction;
static struct osmo_trau2rtp_state trau2rtp_st;
static FILE *out_file;

static void emit_hex_frame(const uint8_t *frame, unsigned nbytes)
{
	unsigned n;

	for (n = 0; n < nbytes; n++)
		fprintf(out_file, "%02x", frame[n]);
	putc('\n', out_file);
}

static void emit_hex_trau_frame_16k(const ubit_t *tf_bits)
{
	uint8_t tf_bytes[40];

	osmo_ubit2pbit(tf_bytes, tf_bits, sizeof(tf_bytes) * 8);
	emit_hex_frame(tf_bytes, sizeof(tf_bytes));
}

static void emit_hex_trau_frame_8k(const ubit_t *tf_bits)
{
	uint8_t tf_bytes[20];

	osmo_ubit2pbit(tf_bytes, tf_bits, sizeof(tf_bytes) * 8);
	emit_hex_frame(tf_bytes, sizeof(tf_bytes));
}

static void process_record(const uint8_t *rtp_pl, unsigned rtp_pl_len,
			   const char *filename, unsigned lineno)
{
	struct osmo_trau_frame tf;
	ubit_t tf_bits[640]; /* 2x space required by osmo_trau_frame_encode() */
	int rc;

	tf.dir = direction;
	rc = osmo_rtp2trau(&tf, rtp_pl, rtp_pl_len, &trau2rtp_st);
	if (rc < 0) {
		fprintf(stderr, "%s line %u: not valid for osmo_rtp2trau()\n",
			filename, lineno);
		exit(1);
	}
	tf.dl_ta_usec = 0;
	rc = osmo_trau_frame_encode(tf_bits, sizeof(tf_bits), &tf);
	switch (rc) {
	case 320:
		emit_hex_trau_frame_16k(tf_bits);
		break;
	case 160:
		emit_hex_trau_frame_8k(tf_bits);
		break;
	default:
		fprintf(stderr,
			"%s line %u: osmo_trau_frame_encode() returned %d\n",
			filename, lineno, rc);
		exit(1);
	}
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
	char *infname, *outfname;

	if (argc < 4 || argc > 5)
		goto usage;
	infname = argv[1];
	if (!strcmp(argv[2], "fr"))
		trau2rtp_st.type = OSMO_TRAU16_FT_FR;
	else if (!strcmp(argv[2], "hr"))
		trau2rtp_st.type = OSMO_TRAU8_SPEECH;
	else if (!strcmp(argv[2], "hr16"))
		trau2rtp_st.type = OSMO_TRAU16_FT_HR;
	else if (!strcmp(argv[2], "efr"))
		trau2rtp_st.type = OSMO_TRAU16_FT_EFR;
	else
		goto usage;
	if (!strcmp(argv[3], "dl"))
		direction = OSMO_TRAU_DIR_DL;
	else if (!strcmp(argv[3], "ul"))
		direction = OSMO_TRAU_DIR_UL;
	else
		goto usage;
	outfname = argv[4];

	process_file(infname, outfname);
	exit(0);

usage:	fprintf(stderr,
		"usage: %s input-file fr|hr|hr16|efr dl|ul [output-file]\n",
		argv[0]);
	exit(1);
}

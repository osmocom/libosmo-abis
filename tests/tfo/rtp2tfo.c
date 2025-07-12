/*
 * This program reads RTP payloads for FR/HR/EFR speech from a TW-TS-005
 * hex file, converts them to TFO frames (modified TRAU-UL), inserts
 * these TFO frames into dummy G.711 PCM samples, and emits the result
 * in hex format with 80 characters per line, mimicking the format used
 * at Themyscira Wireless for hex extracts from E1 timeslot captures.
 * TFO frame encoding and insertion functions are exercised in the process.
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
#include <osmocom/trau/tfo_frame.h>
#include <osmocom/trau/trau_rtp.h>

#include "../trau_conv/tw5reader.h"

#define	SAMPLES_PER_20MS	160

static struct osmo_trau2rtp_state trau2rtp_st;
static bool is_hr;
static FILE *out_file;

static void print_hex_output(const uint8_t *bin)
{
	int i, j, n;

	n = 0;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 40; j++)
			fprintf(out_file, "%02X", bin[n++]);
		putc('\n', out_file);
	}
}

static void process_record(const uint8_t *rtp_pl, unsigned rtp_pl_len,
			   const char *filename, unsigned lineno)
{
	struct osmo_trau_frame tf;
	uint8_t pcm[SAMPLES_PER_20MS];
	int rc;

	tf.dir = OSMO_TRAU_DIR_UL;
	rc = osmo_rtp2trau(&tf, rtp_pl, rtp_pl_len, &trau2rtp_st);
	if (rc < 0) {
		fprintf(stderr, "%s line %u: not valid for osmo_rtp2trau()\n",
			filename, lineno);
		exit(1);
	}
	/* our dummy G.711 PCM fill is A-law silence */
	memset(pcm, 0xD5, SAMPLES_PER_20MS);
	if (is_hr)
		rc = osmo_tfo_insert_frame_hr1(pcm, &tf);
	else
		rc = osmo_tfo_insert_frame_16k(pcm, &tf);
	if (rc < 0) {
		fprintf(stderr,
			"error: TFO frame insertion function returned %d\n",
			rc);
		exit(1);
	}
	print_hex_output(pcm);
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

	if (argc < 3 || argc > 4)
		goto usage;
	infname = argv[1];
	if (strcmp(argv[2], "fr") == 0) {
		trau2rtp_st.type = OSMO_TRAU16_FT_FR;
		is_hr = false;
	} else if (strcmp(argv[2], "hr") == 0) {
		trau2rtp_st.type = OSMO_TRAU8_SPEECH;
		is_hr = true;
	} else if (strcmp(argv[2], "efr") == 0) {
		trau2rtp_st.type = OSMO_TRAU16_FT_EFR;
		is_hr = false;
	} else {
		goto usage;
	}
	outfname = argv[3];

	process_file(infname, outfname);
	exit(0);

usage:	fprintf(stderr,
		"usage: %s input-file fr|hr|efr [output-file]\n",
		argv[0]);
	exit(1);
}

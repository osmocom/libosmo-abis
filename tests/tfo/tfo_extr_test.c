/*
 * This program exercises extraction of TFO frames from G.711 PCM samples,
 * using osmo_tfo_extract_frame_16k() and osmo_tfo_extract_frame_hr1()
 * functions.  Each extracted TFO frame is then passed through osmo_trau2rtp(),
 * converted into TW-TS-001 or TW-TS-002 RTP format, and finally emitted in
 * TW-TS-005 hex format.  The final output can then be checked for correctness
 * with tw5a-dump and tw5b-dump utilities from Themyscira Wireless GSM codec
 * libraries and utilities package.
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

#include <ctype.h>
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
#include <osmocom/gsm/rtp_extensions.h>

#define	SAMPLES_PER_20MS	160

static uint8_t frame_buf[SAMPLES_PER_20MS];
static unsigned hex_chunk_count;
static bool is_hr;
static FILE *out_file;

static void emit_hex_frame(const uint8_t *frame, unsigned nbytes)
{
	unsigned n;

	for (n = 0; n < nbytes; n++)
		fprintf(out_file, "%02X", frame[n]);
	putc('\n', out_file);
}

static void process_frame(void)
{
	struct osmo_trau2rtp_state trau2rtp_st;
	struct osmo_trau_frame tf;
	uint8_t rtp_pl[40];	/* maximum RTP payload length for TW-TS-005 */
	int rc;

	memset(&trau2rtp_st, 0, sizeof(trau2rtp_st));
	if (is_hr) {
		trau2rtp_st.rtp_extensions = OSMO_RTP_EXT_TWTS002;
		rc = osmo_tfo_extract_frame_hr1(&tf, frame_buf);
	} else {
		trau2rtp_st.rtp_extensions = OSMO_RTP_EXT_TWTS001;
		rc = osmo_tfo_extract_frame_16k(&tf, frame_buf);
	}
	if (rc < 0) {
		fprintf(stderr,
			"error: TFO frame extraction function returned %d\n",
			rc);
		exit(1);
	}
	trau2rtp_st.type = tf.type;
	rc = osmo_trau2rtp(rtp_pl, sizeof(rtp_pl), &tf, &trau2rtp_st);
	if (rc < 0) {
		fprintf(stderr, "error: osmo_trau2rtp() returned %d\n", rc);
		exit(1);
	}
	if (rc == 0) {
		/* TW-TS-005 section 4.3.1 */
		fputs("NULL\n", out_file);
		return;
	}
	emit_hex_frame(rtp_pl, rc);
}

static void process_hex_input(const char *hex_str)
{
	osmo_hexparse(hex_str, frame_buf + hex_chunk_count * 40, 40);
	hex_chunk_count++;
	if (hex_chunk_count >= 4) {
		process_frame();
		hex_chunk_count = 0;
	}
}

static void process_line(char *linebuf, const char *infname, int lineno)
{
	char *cp = linebuf, *hex_str;
	int ndig;

	while (isspace(*cp))
		cp++;
	if (*cp == '\0' || *cp == '#')
		return;
	/* expect string of 80 hex digits */
	hex_str = cp;
	for (ndig = 0; ndig < 80; ndig++) {
		if (!isxdigit(*cp))
			goto inv;
		cp++;
	}
	if (*cp) {
		if (!isspace(*cp))
			goto inv;
		*cp++ = '\0';
	}
	/* must be end of non-comment line */
	while (isspace(*cp))
		cp++;
	if (*cp != '\0' && *cp != '#')
		goto inv;

	process_hex_input(hex_str);
	return;

inv:	fprintf(stderr, "%s line %d: invalid syntax\n", infname, lineno);
	exit(1);
}

static void process_file(const char *infname, const char *outfname)
{
	FILE *inf;
	char linebuf[256];
	int lineno;

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
	hex_chunk_count = 0;
	for (lineno = 1; fgets(linebuf, sizeof(linebuf), inf); lineno++)
		process_line(linebuf, infname, lineno);
	fclose(inf);
	if (outfname) {
		fclose(out_file);
		out_file = NULL;
	}
}

int main(int argc, char **argv)
{
	const char *infname, *outfname;

	if (argc < 3 || argc > 4)
		goto usage;
	infname = argv[1];
	if (strcmp(argv[2], "fr") == 0)
		is_hr = false;
	else if (strcmp(argv[2], "hr") == 0)
		is_hr = true;
	else
		goto usage;
	outfname = argv[3];

	process_file(infname, outfname);
	exit(0);

usage:	fprintf(stderr, "usage: %s input-file fr|hr [output-file]\n",
		argv[0]);
	exit(1);
}

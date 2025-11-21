/*
 * This program is an exerciser for AMR TRAU frame interworking facility
 * of <osmocom/trau/amr_trau.h> in the direction from TRAU frame decoding
 * to RTP output encoding.
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
#include <unistd.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/amr_trau.h>

static enum osmo_trau_frame_direction direction;
static bool trau_8k_mode;
static enum osmo_amrt_rtp_format rtp_format;
static bool allow_config_prot;
static struct osmo_amrt_decode_state dec_state;
static FILE *out_file;

static void do_trau_frame_16k(struct osmo_trau_frame *tf, const char *hex_str)
{
	uint8_t input_bytes[40];
	ubit_t trau_bits[320];
	int rc;

	osmo_hexparse(hex_str, input_bytes, sizeof(input_bytes));
	osmo_pbit2ubit(trau_bits, input_bytes, sizeof(trau_bits));
	rc = osmo_trau_frame_decode_16k(tf, trau_bits, direction);
	if (rc < 0) {
		fprintf(stderr,
			"error: osmo_trau_frame_decode_16k() returned %d\n",
			rc);
		exit(1);
	}
}

static void do_trau_frame_8k(struct osmo_trau_frame *tf, const char *hex_str)
{
	uint8_t input_bytes[20];
	ubit_t trau_bits[160];
	int rc;

	osmo_hexparse(hex_str, input_bytes, sizeof(input_bytes));
	osmo_pbit2ubit(trau_bits, input_bytes, sizeof(trau_bits));
	rc = osmo_trau_frame_decode_8k(tf, trau_bits, direction);
	if (rc < 0) {
		fprintf(stderr,
			"error: osmo_trau_frame_decode_8k() returned %d\n",
			rc);
		exit(1);
	}
}

static void emit_hex_frame(const uint8_t *frame, unsigned nbytes)
{
	unsigned n;

	for (n = 0; n < nbytes; n++)
		fprintf(out_file, "%02X", frame[n]);
	putc('\n', out_file);
}

static void process_record(const char *hex_input, int lineno)
{
	struct osmo_trau_frame tf;
	struct osmo_amrt_if fr;
	enum osmo_amrt_decode_result dec_rc;
	const char *dec_rc_str;
	uint8_t rtp_pl[40];	/* maximum RTP payload length for TW-TS-005 */
	int rc;

	if (trau_8k_mode)
		do_trau_frame_8k(&tf, hex_input);
	else
		do_trau_frame_16k(&tf, hex_input);
	dec_rc = osmo_amrt_decode_trau_frame(&dec_state, &fr, &tf,
						allow_config_prot);
	switch (dec_rc) {
	case OSMO_AMRT_FRAME_DEC_GOOD:
		dec_rc_str = "good";
		break;
	case OSMO_AMRT_FRAME_DEC_BAD:
		dec_rc_str = "UFE";
		break;
	case OSMO_AMRT_FRAME_DEC_REMOTE_BAD:
		dec_rc_str = "DFE";
		break;
	default:
		dec_rc_str = "invalid code";
	}
	fprintf(out_file, "# input line %d frame decoding status: %s\n",
		lineno, dec_rc_str);

	rc = osmo_amrt_encode_rtp(rtp_pl, sizeof(rtp_pl), &fr, rtp_format);
	if (rc < 0) {
		fprintf(stderr, "error: osmo_amrt_encode_rtp() returned %d\n",
			rc);
		exit(1);
	}
	if (rc == 0) {
		/* TW-TS-005 section 4.3.1 */
		fputs("NULL\n", out_file);
		return;
	}
	emit_hex_frame(rtp_pl, rc);
}

static void reset_dec_state(void)
{
	dec_state.cmi_valid = false;
	dec_state.cmr_valid = false;
}

static void process_line(char *linebuf, const char *infname, int lineno)
{
	char *cp = linebuf, *hex_str;
	int ndig, expect_digits;

	while (isspace(*cp))
		cp++;
	if (*cp == '\0' || *cp == '#')
		return;
	/* allow RESET keyword in this stateful AMR version */
	if (strncmp(cp, "RESET", 5) == 0) {
		reset_dec_state();
		return;
	}
	/* expect string of 40 or 80 hex digits */
	if (trau_8k_mode)
		expect_digits = 40;
	else
		expect_digits = 80;
	hex_str = cp;
	for (ndig = 0; ndig < expect_digits; ndig++) {
		if (!isxdigit(*cp))
			goto inv_syntax;
		cp++;
	}
	if (*cp) {
		if (!isspace(*cp))
			goto inv_syntax;
		*cp++ = '\0';
	}
	/* must be end of non-comment line */
	while (isspace(*cp))
		cp++;
	if (*cp != '\0' && *cp != '#')
		goto inv_syntax;

	process_record(hex_str, lineno);
	return;

inv_syntax:
	fprintf(stderr, "%s line %d: invalid syntax\n", infname, lineno);
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
	int opt;
	extern int optind;

	trau_8k_mode = false;
	direction = OSMO_TRAU_DIR_UL;
	rtp_format = OSMO_AMRT_RTP_FMT_OA;
	allow_config_prot = false;
	while ((opt = getopt(argc, argv, "8bcdx")) != EOF) {
		switch (opt) {
		case '8':
			trau_8k_mode = true;
			break;
		case 'b':
			rtp_format = OSMO_AMRT_RTP_FMT_BWE;
			break;
		case 'c':
			allow_config_prot = true;
			break;
		case 'd':
			direction = OSMO_TRAU_DIR_DL;
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

	/* decoder initial state */
	reset_dec_state();
	process_file(argv[optind], argv[optind + 1]);
	exit(0);

usage:	fprintf(stderr, "usage: %s [options] input-file [output-file]\n",
		argv[0]);
	exit(1);
}

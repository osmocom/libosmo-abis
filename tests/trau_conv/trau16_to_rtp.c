/*
 * This program is a libosmotrau unit test for osmo_trau_frame_decode_16k()
 * and osmo_trau2rtp() functions.  It reads input in the form of hex lines,
 * with each line representing a TRAU-16k frame as a string of 80 hex digits.
 * For each test frame thus read, we first call osmo_trau_frame_decode_16k()
 * and print the frame type decoded by that function.  We then call
 * osmo_trau2rtp() and report its integer result, as in length of returned
 * RTP payload or error code.  Finally, for known frame types (initially
 * FR and EFR speech) we break the RTP payload into an array of codec
 * parameters and print them out in the same format as used in Themyscira
 * Wireless GSM codec libraries and utilities.
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

#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/codec/codec.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>
#include <osmocom/gsm/rtp_extensions.h>

static enum osmo_trau_frame_direction direction;
static struct osmo_trau2rtp_state trau2rtp_st;

/*
 * The following function has been copied from Themyscira libgsmfr2 -
 * but it is mostly generated code, produced with 'sweet' utility
 * from classic libgsm source.
 */
static void gsmfr_unpack_to_array(const uint8_t *frame, int16_t *params)
{
	const uint8_t *c = frame;
	unsigned sub;

	params[0]  = (*c++ & 0xF) << 2;
	params[0] |= (*c >> 6) & 0x3;
	params[1]  = *c++ & 0x3F;
	params[2]  = (*c >> 3) & 0x1F;
	params[3]  = (*c++ & 0x7) << 2;
	params[3] |= (*c >> 6) & 0x3;
	params[4]  = (*c >> 2) & 0xF;
	params[5]  = (*c++ & 0x3) << 2;
	params[5] |= (*c >> 6) & 0x3;
	params[6]  = (*c >> 3) & 0x7;
	params[7]  = *c++ & 0x7;
	params += 8;
	for (sub = 0; sub < 4; sub++) {
		params[0]  = (*c >> 1) & 0x7F;
		params[1]  = (*c++ & 0x1) << 1;
		params[1] |= (*c >> 7) & 0x1;
		params[2]  = (*c >> 5) & 0x3;
		params[3]  = (*c++ & 0x1F) << 1;
		params[3] |= (*c >> 7) & 0x1;
		params[4]  = (*c >> 4) & 0x7;
		params[5]  = (*c >> 1) & 0x7;
		params[6]  = (*c++ & 0x1) << 2;
		params[6] |= (*c >> 6) & 0x3;
		params[7]  = (*c >> 3) & 0x7;
		params[8]  = *c++ & 0x7;
		params[9]  = (*c >> 5) & 0x7;
		params[10]  = (*c >> 2) & 0x7;
		params[11]  = (*c++ & 0x3) << 1;
		params[11] |= (*c >> 7) & 0x1;
		params[12]  = (*c >> 4) & 0x7;
		params[13]  = (*c >> 1) & 0x7;
		params[14]  = (*c++ & 0x1) << 2;
		params[14] |= (*c >> 6) & 0x3;
		params[15]  = (*c >> 3) & 0x7;
		params[16]  = *c++ & 0x7;
		params += 17;
	}
}

/*
 * The following function has been copied from Themyscira libgsmefr -
 * but it is mostly generated code, produced with 'sweet' utility
 * from classic libgsm source.
 */
static void EFR_frame2params(const uint8_t *frame, int16_t *params)
{
	const uint8_t *c = frame;

	/* EFR_MAGIC  = (*c >> 4) & 0xF; */
	params[0]  = (*c++ & 0xF) << 3;
	params[0] |= (*c >> 5) & 0x7;
	params[1]  = (*c++ & 0x1F) << 3;
	params[1] |= (*c >> 5) & 0x7;
	params[2]  = (*c++ & 0x1F) << 4;
	params[2] |= (*c >> 4) & 0xF;
	params[3]  = (*c++ & 0xF) << 4;
	params[3] |= (*c >> 4) & 0xF;
	params[4]  = (*c++ & 0xF) << 2;
	params[4] |= (*c >> 6) & 0x3;
	params[5]  = (*c++ & 0x3F) << 3;
	params[5] |= (*c >> 5) & 0x7;
	params[6]  = (*c >> 1) & 0xF;
	params[7]  = (*c++ & 0x1) << 3;
	params[7] |= (*c >> 5) & 0x7;
	params[8]  = (*c >> 1) & 0xF;
	params[9]  = (*c++ & 0x1) << 3;
	params[9] |= (*c >> 5) & 0x7;
	params[10]  = (*c >> 1) & 0xF;
	params[11]  = (*c++ & 0x1) << 3;
	params[11] |= (*c >> 5) & 0x7;
	params[12]  = (*c >> 2) & 0x7;
	params[13]  = (*c++ & 0x3) << 1;
	params[13] |= (*c >> 7) & 0x1;
	params[14]  = (*c >> 4) & 0x7;
	params[15]  = (*c >> 1) & 0x7;
	params[16]  = (*c++ & 0x1) << 2;
	params[16] |= (*c >> 6) & 0x3;
	params[17]  = (*c >> 1) & 0x1F;
	params[18]  = (*c++ & 0x1) << 5;
	params[18] |= (*c >> 3) & 0x1F;
	params[19]  = (*c++ & 0x7) << 1;
	params[19] |= (*c >> 7) & 0x1;
	params[20]  = (*c >> 3) & 0xF;
	params[21]  = (*c++ & 0x7) << 1;
	params[21] |= (*c >> 7) & 0x1;
	params[22]  = (*c >> 3) & 0xF;
	params[23]  = (*c++ & 0x7) << 1;
	params[23] |= (*c >> 7) & 0x1;
	params[24]  = (*c >> 3) & 0xF;
	params[25]  = *c++ & 0x7;
	params[26]  = (*c >> 5) & 0x7;
	params[27]  = (*c >> 2) & 0x7;
	params[28]  = (*c++ & 0x3) << 1;
	params[28] |= (*c >> 7) & 0x1;
	params[29]  = (*c >> 4) & 0x7;
	params[30]  = (*c++ & 0xF) << 1;
	params[30] |= (*c >> 7) & 0x1;
	params[31]  = (*c++ & 0x7F) << 2;
	params[31] |= (*c >> 6) & 0x3;
	params[32]  = (*c >> 2) & 0xF;
	params[33]  = (*c++ & 0x3) << 2;
	params[33] |= (*c >> 6) & 0x3;
	params[34]  = (*c >> 2) & 0xF;
	params[35]  = (*c++ & 0x3) << 2;
	params[35] |= (*c >> 6) & 0x3;
	params[36]  = (*c >> 2) & 0xF;
	params[37]  = (*c++ & 0x3) << 2;
	params[37] |= (*c >> 6) & 0x3;
	params[38]  = (*c >> 3) & 0x7;
	params[39]  = *c++ & 0x7;
	params[40]  = (*c >> 5) & 0x7;
	params[41]  = (*c >> 2) & 0x7;
	params[42]  = (*c++ & 0x3) << 1;
	params[42] |= (*c >> 7) & 0x1;
	params[43]  = (*c >> 2) & 0x1F;
	params[44]  = (*c++ & 0x3) << 4;
	params[44] |= (*c >> 4) & 0xF;
	params[45]  = *c++ & 0xF;
	params[46]  = (*c >> 4) & 0xF;
	params[47]  = *c++ & 0xF;
	params[48]  = (*c >> 4) & 0xF;
	params[49]  = *c++ & 0xF;
	params[50]  = (*c >> 4) & 0xF;
	params[51]  = (*c >> 1) & 0x7;
	params[52]  = (*c++ & 0x1) << 2;
	params[52] |= (*c >> 6) & 0x3;
	params[53]  = (*c >> 3) & 0x7;
	params[54]  = *c++ & 0x7;
	params[55]  = (*c >> 5) & 0x7;
	params[56]  = *c++ & 0x1F;
}

static void decode_fr(const uint8_t *rtp_pl, int conv_result)
{
	int16_t params[76];
	int i, j, p;

	if (conv_result <= 0)
		return;
	switch (conv_result) {
	case 1:
		/* The only valid 1-byte payload is TEH by itself
		 * in the case of TW-TS-001 output. */
		if ((rtp_pl[0] & 0xF0) != 0xE0)
			goto inv_payload;
		printf("  TW-TS-001 TEH octet: 0x%02X\n", rtp_pl[0]);
		return;
	case GSM_FR_BYTES:
		if ((rtp_pl[0] & 0xF0) != 0xD0)
			goto inv_payload;
		/* standard FR payload without TW-TS-001 */
		break;
	case GSM_FR_BYTES+1:
		if ((rtp_pl[0] & 0xF0) != 0xE0)
			goto inv_payload;
		if ((rtp_pl[1] & 0xF0) != 0xD0)
			goto inv_payload;
		/* TW-TS-001 extended RTP payload for FR */
		printf("  TW-TS-001 TEH octet: 0x%02X\n", rtp_pl[0]);
		rtp_pl++;
		break;
	default:
		goto inv_payload;
	}

	printf("  FR frame:\n");
	gsmfr_unpack_to_array(rtp_pl, params);
	/* print frame in the same format as used in ThemWi tools */
	p = 0;
	printf("    LARc");
	for (i = 0; i < 8; i++)
		printf(" %d", params[p++]);
	putchar('\n');
	for (i = 0; i < 4; i++) {
		printf("   ");
		for (j = 0; j < 17; j++)
			printf(" %d", params[p++]);
		putchar('\n');
	}
	printf("    SID recompute: %d\n", (int) osmo_fr_sid_classify(rtp_pl));
	return;

inv_payload:
	printf("  Error: conversion result is not a valid FR payload\n");
}

static void decode_efr(const uint8_t *rtp_pl, int conv_result)
{
	int16_t params[57];
	int i, j, p;

	if (conv_result <= 0)
		return;
	switch (conv_result) {
	case 1:
		/* The only valid 1-byte payload is TEH by itself
		 * in the case of TW-TS-001 output. */
		if ((rtp_pl[0] & 0xF0) != 0xE0)
			goto inv_payload;
		printf("  TW-TS-001 TEH octet: 0x%02X\n", rtp_pl[0]);
		return;
	case GSM_EFR_BYTES:
		if ((rtp_pl[0] & 0xF0) != 0xC0)
			goto inv_payload;
		/* standard EFR payload without TW-TS-001 */
		break;
	case GSM_EFR_BYTES+1:
		if ((rtp_pl[0] & 0xF0) != 0xE0)
			goto inv_payload;
		if ((rtp_pl[1] & 0xF0) != 0xC0)
			goto inv_payload;
		/* TW-TS-001 extended RTP payload for EFR */
		printf("  TW-TS-001 TEH octet: 0x%02X\n", rtp_pl[0]);
		rtp_pl++;
		break;
	default:
		goto inv_payload;
	}

	printf("  EFR frame:\n");
	EFR_frame2params(rtp_pl, params);
	/* print frame in the same format as used in ThemWi tools */
	p = 0;
	printf("    LPC");
	for (i = 0; i < 5; i++)
		printf(" %d", params[p++]);
	putchar('\n');
	for (i = 0; i < 4; i++) {
		printf("   ");
		for (j = 0; j < 13; j++)
			printf(" %d", params[p++]);
		putchar('\n');
	}
	printf("    SID recompute: %d\n", (int) osmo_efr_sid_classify(rtp_pl));
	return;

inv_payload:
	printf("  Error: conversion result is not a valid EFR payload\n");
}

static void process_record(const char *hex_str)
{
	uint8_t input_bytes[40];
	ubit_t trau_bits[320];
	struct osmo_trau_frame tf;
	uint8_t rtp_pl[160];	/* maximum length for CSD */
	int decode_result, conv_result;

	/* In the output, we reprint each input line followed by
	 * our analysis thereof. */
	printf("%s\n", hex_str);
	osmo_hexparse(hex_str, input_bytes, sizeof(input_bytes));
	osmo_pbit2ubit(trau_bits, input_bytes, sizeof(trau_bits));
	decode_result = osmo_trau_frame_decode_16k(&tf, trau_bits, direction);
	if (decode_result < 0) {
		printf("  osmo_trau_frame_decode_16k() returned %d\n",
			decode_result);
		return;
	}
	printf("  TRAU frame type: %s\n", osmo_trau_frame_type_name(tf.type));
	trau2rtp_st.type = tf.type;
	conv_result = osmo_trau2rtp(rtp_pl, sizeof(rtp_pl), &tf, &trau2rtp_st);
	printf("  osmo_trau2rtp() result: %d\n", conv_result);
	/* additional decoding for specific frame types */
	switch (tf.type) {
	case OSMO_TRAU16_FT_FR:
		decode_fr(rtp_pl, conv_result);
		break;
	case OSMO_TRAU16_FT_EFR:
		decode_efr(rtp_pl, conv_result);
		break;
	default:
		break;	/* do nothing */
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
	/* pass ID lines through to output */
	if (!strncmp(cp, "ID", 2)) {
		fputs(cp, stdout);
		return;
	}
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

	process_record(hex_str);
	return;

inv:	fprintf(stderr, "%s line %d: invalid syntax\n", infname, lineno);
	exit(1);
}

int main(int argc, char **argv)
{
	const char *infname;
	FILE *inf;
	char linebuf[256];
	int lineno;

	if (argc != 4)
		goto usage;
	infname = argv[1];
	inf = fopen(infname, "r");
	if (!inf) {
		perror(infname);
		exit(1);
	}
	if (!strcmp(argv[2], "ul"))
		direction = OSMO_TRAU_DIR_UL;
	else if (!strcmp(argv[2], "dl"))
		direction = OSMO_TRAU_DIR_DL;
	else
		goto usage;
	if (!strcmp(argv[3], "std"))
		trau2rtp_st.rtp_extensions = 0;
	else if (!strcmp(argv[3], "tw-ts-001"))
		trau2rtp_st.rtp_extensions = OSMO_RTP_EXT_TWTS001;
	else
		goto usage;

	for (lineno = 1; fgets(linebuf, sizeof(linebuf), inf); lineno++)
		process_line(linebuf, infname, lineno);
	fclose(inf);
	exit(0);

usage:	fprintf(stderr, "usage: %s input-file ul|dl std|tw-ts-001\n", argv[0]);
	exit(1);
}

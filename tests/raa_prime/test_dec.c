/*
 * This C module is a unit test program for libosmotrau implementation
 * of RAA' function in the decoding direction.
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
#include <osmocom/trau/csd_raa_prime.h>

static uint8_t frame_buf[OSMO_ATRAU_RA2_OCTETS];
static unsigned hex_chunk_count;

static void process_frame(void)
{
	ubit_t m_bits[2], d_bits[288];
	uint8_t d_bits_packed[36];
	int rc, i;

	rc = osmo_csd144_from_atrau_ra2(m_bits, d_bits, NULL, NULL, frame_buf);
	if (rc < 0) {
		printf("Bad A-TRAU frame!\n");
		return;
	}
	osmo_ubit2pbit(d_bits_packed, d_bits, 288);
	printf("%u%u ", m_bits[0], m_bits[1]);
	for (i = 0; i < 36; i++)
		printf("%02x", d_bits_packed[i]);
	putchar('\n');
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

int main(int argc, char **argv)
{
	const char *infname;
	FILE *inf;
	char linebuf[256];
	int lineno;

	if (argc != 2) {
		fprintf(stderr, "usage: %s input-file\n", argv[0]);
		exit(1);
	}
	infname = argv[1];
	inf = fopen(infname, "r");
	if (!inf) {
		perror(infname);
		exit(1);
	}

	for (lineno = 1; fgets(linebuf, sizeof(linebuf), inf); lineno++)
		process_line(linebuf, infname, lineno);
	fclose(inf);
	exit(0);
}

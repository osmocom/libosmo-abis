/*
 * This C module is a unit test program for libosmotrau implementation
 * of RAA' function in the encoding direction.
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

/*
 * The output A-TRAU frame in RA2 dressing is 160 binary bytes.
 * But instead of emitting it as a single hex line of 320 digits
 * followed by a single newline, we split it into 4 hex lines
 * of 80 characters each.  This approach is taken because the
 * hex file against which we'll be comparing our output comes from
 * Themyscira Wireless, and ThemWi (unlike Osmocom) treats the
 * 80 characters line length as absolute and inviolable.
 */
static void print_hex_output(const uint8_t *bin)
{
	int i, j, n;

	n = 0;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 40; j++)
			printf("%02X", bin[n++]);
		putchar('\n');
	}
}

static void process_record(const char *mbits_arg, const char *hex_str)
{
	uint8_t input_bytes[36];
	ubit_t m_bits[2], d_bits[288];
	uint8_t outbuf[OSMO_ATRAU_RA2_OCTETS];

	m_bits[0] = mbits_arg[0] - '0';
	m_bits[1] = mbits_arg[1] - '0';
	osmo_hexparse(hex_str, input_bytes, sizeof(input_bytes));
	osmo_pbit2ubit(d_bits, input_bytes, sizeof(d_bits));
	/*
	 * We set C4=1 as will always be the case in BSS->IWF direction:
	 * see Table 3 in TS 48.020 section 11.1.  We set C5=0 in this
	 * unit test to match the condition that existed in the hw test
	 * setup with Nokia TCSM2, which was used to produce the A-TRAU
	 * output we'll be comparing against.
	 */
	osmo_csd144_to_atrau_ra2(outbuf, m_bits, d_bits, 1, 0);
	print_hex_output(outbuf);
}

static void process_line(char *linebuf, const char *infname, int lineno)
{
	char *cp = linebuf, *mbits_arg, *hex_str;
	int ndig;

	while (isspace(*cp))
		cp++;
	if (*cp == '\0' || *cp == '#')
		return;
	/* expect M-bits argument */
	mbits_arg = cp;
	for (ndig = 0; ndig < 2; ndig++) {
		if (*cp != '0' && *cp != '1')
			goto inv;
		cp++;
	}
	if (!isspace(*cp))
		goto inv;
	*cp++ = '\0';
	while (isspace(*cp))
		cp++;
	if (*cp == '\0' || *cp == '#')
		goto inv;
	/* expect string of 72 hex digits */
	hex_str = cp;
	for (ndig = 0; ndig < 72; ndig++) {
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

	process_record(mbits_arg, hex_str);
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

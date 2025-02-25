/*
 * This C module has been adapted from Themyscira Wireless GSM codec libraries
 * and utilities suite.  It implements a function that reads RTP payloads
 * from hex files in TW-TS-005 format.
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
#include <string.h>
#include "tw5reader.h"

static int decode_hex_digit(char c)
{
	if (isdigit(c))
		return c - '0';
	else if (isupper(c))
		return c - 'A' + 10;
	else
		return c - 'a' + 10;
}

int twts005_read_frame(FILE *hexf, unsigned *lineno, uint8_t *frame,
			unsigned *lenp)
{
	char linebuf[82];
	char *cp, *np;
	uint8_t *dp;
	unsigned len;

	for (;;) {
		if (!fgets(linebuf, sizeof(linebuf), hexf))
			return 0;
		(*lineno)++;
		if (!strchr(linebuf, '\n'))
			return -2;
		for (cp = linebuf; isspace(*cp); cp++)
			;
		if (*cp != '\0' && *cp != '#')
			break;
	}
	for (np = cp; *cp && !isspace(*cp); cp++)
		;
	if (*cp)
		*cp++ = '\0';
	while (isspace(*cp))
		cp++;
	if (*cp != '\0' && *cp != '#')
		return -1;
	if (!strcasecmp(np, "NULL")) {
		*lenp = 0;
		return 1;
	}

	dp = frame;
	len = 0;
	for (cp = np; *cp; cp += 2) {
		if (!isxdigit(cp[0]) || !isxdigit(cp[1]))
			return -1;
		*dp++ = (decode_hex_digit(cp[0]) << 4) |
			decode_hex_digit(cp[1]);
		len++;
	}
	*lenp = len;
	return 1;
}

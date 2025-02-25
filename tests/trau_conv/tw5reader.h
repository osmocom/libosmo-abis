/*
 * This header file defines the interface to our reader function for
 * hexadecimal RTP frame sequence files in TW-TS-005 format.
 *
 * twts005_read_frame() return values are:
 *  1 = successfully read valid frame
 *  0 = normal EOF
 * -1 = read line with invalid content
 * -2 = line too long or missing newline
 *
 * The reader function skips blank, whitespace-only and comment lines,
 * returning only actual frames.  lineno variable must be initialized to 0
 * by the application program, but not touched otherwise.  In case of an
 * error, this variable will hold the line number at which the error was
 * encountered.
 */

#pragma once

#define	TWTS005_MAX_FRAME	40

int twts005_read_frame(FILE *hexf, unsigned *lineno, uint8_t *frame,
			unsigned *lenp);

/*
 * This header file defines the public API for working with TFO frames
 * of GSM 08.62 (or 3GPP TS 28.062) chapter 5.  These frames are overlapped
 * onto G.711 PCM speech, replacing one or two lsbs of each PCM sample,
 * hence the two required functions are insertion for output and extraction
 * for input.
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

#pragma once

#include <osmocom/trau/trau_frame.h>

int osmo_tfo_insert_frame_16k(uint8_t *pcm, const struct osmo_trau_frame *fr);
int osmo_tfo_insert_frame_hr1(uint8_t *pcm, const struct osmo_trau_frame *fr);

int osmo_tfo_extract_frame_16k(struct osmo_trau_frame *fr, const uint8_t *pcm);
int osmo_tfo_extract_frame_hr1(struct osmo_trau_frame *fr, const uint8_t *pcm);

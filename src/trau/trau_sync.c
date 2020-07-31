/* GSM A-bis TRAU frame synchronization as per TS 48.060 / 48.061 */

/* (C) 2020 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>

#include <osmocom/core/msgb.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/fsm.h>

#include "ubit_buf.h"
#include <osmocom/trau/trau_sync.h>

#define S(x)	(1 << (x))

#define MAX_TRAU_BYTES 	40

#define T_SYNC		1

struct sync_pattern {
	/* provided by user */
	const char *name;				/*!< human-readable name */
	const uint8_t byte_pattern[MAX_TRAU_BYTES];	/*!< bytes to match against */
	const uint8_t byte_mask[MAX_TRAU_BYTES];	/*!< mask applied before matching */
	uint8_t byte_len;				/*!< length of mask in bytes */

	/* generated by code */
	ubit_t ubit_pattern[MAX_TRAU_BYTES*8];		/*!< bits to match against */
	ubit_t ubit_mask[MAX_TRAU_BYTES*8];		/*!< mask applied before matching */
	uint8_t bitcount;				/*!< number of high bits in mask */
};

static struct sync_pattern sync_patterns[] = {
	[OSMO_TRAU_SYNCP_16_FR_EFR] = {
		/* TS 08.60 Section 4.8.1 */
		.name = "FR/EFR",
		.byte_pattern = {
			0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		},
		.byte_mask = {
			0xff, 0xff, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
		},
		.byte_len = 40,
	},
	[OSMO_TRAU_SYNCP_8_HR] = {
		/* TS 08.61 Section 6.8.2.1.1 */
		.name = "HR8",
		.byte_pattern = {
			0x00, 0x80, 0x40, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
		},
		.byte_mask = {
			0xff, 0x80, 0xC0, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
		},
		.byte_len = 20,
	},
	[OSMO_TRAU_SYNCP_8_AMR_LOW] = {
		/* TS 08.61 Section 6.8.2.1.2 */
		/* The frame synchronisation for No_Speech frames and the speech frames of the three lower codec modes */
		.name = "AMR8_LOW",
		.byte_pattern = {
			0x00, 0x80, 0x80, 0x40,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
		},
		.byte_mask = {
			0xff, 0x80, 0x80, 0xC0,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x80,
		},
		.byte_len = 20,
	},
	[OSMO_TRAU_SYNCP_8_AMR_6K7] = {
		/* The frame synchronisation for the speech frames for codec mode 6,70 kBit/s */
		.name = "AMR8_67",
		.byte_pattern = {
			0x00, 0x80, 0x80, 0x80,
			0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
		},
		.byte_mask = {
			0xff, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
			0x80, 0x00, 0x80, 0x00,
		},
		.byte_len = 20
	},
	[OSMO_TRAU_SYNCP_8_AMR_7K4] = {
		/* The frame synchronisation for the speech frames for codec mode 7,40 kBit/s */
		.name = "AMR8_74",
		.byte_pattern = {
			0x20, 0x00, 0x80, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
		},
		.byte_mask = {
			0xe0, 0x80, 0x80, 0x80,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
		},
		.byte_len = 20,
	},
};

#if 0
static struct sync_pattern rbs_ccu_sync_ind_16_pattern = {
	.name ="RBS_CCU_SYNC_IND_16",
	.byte_pattern = {
		0x00, 0x00, 0x01, 0x00,
		0x00, 0x00, 0x01, 0x00,
		0x01, 0x00, 0x01, 0x00,
	},
	.byte_mask = {
		0xff, 0xff, 0x01, 0x00,
		0x01, 0x00, 0x01, 0x00,
		0x01, 0x00, 0x01, 0x00,
	},
};

static struct sync_pattern rbs_ccu_data_ind_16_pattern = {
	.name ="RBS_CCU_DATA_IND_16",
	.byte_pattern = {
		0x00, 0x00, 0x01, 0x00,
	},
	.byte_mask = {
		0xff, 0xff, 0x01, 0x00,
	},
};
#endif


static void expand_sync_pattern(struct sync_pattern *pat)
{
	osmo_pbit2ubit(pat->ubit_pattern, pat->byte_pattern, pat->byte_len*8);
	osmo_pbit2ubit(pat->ubit_mask, pat->byte_mask, pat->byte_len*8);
}

static unsigned int count_one_bits(const ubit_t *in, unsigned int in_bits)
{
	unsigned int i, count = 0;

	for (i = 0; i < in_bits; i++) {
		if (in[i])
			count++;
	}
	return count;
}

static void sync_pattern_register(struct sync_pattern *p)
{
	expand_sync_pattern(p);
	p->bitcount = count_one_bits(p->ubit_mask, p->byte_len*8);
}

#if 0
/*! correlate pattern with unpacked bits from buffer.
 *  \param[in] pattern sync_pattern against which we shall compare
 *  \param[in] bits unpacked bits to compare against pattern
 *  \param[in] num_bits number of unpacked bits
 *  \returns number of bits not matching pattern; -1 if insufficient bits available. */
static int correlate_pattern_ubits(const struct sync_pattern *pattern,
				    const ubit_t *bits, size_t num_bits)
{
	int i, num_wrong = 0;

	if (num_bits < pattern->byte_len*8)
		return -1; /* insufficient data */

	for (i = 0; i < pattern->byte_len *8; i++) {
		/* if mask doesn't contain '1', we can skip this octet */
		if (!pattern->ubit_mask)
			continue;
		if (bits[i] != pattern->ubit_pattern[i])
			num_wrong++;
	}

	return num_wrong;
}
#endif

struct trau_rx_sync_state {
	/*! call-back to be called for every TRAU frame (called with
	 * bits=NULL in case of frame sync loss */
	frame_out_cb_t out_cb;
	/*! opaque user data; passed to out_cb */
	void *user_data;

	/*! history of received bits */
	ubit_t history[MAX_TRAU_BYTES*8+1]; /* +1 not required, but helps to expose bugs */
	/*! index of next-to-be-written ubit in history */
	unsigned int history_idx;
	/*! the pattern we are trying to sync to */
	const struct sync_pattern *pattern;
	/*! number of consecutive frames without sync */
	unsigned int num_consecutive_errors;
};

/* correlate the history (up to the last received bit) against the pattern */
static int correlate_history_against_pattern(struct trau_rx_sync_state *tss)
{
	const struct sync_pattern *pattern = tss->pattern;
	int i, start, num_wrong = 0;

	/* compute index of first bit in history array */
	start = (ARRAY_SIZE(tss->history) + tss->history_idx - pattern->byte_len*8)
		% ARRAY_SIZE(tss->history);

	OSMO_ASSERT(ARRAY_SIZE(tss->history) >= pattern->byte_len*8);

	for (i = 0; i < pattern->byte_len*8; i++) {
		unsigned int pos = (start + i) % ARRAY_SIZE(tss->history);

		/* if mask doesn't contain '1', we can skip this octet */
		if (!pattern->ubit_mask[i])
			continue;
		if (tss->history[pos] != pattern->ubit_pattern[i])
			num_wrong++;
	}

	return num_wrong;
}

/* add (append) one ubit to the history; wrap as needed */
static void rx_history_add_bit(struct trau_rx_sync_state *tss, ubit_t bit)
{
	tss->history[tss->history_idx] = bit;
	/* simply wrap around at the end */
	tss->history_idx = (tss->history_idx + 1) % ARRAY_SIZE(tss->history);
}

/* append bits to history. We assume that this does NOT wrap */
static void rx_history_add_bits(struct trau_rx_sync_state *tss, const ubit_t *bits, size_t n_bits)
{
	unsigned int frame_bits_remaining = tss->pattern->byte_len*8 - tss->history_idx;
	OSMO_ASSERT(frame_bits_remaining >= n_bits);
	memcpy(&tss->history[tss->history_idx], bits, n_bits);
	tss->history_idx = tss->history_idx + n_bits;
}

/* align the history, i.e. next received bit is start of frame */
static void rx_history_align(struct trau_rx_sync_state *tss)
{
	ubit_t tmp[sizeof(tss->history)];
	size_t history_size = sizeof(tss->history);
	size_t pattern_bits = tss->pattern->byte_len*8;
	size_t first_bit = (history_size + tss->history_idx - pattern_bits) % history_size;
	int i;

	/* we need to shift the last received frame to the start of the history buffer;
	 * do this in two steps: First copy to a local buffer on the stack, using modulo-arithmetic
	 * as index into the history.  Second, copy it back to history */

	for (i = 0; i < pattern_bits; i++)
		tmp[i] = tss->history[(first_bit + i) % history_size];

	memcpy(tss->history, tmp, history_size);
	tss->history_idx = 0;
}

enum trau_sync_state {
	WAIT_FRAME_ALIGN,
	FRAME_ALIGNED,
	/* if at least 3 consecutive frames with each at least one framing error have been received */
	FRAME_ALIGNMENT_LOST,
};

enum trau_sync_event {
	TRAUSYNC_E_RESET,
	/*! a buffer of bits was received (msgb with ubits) */
	TRAUSYNC_E_RX_BITS,
};

static const struct value_string trau_sync_event_names[] = {
	{ TRAUSYNC_E_RESET, "RESET" },
	{ TRAUSYNC_E_RX_BITS, "RX_BITS" },
	{ 0, NULL }
};


static void trau_sync_wait_align(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trau_rx_sync_state *tss = (struct trau_rx_sync_state *) fi->priv;
	struct ubit_buf *ubb;

	switch (event) {
	case TRAUSYNC_E_RX_BITS:
		ubb = data;
		/* append every bit individually + check if we have sync */
		while (ubb_length(ubb) > 0) {
			ubit_t bit = ubb_pull_ubit(ubb);
			int rc;

			rx_history_add_bit(tss, bit);
			rc = correlate_history_against_pattern(tss);
			if (!rc) {
				osmo_fsm_inst_state_chg(fi, FRAME_ALIGNED, 0, 0);
				/* treat remainder of input bits in correct state */
				osmo_fsm_inst_dispatch(fi, TRAUSYNC_E_RX_BITS, ubb);
				return;
			}
		}
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void trau_sync_aligned_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct trau_rx_sync_state *tss = (struct trau_rx_sync_state *) fi->priv;
	/* dispatch aligned frame to user */
	rx_history_align(tss);
	tss->out_cb(tss->user_data, tss->history, tss->pattern->byte_len*8);
}

static void trau_sync_aligned(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct trau_rx_sync_state *tss = (struct trau_rx_sync_state *) fi->priv;
	struct ubit_buf *ubb;
	int rc;

	switch (event) {
	case TRAUSYNC_E_RX_BITS:
		ubb = data;
		while (ubb_length(ubb)) {
			unsigned int frame_bits_remaining = tss->pattern->byte_len*8 - tss->history_idx;
			if (ubb_length(ubb) < frame_bits_remaining) {
				/* frame not filled by this message; just add data */
				rx_history_add_bits(tss, ubb_data(ubb), ubb_length(ubb));
				ubb_pull(ubb, ubb_length(ubb));
			} else {
				/* append as many bits as are missing in the current frame */
				rx_history_add_bits(tss, ubb_data(ubb), frame_bits_remaining);
				ubb_pull(ubb, frame_bits_remaining);

				/* check if we still have frame sync */
				rc = correlate_history_against_pattern(tss);
				if (rc > 0) {
					tss->num_consecutive_errors++;
					if (tss->num_consecutive_errors >= 3) {
						tss->history_idx = 0;
						/* send NULL frame to user */
						tss->out_cb(tss->user_data, NULL, 0);
						osmo_fsm_inst_state_chg(fi, FRAME_ALIGNMENT_LOST, 1, T_SYNC);
						osmo_fsm_inst_dispatch(fi, TRAUSYNC_E_RX_BITS, ubb);
						return;
					}
				} else
					tss->num_consecutive_errors = 0;

				/* dispatch aligned frame to user */
				tss->out_cb(tss->user_data, tss->history, tss->history_idx);
				tss->history_idx = 0;
			}
		}
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void trau_sync_alignment_lost(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	/* we try to restore sync for some amount of time before generating an error */

	switch (event) {
	case TRAUSYNC_E_RX_BITS:
		trau_sync_wait_align(fi, event, data);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static void trau_sync_allstate(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	switch (event) {
	case TRAUSYNC_E_RESET:
		osmo_fsm_inst_state_chg(fi, WAIT_FRAME_ALIGN, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static int trau_sync_timeout(struct osmo_fsm_inst *fi)
{
	switch (fi->T) {
	case T_SYNC:
		/* if Tsync expires before frame synchronization is
		 * again obtained the TRAU initiates sending of the
		 * urgent alarm pattern described in clause 4.10.2. */
		osmo_fsm_inst_state_chg(fi, WAIT_FRAME_ALIGN, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
	}
	return 0;
}

static const struct osmo_fsm_state trau_sync_states[] = {
	[WAIT_FRAME_ALIGN] = {
		.name = "WAIT_FRAME_ALIGN",
		.in_event_mask = S(TRAUSYNC_E_RX_BITS),
		.out_state_mask = S(FRAME_ALIGNED),
		.action = trau_sync_wait_align,
	},
	[FRAME_ALIGNED] = {
		.name = "FRAME_ALIGNED",
		.in_event_mask = S(TRAUSYNC_E_RX_BITS),
		.out_state_mask = S(FRAME_ALIGNMENT_LOST) | S(WAIT_FRAME_ALIGN),
		.action = trau_sync_aligned,
		.onenter = trau_sync_aligned_onenter,
	},
	[FRAME_ALIGNMENT_LOST] = {
		.name = "FRAME_ALIGNMENT_LOST",
		.in_event_mask = S(TRAUSYNC_E_RX_BITS),
		.out_state_mask = S(WAIT_FRAME_ALIGN) | S(FRAME_ALIGNED),
		.action = trau_sync_alignment_lost,
	},
};

static struct osmo_fsm trau_sync_fsm = {
	.name = "trau_sync",
	.states = trau_sync_states,
	.num_states = ARRAY_SIZE(trau_sync_states),
	.allstate_event_mask = S(TRAUSYNC_E_RESET),
	.allstate_action = trau_sync_allstate,
	.timer_cb = trau_sync_timeout,
	.log_subsys = DLGLOBAL,
	.event_names = trau_sync_event_names,
};


struct osmo_fsm_inst *
osmo_trau_sync_alloc(void *ctx, const char *name, frame_out_cb_t frame_out_cb,
		     enum osmo_tray_sync_pat_id pat_id, void *user_data)
{
	struct trau_rx_sync_state *tss;
	struct osmo_fsm_inst *fi;

	if (pat_id >= ARRAY_SIZE(sync_patterns))
		return NULL;

	fi = osmo_fsm_inst_alloc(&trau_sync_fsm, ctx, NULL, LOGL_NOTICE, name);
	if (!fi)
		return NULL;
	tss = talloc_zero(fi, struct trau_rx_sync_state);
	if (!tss) {
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_ERROR, NULL);
		return NULL;
	}
	fi->priv = tss;

	tss->out_cb = frame_out_cb;
	tss->user_data = user_data;
	/* FIXME: this must be configurable */
	tss->pattern = &sync_patterns[pat_id];

	/* An unusued E1 timeslot normally would send an idle signal that
	 * has all bits set to one. In order to prevent false-positive
	 * synchronization on startup we set all history bits to 1, to make
         * it look like a signal from an unused timeslot. */
	memset(tss->history, 1, sizeof(tss->history));

	return fi;
}

void osmo_trau_sync_rx_ubits(struct osmo_fsm_inst *fi, const ubit_t *bits, size_t n_bits)
{
	struct ubit_buf ubb;
	ubb_init(&ubb, bits, n_bits);
	osmo_fsm_inst_dispatch(fi, TRAUSYNC_E_RX_BITS, &ubb);
}

static void __attribute__((constructor)) on_dso_load_sync(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sync_patterns); i++)
		sync_pattern_register(&sync_patterns[i]);
	osmo_fsm_register(&trau_sync_fsm);
}

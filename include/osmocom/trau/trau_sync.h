#pragma once
#include <osmocom/core/bits.h>
#include <osmocom/core/fsm.h>

enum osmo_tray_sync_pat_id {
	OSMO_TRAU_SYNCP_16_FR_EFR,
	OSMO_TRAU_SYNCP_8_HR,
	OSMO_TRAU_SYNCP_8_AMR_LOW,
	OSMO_TRAU_SYNCP_8_AMR_6K7,
	OSMO_TRAU_SYNCP_8_AMR_7K4,
};

typedef void (*frame_out_cb_t)(void *user_data, const ubit_t *bits, unsigned int num_bits);

struct osmo_fsm_inst *
osmo_trau_sync_alloc(void *ctx, const char *name, frame_out_cb_t frame_out_cb,
		     enum osmo_tray_sync_pat_id pat_id, void *user_data);

void osmo_trau_sync_rx_ubits(struct osmo_fsm_inst *fi, const ubit_t *bits, size_t n_bits);
void osmo_trau_sync_set_pat(struct osmo_fsm_inst *fi, enum osmo_tray_sync_pat_id pat_id);


#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/bits.h>

#include <osmocom/trau/trau_sync.h>

static void frame_out_cb(void *user_data, const ubit_t *bits, unsigned int num_bits)
{
	char *str = user_data;
	printf("demux_bits_cb '%s': %s\n", str, osmo_ubit_dump(bits, num_bits));
}

static const uint8_t sync_pattern[] = {
	0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
	0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
	0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
	0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
	0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
};

#define ASSERT_STATE(fi, x) OSMO_ASSERT(!strcmp(osmo_fsm_inst_state_name(fi), x))

static void test_body(void)
{
	struct osmo_fsm_inst *fi = osmo_trau_sync_alloc(NULL, "test", frame_out_cb, OSMO_TRAU_SYNCP_16_FR_EFR, "test");
	OSMO_ASSERT(fi);

	printf("\n==> %s\n", __func__);

	ubit_t bits[40*8];

	/* send some invalid data */
	memset(bits, 0, sizeof(bits));
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	osmo_trau_sync_rx_ubits(fi, bits, 23);

	/* first valid frame */
	osmo_pbit2ubit(bits, sync_pattern, sizeof(sync_pattern)*8);
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNED");

	/* second valid frame */
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNED");

	/* send wrong frame */
	memset(bits, 1, sizeof(bits));
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNED");

	/* intersperse a valid frame */
	osmo_pbit2ubit(bits, sync_pattern, sizeof(sync_pattern)*8);
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));

	/* second wrong frame - but not consecutive */
	memset(bits, 1, sizeof(bits));
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNED");

	/* third wrong frame - second consecutive */
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNED");

	/* only from third consecutive invalid frame onwards we should loose alignment */
	osmo_trau_sync_rx_ubits(fi, bits, sizeof(bits));
	ASSERT_STATE(fi, "FRAME_ALIGNMENT_LOST");
}


static const struct log_info_cat default_categories[] = {
};

const struct log_info log_info = {
	.cat = default_categories,
	.num_cat = ARRAY_SIZE(default_categories),
};

int main(int argc, char **argv)
{
	osmo_init_logging2(NULL, NULL);
	log_set_use_color(osmo_stderr_target, 0);
	osmo_fsm_log_addr(false);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category(osmo_stderr_target, 0);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_log_level(osmo_stderr_target, LOGL_INFO);
	test_body();
}

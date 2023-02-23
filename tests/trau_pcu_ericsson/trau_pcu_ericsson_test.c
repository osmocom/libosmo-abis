
#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/trau/trau_pcu_ericsson.h>

void dump_frame(ubit_t *trau_frame, size_t len)
{
	uint16_t i;
	printf(" ");
	for (i = 0; i < len; i++) {
		if (i % 8 == 0 && i > 0)
			printf("\n ");
		printf("%u ", trau_frame[i]);
	}

	printf("\n");
}

static void dump_ab(struct er_gprs_ab *ab)
{
	printf("  ab->ab_type=%u\n", ab->ab_type);
	printf("  ab->rxlev=%u\n", ab->rxlev);
	printf("  ab->acc_delay=%u\n", ab->acc_delay);
	printf("  ab->data=%04x\n", ab->data);

	if (ab->ab_type == 1) {
		printf("  ab->u.type_1.crc=%u\n", ab->u.type_1.crc);
		printf("  ab->u.type_1.burst_qual=%u\n", ab->u.type_1.burst_qual);
		printf("  ab->u.type_1.frame_qual=%u\n", ab->u.type_1.frame_qual);
	} else if (ab->ab_type == 2) {
		printf("  ab->u.type_2.abi=%u\n", ab->u.type_2.abi);
		printf("  ab->u.type_2.type=%u\n", ab->u.type_2.type);
	} else
		printf("  bad data\n");
}

void er_gprs_trau_frame_encode_16k_FT_SYNC_test(void)
{
	struct er_gprs_trau_frame frame;
	ubit_t trau_frame[ER_GPRS_TRAU_FRAME_LEN_16K];
	int rc;

	printf("\n==> %s\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_sync_ind.pseq = 1234;
	frame.u.pcu_sync_ind.tav = 0;
	frame.u.pcu_sync_ind.fn_ul = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.fn_dl = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.fn_ss = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.ls = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.ss = 0xFFFFFFFF;	/* optional, not included */
	frame.type = ER_GPRS_TRAU_FT_SYNC;

	rc = er_gprs_trau_frame_encode_16k(trau_frame, &frame);
	OSMO_ASSERT(rc == 320);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_16K);
}

void er_gprs_trau_frame_encode_16k_FT_DATA_test(void)
{
	struct er_gprs_trau_frame frame;
	ubit_t trau_frame[ER_GPRS_TRAU_FRAME_LEN_16K];
	int rc;

	uint8_t data_cs1[] = {
		0x07, 0x00, 0x05, 0x01, 0xc0, 0x05, 0x08, 0x02,
		0x01, 0x2a, 0x44, 0x00, 0xf1, 0x10, 0x23, 0x6e,
		0x00, 0x17, 0x16, 0x18, 0x05, 0xf4, 0xf5
	};

	uint8_t data_cs2[] = {
		0x0f, 0x00, 0x08, 0x69, 0x01, 0xc0, 0x09, 0x08,
		0x02, 0x01, 0x2a, 0x44, 0x00, 0xf1, 0x10, 0x23,
		0x6e, 0x00, 0x17, 0x16, 0x18, 0x05, 0xf4, 0xd9,
		0xa1, 0x82, 0xdd, 0xd6, 0xee, 0xaa, 0x2b, 0x2b,
		0x2b, 0x00
	};

	printf("\n==> %s (CS1)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS1;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x01;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs1, sizeof(data_cs1));
	rc = er_gprs_trau_frame_encode_16k(trau_frame, &frame);
	OSMO_ASSERT(rc == 320);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_16K);

	printf("\n==> %s (CS2)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS2;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x08;
	frame.u.pcu_data_ind.timing_offset = 0x80;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs2, sizeof(data_cs2));
	rc = er_gprs_trau_frame_encode_16k(trau_frame, &frame);
	OSMO_ASSERT(rc == 320);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_16K);
}

void er_gprs_trau_frame_decode_16k_FT_SYNC_test(void)
{
	int rc;
	struct er_gprs_trau_frame frame;

	/* TRAU frame as received from BTS when no PCU-SYNC-IND was sent yet.
	 * The PSEQ value is not yet available */
	const ubit_t bits_1[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 0, 1, 0, 1, 0, 0,
		1, 1, 1, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 0, 1, 0, 1, 1, 1,
		1, 1, 0, 0, 0, 0, 1, 1,
		0, 1, 1, 1, 1, 0, 0, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 0, 1, 0, 1, 1, 1,
		1, 1, 0, 0, 0, 0, 1, 1,
		0, 1, 1, 1, 0, 0, 1, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	/* TRAU frame as received from BTS after a few PCU-SYNC-IND frames
	 * were sent. The PSEQ value is now known to the BTS. */
	const ubit_t bits_2[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 0, 1, 0, 1, 1, 0,
		0, 1, 1, 0, 1, 1, 1, 1,
		1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 0, 1,
		1, 1, 1, 0, 1, 0, 0, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 0, 1, 1, 1, 0,
		1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 1, 0, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 0, 1, 1, 1, 0,
		1, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 1, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	printf("\n==> %s (not yet synced)\n", __func__);
	rc = er_gprs_trau_frame_decode_16k(&frame, bits_1);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_SYNC);
	printf(" ccu_sync_ind.tav=%02x\n", frame.u.ccu_sync_ind.tav);
	printf(" ccu_sync_ind.dfe=%02x\n", frame.u.ccu_sync_ind.dfe);
	printf(" ccu_sync_ind.dbe=%02x\n", frame.u.ccu_sync_ind.dbe);
	printf(" ccu_sync_ind.pseq=%u\n", frame.u.ccu_sync_ind.pseq);
	printf(" ccu_sync_ind.afn_ul=%u\n", frame.u.ccu_sync_ind.afn_ul);
	printf(" ccu_sync_ind.afn_dl=%u\n", frame.u.ccu_sync_ind.afn_dl);

	printf("\n==> %s (synced)\n", __func__);
	rc = er_gprs_trau_frame_decode_16k(&frame, bits_2);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_SYNC);
	printf(" ccu_sync_ind.tav=%02x\n", frame.u.ccu_sync_ind.tav);
	printf(" ccu_sync_ind.dfe=%02x\n", frame.u.ccu_sync_ind.dfe);
	printf(" ccu_sync_ind.dbe=%02x\n", frame.u.ccu_sync_ind.dbe);
	printf(" ccu_sync_ind.pseq=%u\n", frame.u.ccu_sync_ind.pseq);
	printf(" ccu_sync_ind.afn_ul=%u\n", frame.u.ccu_sync_ind.afn_ul);
	printf(" ccu_sync_ind.afn_dl=%u\n", frame.u.ccu_sync_ind.afn_dl);
}

void er_gprs_trau_frame_decode_16k_FT_DATA_test(void)
{
	int rc;
	struct er_gprs_trau_frame frame;

	/* TRAU frame with valid CS1 block and correct CRC. */
	const ubit_t bits_cs1[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 1, 1, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 1, 0, 1,
		0, 1, 0, 0, 0, 0, 1, 0,
		0, 0, 1, 1, 1, 0, 0, 0,
		1, 1, 1, 1, 0, 1, 0, 0,
		1, 1, 0, 0, 0, 1, 0, 1,
		1, 0, 0, 1, 1, 1, 0, 1,
		0, 0, 0, 1, 0, 0, 1, 0,
		0, 1, 0, 1, 0, 1, 0, 1,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 1, 1,
		0, 1, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 1, 1, 0, 0, 0,
		1, 0, 1, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 1, 1,
		0, 1, 1, 0, 1, 0, 1, 1,
		0, 1, 0, 1, 0, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	/* TRAU frame with valid CS2 block. */
	const ubit_t bits_cs2[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 0, 0,
		0, 1, 0, 0, 1, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1,
		0, 0, 0, 0, 0, 1, 0, 0,
		1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 1, 1, 1, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		1, 1, 1, 0, 0, 1, 1, 1,
		0, 0, 1, 1, 0, 1, 1, 1,
		1, 0, 1, 1, 0, 1, 1, 1,
		1, 0, 1, 1, 0, 1, 1, 1,
		0, 0, 1, 1, 0, 0, 0, 1,
		1, 0, 1, 1, 0, 1, 0, 1,
		0, 0, 1, 1, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 1, 1, 0, 1, 1, 1,
		1, 0, 1, 1, 0, 1, 0, 1,
		1, 0, 1, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		1, 1, 0, 0, 1, 1, 0, 0,
		1, 1, 1, 0, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 1, 1, 1,
	};

	/* TRAU frame with AB data (noise). */
	const ubit_t bits_ab[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 0, 0,
		0, 1, 1, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 1, 0, 1, 1, 1,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 1, 1, 1, 0, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 1, 1, 0, 0, 0,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 1, 1, 0, 0, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 1, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	printf("\n==> %s (CS1)\n", __func__);
	rc = er_gprs_trau_frame_decode_16k(&frame, bits_cs1);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));

	printf("\n==> %s (CS2)\n", __func__);
	rc = er_gprs_trau_frame_decode_16k(&frame, bits_cs2);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));

	printf("\n==> %s (AB)\n", __func__);
	rc = er_gprs_trau_frame_decode_16k(&frame, bits_ab);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));
	printf(" ccu_data_ind.ab[0]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[0]);
	printf(" ccu_data_ind.ab[1]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[1]);
	printf(" ccu_data_ind.ab[2]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[2]);
	printf(" ccu_data_ind.ab[3]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[3]);
}

void er_gprs_trau_frame_encode_64k_FT_SYNC_test(void)
{
	struct er_gprs_trau_frame frame;
	ubit_t trau_frame[ER_GPRS_TRAU_FRAME_LEN_64K];
	int rc;

	printf("\n==> %s\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_sync_ind.pseq = 1234;
	frame.u.pcu_sync_ind.tav = 0;
	frame.u.pcu_sync_ind.fn_ul = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.fn_dl = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.fn_ss = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.ls = 0xFFFFFFFF;	/* optional, not included */
	frame.u.pcu_sync_ind.ss = 0xFFFFFFFF;	/* optional, not included */
	frame.type = ER_GPRS_TRAU_FT_SYNC;

	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);
}

void er_gprs_trau_frame_encode_64k_FT_DATA_test(void)
{
	struct er_gprs_trau_frame frame;
	ubit_t trau_frame[ER_GPRS_TRAU_FRAME_LEN_64K];
	int rc;

	uint8_t data_cs1[] = {
		0x07, 0x00, 0x05, 0x01, 0xc0, 0x05, 0x08, 0x02,
		0x01, 0x2a, 0x44, 0x00, 0xf1, 0x10, 0x23, 0x6e,
		0x00, 0x17, 0x16, 0x18, 0x05, 0xf4, 0xf5
	};

	uint8_t data_cs2[] = {
		0x0f, 0x00, 0x08, 0x69, 0x01, 0xc0, 0x09, 0x08,
		0x02, 0x01, 0x2a, 0x44, 0x00, 0xf1, 0x10, 0x23,
		0x6e, 0x00, 0x17, 0x16, 0x18, 0x05, 0xf4, 0xd9,
		0xa1, 0x82, 0xdd, 0xd6, 0xee, 0xaa, 0x2b, 0x2b,
		0x2b, 0x00
	};

	uint8_t data_cs3[] = {
		0x07, 0x00, 0x01, 0x01, 0xc0, 0x0d, 0x8a, 0x42,
		0x03, 0x0e, 0x23, 0x62, 0x1f, 0x72, 0x99, 0x3f,
		0x3f, 0x11, 0x43, 0xff, 0xff, 0x00, 0x00, 0x00,
		0x04, 0x2b, 0x06, 0x01, 0x21, 0xc0, 0xa8, 0x00,
		0x05, 0x27, 0x22, 0x80, 0x80, 0x21, 0x10, 0x00
	};

	uint8_t data_cs4[] = {
		0x07, 0x00, 0x01, 0x01, 0xc0, 0x09, 0x8a, 0x42,
		0x03, 0x0e, 0x23, 0x62, 0x1f, 0x72, 0x99, 0x3f,
		0x3f, 0x11, 0x43, 0xff, 0xff, 0x00, 0x00, 0x00,
		0x04, 0x2b, 0x06, 0x01, 0x21, 0xc0, 0xa8, 0x00,
		0x05, 0x27, 0x22, 0x80, 0x80, 0x21, 0x10, 0x02,
		0x00, 0x00, 0x10, 0x81, 0x06, 0x08, 0x08, 0x08,
		0x08, 0x83, 0x06, 0x08, 0x08, 0x00
	};

	uint8_t data_mcs1[] = {
		0x07, 0x80, 0x01, 0x18, 0x2c, 0xfe, 0x07, 0xf6,
		0x35, 0x0a, 0xe0, 0x2d, 0x0a, 0xe0, 0x63, 0x31,
		0xa3, 0x57, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
		0x56, 0x56, 0x00
	};

	uint8_t data_mcs2[] = {
		0x07, 0x80, 0x00, 0x12, 0x36, 0x10, 0x10, 0x10,
		0x00, 0x1a, 0x08, 0x10, 0x10, 0x10, 0x10, 0x2c,
		0xbd, 0xb2, 0x87, 0xf6, 0x03, 0x00, 0x2c, 0x02,
		0xe8, 0x35, 0x0a, 0xbe, 0x59, 0xc1, 0x00, 0x02,
		0x00
	};

	uint8_t data_mcs3[] = {
		0x0f, 0x80, 0x00, 0x06, 0x68, 0xfe, 0x03, 0x80,
		0x0b, 0x10, 0x04, 0x02, 0x54, 0x88, 0x00, 0xe2,
		0x21, 0x46, 0xdc, 0x00, 0x2e, 0x2c, 0x30, 0x0a,
		0xe8, 0xd7, 0x6f, 0x4e, 0x55, 0xbd, 0xb0, 0xfe,
		0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
		0x56, 0x00
	};

	uint8_t data_mcs4[] = {
		0x07, 0x80, 0x00, 0x04, 0x68, 0xfe, 0x03, 0x80,
		0x0b, 0x10, 0x04, 0x02, 0x54, 0x88, 0x00, 0xe2,
		0x21, 0x46, 0xdc, 0x00, 0x2e, 0x2c, 0x30, 0x0a,
		0xe8, 0xfd, 0xe1, 0xb7, 0x79, 0x6c, 0xe7, 0xd8,
		0x57, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
		0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56,
		0x00
	};

	uint8_t data_mcs5[] = {
		0x07, 0x40, 0x00, 0x88, 0x86, 0xd3, 0x3f, 0x02,
		0x02, 0x02, 0x40, 0x03, 0x01, 0x02, 0x02, 0x02,
		0x82, 0x07, 0x33, 0xf2, 0xd0, 0x7e, 0x00, 0x80,
		0x45, 0x00, 0xbd, 0x46, 0xc1, 0x37, 0x2b, 0x18,
		0x40, 0x00, 0x40, 0x80, 0x42, 0x20, 0xc0, 0x01,
		0x04, 0x00, 0xc8, 0x80, 0x00, 0x00, 0x85, 0xc0,
		0x01, 0x20, 0x00, 0x81, 0x04, 0x00, 0x48, 0x41,
		0xa2, 0xbb, 0xcc, 0x0a
	};

	uint8_t data_mcs6[] = {
		0x07, 0x00, 0x00, 0xc2, 0x62, 0x00, 0x70, 0x82,
		0xa2, 0xd0, 0x80, 0xc3, 0x88, 0xd8, 0x87, 0x5c,
		0xe6, 0xcf, 0x4f, 0xc4, 0xd0, 0xff, 0x3f, 0x00,
		0x00, 0x00, 0xc1, 0x8a, 0x41, 0x40, 0x08, 0x30,
		0x2a, 0x40, 0xc1, 0x89, 0x08, 0x20, 0x60, 0x08,
		0x84, 0x00, 0x00, 0x00, 0x44, 0xa0, 0x01, 0x02,
		0x02, 0x02, 0xc2, 0xa0, 0x01, 0x02, 0x02, 0x02,
		0x02, 0x40, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02,
		0x40, 0x03, 0x01, 0x02, 0x02, 0x02, 0xc2, 0x07,
		0x33, 0xf2, 0xd0, 0x7e, 0x00, 0x00
	};

	uint8_t data_mcs7[] = {
		0x0f, 0x00, 0x00, 0x02, 0xa0, 0x05, 0x00, 0x27,
		0x28, 0x0a, 0x0d, 0x38, 0x8c, 0x88, 0x7d, 0xc8,
		0x65, 0xfe, 0xfc, 0x44, 0x0c, 0xfd, 0xff, 0x03,
		0x00, 0x00, 0x10, 0xac, 0x18, 0x04, 0x84, 0x00,
		0xa3, 0x02, 0x14, 0x9c, 0x88, 0x00, 0x02, 0x86,
		0x40, 0x08, 0x00, 0x00, 0x40, 0x04, 0x1a, 0x20,
		0x20, 0x20, 0x20, 0x0c, 0x1a, 0x20, 0x20, 0x20,
		0x20, 0x00, 0x34, 0x10, 0x20, 0xa0, 0xe1, 0xf4,
		0x8f, 0x80, 0x80, 0x00, 0xd0, 0x40, 0x80, 0x80,
		0x80, 0x80, 0xf0, 0xc1, 0x8c, 0x3c, 0xb4, 0x1f,
		0x00, 0x60, 0x11, 0x40, 0xaf, 0x51, 0xf0, 0xcd,
		0x0a, 0x06, 0x10, 0x00, 0x10, 0xa0, 0x10, 0x08,
		0x70, 0x00, 0x01, 0x00, 0x32, 0x20, 0x00, 0x40,
		0x21, 0x70, 0x00, 0x08, 0x40, 0x20, 0x01, 0x00,
		0x52, 0x90, 0xe8, 0x2e, 0xb3, 0x02
	};

	uint8_t data_mcs8[] = {
		0x07, 0x00, 0x00, 0x02, 0x58, 0x05, 0x00, 0x27,
		0x28, 0x0a, 0x0d, 0x38, 0x8c, 0x88, 0x7d, 0xc8,
		0x65, 0xfe, 0xfc, 0x44, 0x0c, 0xfd, 0xff, 0x03,
		0x00, 0x00, 0x10, 0xac, 0x18, 0x04, 0x84, 0x00,
		0xa3, 0x02, 0x14, 0x9c, 0x88, 0x00, 0x02, 0x86,
		0x40, 0x08, 0x00, 0x00, 0x40, 0x04, 0x1a, 0x20,
		0x20, 0x20, 0x20, 0x0c, 0x1a, 0x20, 0x20, 0x20,
		0x20, 0x00, 0x34, 0x10, 0x20, 0x20, 0x20, 0x20,
		0x00, 0x34, 0x10, 0x20, 0x20, 0x20, 0x20, 0x7c,
		0x30, 0x23, 0xe0, 0xf4, 0x8f, 0x3c, 0xb4, 0x1f,
		0x00, 0x60, 0x11, 0x40, 0xaf, 0x51, 0xf0, 0xcd,
		0x0a, 0x06, 0x10, 0x00, 0x10, 0xa0, 0x10, 0x08,
		0x70, 0x00, 0x01, 0x00, 0x32, 0x20, 0x00, 0x40,
		0x21, 0x70, 0x00, 0x08, 0x40, 0x20, 0x01, 0x00,
		0x52, 0x90, 0xe8, 0x2e, 0xb3, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0x02
	};

	uint8_t data_mcs9[] = {
		0x07, 0x00, 0x00, 0x02, 0x28, 0x2c, 0x06, 0x00,
		0x27, 0x28, 0x0a, 0x0d, 0x38, 0x8c, 0x88, 0x7d,
		0xc8, 0x65, 0xfe, 0xfc, 0x44, 0x0c, 0xfd, 0xff,
		0x03, 0x00, 0x00, 0x10, 0xac, 0x18, 0x04, 0x84,
		0x00, 0xa3, 0x02, 0x14, 0x9c, 0x88, 0x00, 0x02,
		0x86, 0x40, 0x08, 0x00, 0x00, 0x40, 0x04, 0x1a,
		0x20, 0x20, 0x20, 0x20, 0x0c, 0x1a, 0x20, 0x20,
		0x20, 0x20, 0x00, 0x34, 0x10, 0x20, 0x20, 0x20,
		0x20, 0x00, 0x34, 0x10, 0x20, 0x20, 0x20, 0x20,
		0x7c, 0x30, 0x23, 0x0f, 0xed, 0x07, 0x00, 0x60,
		0xf4, 0x6f, 0x11, 0x40, 0xaf, 0x51, 0xf0, 0xcd,
		0x0a, 0x06, 0x10, 0x00, 0x10, 0xa0, 0x10, 0x08,
		0x70, 0x00, 0x01, 0x00, 0x32, 0x20, 0x00, 0x40,
		0x21, 0x70, 0x00, 0x08, 0x40, 0x20, 0x01, 0x00,
		0x52, 0x90, 0xe8, 0x2e, 0xb3, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2, 0xb2,
		0xb2, 0x02
	};

	printf("\n==> %s (CS1)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS1;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs1, sizeof(data_cs1));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (CS2)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS2;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs2, sizeof(data_cs2));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (CS3)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS3;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs3, sizeof(data_cs3));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (CS4)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_CS4;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_cs4, sizeof(data_cs4));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS1)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR3;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs1, sizeof(data_mcs1));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS2)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR3;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs2, sizeof(data_mcs2));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS3)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR3;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs3, sizeof(data_mcs3));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS4)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR3;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs4, sizeof(data_mcs4));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS5)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR2;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs5, sizeof(data_mcs5));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS6)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR2;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs6, sizeof(data_mcs6));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS7)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR1;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs7, sizeof(data_mcs7));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS8)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR1;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs8, sizeof(data_mcs8));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

	printf("\n==> %s (MCS9)\n", __func__);
	memset(&frame, 0, sizeof(frame));
	frame.u.pcu_data_ind.tav = 0;
	frame.u.pcu_data_ind.ul_frame_err = false;
	frame.u.pcu_data_ind.cs_hdr = CS_OR_HDR_HDR1;
	frame.u.pcu_data_ind.ul_chan_mode = ER_UL_CHMOD_NB_GMSK;
	frame.u.pcu_data_ind.atten_db = 0x0f;
	frame.u.pcu_data_ind.timing_offset = 0x01;
	frame.type = ER_GPRS_TRAU_FT_DATA;
	memcpy(frame.u.pcu_data_ind.data, data_mcs9, sizeof(data_mcs9));
	rc = er_gprs_trau_frame_encode_64k(trau_frame, &frame);
	OSMO_ASSERT(rc == 1280);
	dump_frame(trau_frame, ER_GPRS_TRAU_FRAME_LEN_64K);

}

void er_gprs_trau_frame_decode_64k_FT_SYNC_test(void)
{
	int rc;
	struct er_gprs_trau_frame frame;

	/* TRAU frame as received from BTS when no PCU-SYNC-IND was sent yet.
	 * The PSEQ value is not yet available */
	const ubit_t bits_1[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 0, 1, 0, 1, 0, 0,
		1, 1, 1, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 1, 0, 1, 0, 0,
		1, 1, 1, 1, 0, 0, 0, 0,
		0, 1, 1, 0, 1, 0, 0, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 1, 0, 1, 0, 0,
		1, 1, 1, 1, 0, 0, 0, 0,
		0, 1, 1, 0, 0, 0, 1, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	/* TRAU frame as received from BTS after a few PCU-SYNC-IND frames
	 * were sent. The PSEQ value is now known to the BTS. */
	const ubit_t bits_2[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 0, 1, 0, 1, 0, 0,
		0, 0, 1, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1,
		1, 0, 0, 0, 0, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 0, 1, 0, 0, 1,
		0, 0, 1, 0, 1, 0, 1, 1,
		1, 0, 1, 0, 1, 1, 1, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 0, 1, 0, 0, 1,
		0, 0, 1, 0, 1, 0, 1, 1,
		1, 0, 1, 0, 0, 1, 1, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
	};

	printf("\n==> %s (not yet synced)\n", __func__);
	rc = er_gprs_trau_frame_decode_64k(&frame, bits_1);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_SYNC);
	printf(" ccu_sync_ind.tav=%02x\n", frame.u.ccu_sync_ind.tav);
	printf(" ccu_sync_ind.dfe=%02x\n", frame.u.ccu_sync_ind.dfe);
	printf(" ccu_sync_ind.dbe=%02x\n", frame.u.ccu_sync_ind.dbe);
	printf(" ccu_sync_ind.pseq=%u\n", frame.u.ccu_sync_ind.pseq);
	printf(" ccu_sync_ind.afn_ul=%u\n", frame.u.ccu_sync_ind.afn_ul);
	printf(" ccu_sync_ind.afn_dl=%u\n", frame.u.ccu_sync_ind.afn_dl);

	printf("\n==> %s (synced)\n", __func__);
	rc = er_gprs_trau_frame_decode_64k(&frame, bits_2);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_SYNC);
	printf(" ccu_sync_ind.tav=%02x\n", frame.u.ccu_sync_ind.tav);
	printf(" ccu_sync_ind.dfe=%02x\n", frame.u.ccu_sync_ind.dfe);
	printf(" ccu_sync_ind.dbe=%02x\n", frame.u.ccu_sync_ind.dbe);
	printf(" ccu_sync_ind.pseq=%u\n", frame.u.ccu_sync_ind.pseq);
	printf(" ccu_sync_ind.afn_ul=%u\n", frame.u.ccu_sync_ind.afn_ul);
	printf(" ccu_sync_ind.afn_dl=%u\n", frame.u.ccu_sync_ind.afn_dl);
}

void er_gprs_trau_frame_decode_64k_FT_DATA_test(void)
{
	int rc;
	struct er_gprs_trau_frame frame;

	/* TRAU frame with valid CS1 block and correct CRC */
	const ubit_t bits_cs1[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 0, 0,
		0, 1, 0, 0, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 1, 1, 1, 0, 0,
		0, 0, 0, 0, 1, 0, 1, 1,
		1, 0, 0, 0, 0, 0, 0, 1,
		0, 0, 0, 0, 1, 0, 1, 0,
		1, 0, 1, 1, 1, 0, 1, 0,
		0, 0, 1, 1, 1, 0, 1, 1,
		0, 1, 1, 1, 1, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 0, 0,
		0, 1, 1, 1, 1, 1, 0, 1,
		0, 1, 1, 0, 1, 0, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		/* [...] shortened since decode won't access filler bits after
		 * the TRAU frame. */
	};

	/* TRAU frame with valid CS2 block */
	const ubit_t bits_cs2[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 0, 1,
		1, 1, 0, 1, 0, 1, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 0, 1, 1, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 1, 0,
		0, 1, 0, 0, 1, 0, 0, 0,
		0, 0, 1, 0, 1, 1, 1, 0,
		1, 1, 1, 0, 0, 1, 0, 0,
		1, 1, 1, 0, 1, 0, 0, 1,
		1, 1, 1, 0, 1, 0, 1, 1,
		1, 0, 0, 1, 1, 0, 1, 1,
		1, 1, 0, 1, 1, 0, 1, 1,
		1, 1, 0, 1, 1, 0, 1, 1,
		1, 0, 0, 1, 1, 0, 0, 0,
		1, 1, 0, 1, 1, 0, 1, 0,
		1, 0, 0, 1, 1, 0, 1, 1,
		0, 0, 0, 0, 0, 0, 1, 1,
		0, 0, 0, 1, 1, 0, 1, 1,
		1, 1, 0, 1, 1, 0, 1, 0,
		1, 1, 0, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 1, 0, 0,
		1, 1, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 1, 1,
		0, 1, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		/* [...] shortened since decode won't access filler bits after
		 * the TRAU frame. */
	};

	/* TRAU frame with AB data (noise). */
	const ubit_t bits_ab[] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 1, 1, 1, 0,
		1, 1, 0, 0, 0, 1, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 1, 1, 1, 0, 1, 0,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 0, 1, 0, 0, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 0, 0,
		1, 1, 1, 1, 1, 1, 0, 0,
		0, 0, 1, 1, 1, 0, 1, 1,
		1, 1, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		/* [...] shortened since decode won't access filler bits after
		 * the TRAU frame. */
	};

	printf("\n==> %s (CS1)\n", __func__);
	rc = er_gprs_trau_frame_decode_64k(&frame, bits_cs1);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));

	printf("\n==> %s (CS2)\n", __func__);
	rc = er_gprs_trau_frame_decode_64k(&frame, bits_cs2);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));

	printf("\n==> %s (AB)\n", __func__);
	rc = er_gprs_trau_frame_decode_64k(&frame, bits_ab);
	OSMO_ASSERT(rc == 0);
	OSMO_ASSERT(frame.type == ER_GPRS_TRAU_FT_DATA);
	printf(" ccu_data_ind.tav=%02x\n", frame.u.ccu_data_ind.tav);
	printf(" ccu_data_ind.dbe=%u\n", frame.u.ccu_data_ind.dbe);
	printf(" ccu_data_ind.cs_hdr=%u\n", frame.u.ccu_data_ind.cs_hdr);
	printf(" ccu_data_ind.gprs.rx_lev=%u\n", frame.u.ccu_data_ind.rx_lev);
	printf(" ccu_data_ind.gprs.est_acc_del_dev=%d\n", frame.u.ccu_data_ind.est_acc_del_dev);
	printf(" ccu_data_ind.u.gprs.block_qual=%u\n", frame.u.ccu_data_ind.u.gprs.block_qual);
	printf(" ccu_data_ind.u.gprs.parity_ok=%u\n", frame.u.ccu_data_ind.u.gprs.parity_ok);
	printf(" ccu_data_ind.u.data_len=%u\n", frame.u.ccu_data_ind.data_len);
	printf(" ccu_data_ind.data=%s\n", osmo_hexdump_nospc(frame.u.ccu_data_ind.data, frame.u.ccu_data_ind.data_len));
	printf(" ccu_data_ind.ab[0]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[0]);
	printf(" ccu_data_ind.ab[1]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[1]);
	printf(" ccu_data_ind.ab[2]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[2]);
	printf(" ccu_data_ind.ab[3]=\n");
	dump_ab(&frame.u.ccu_data_ind.ab[3]);
}

static void test_body(void)
{
	er_gprs_trau_frame_encode_16k_FT_SYNC_test();
	er_gprs_trau_frame_encode_16k_FT_DATA_test();
	er_gprs_trau_frame_decode_16k_FT_SYNC_test();
	er_gprs_trau_frame_decode_16k_FT_DATA_test();

	er_gprs_trau_frame_encode_64k_FT_SYNC_test();
	er_gprs_trau_frame_encode_64k_FT_DATA_test();
	er_gprs_trau_frame_decode_64k_FT_SYNC_test();
	er_gprs_trau_frame_decode_64k_FT_DATA_test();
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
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_NONE);
	log_set_print_category(osmo_stderr_target, 0);
	log_set_print_category_hex(osmo_stderr_target, 0);
	log_set_log_level(osmo_stderr_target, LOGL_INFO);
	test_body();
}

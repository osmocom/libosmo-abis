#pragma once

enum time_adj_val {
	TIME_ADJ_NONE,
	TIME_ADJ_DELAY_250us,
	TIME_ADJ_ADVANCE_250us,
};

enum er_cs_or_hdr {
	CS_OR_HDR_AB = 0,
	CS_OR_HDR_CS1 = 1,
	CS_OR_HDR_CS2 = 2,
	CS_OR_HDR_CS3 = 3,
	CS_OR_HDR_CS4 = 4,
	CS_OR_HDR_HDR1 = 5,
	CS_OR_HDR_HDR2 = 6,
	CS_OR_HDR_HDR3 = 7,
};

/* Block Quality / Soft Frame Quality */
enum er_block_qual {
	ER_SFQ_00_09 = 0,
	ER_SFQ_10_19 = 1,
	ER_SFQ_20_29 = 2,
	ER_SFQ_30_39 = 3,
	ER_SFQ_40_49 = 4,
	ER_SFQ_50_59 = 5,
	ER_SFQ_60_69 = 6,
	ER_SFQ_70_MORE = 7,
};

enum er_ul_chan_mode {
	/* no demodulation, ignore uplink */
	ER_UL_CHMOD_VOID = 0,

	/* normal burst, GMSK only */
	ER_UL_CHMOD_NB_GMSK = 1,

	/* normal burst, MCS1..MCS9 or CS1 */
	ER_UL_CHMOD_NB_UNKN = 2,

	/* Access Burst (only TS0 and 8bit) */
	ER_UL_CHMOD_AB = 3,

	/* Access Burst (TS0/1/2) */
	ER_UL_CHMOD_AB_UNKN = 4,
};

/* Access burst data */
struct er_gprs_ab {
	uint8_t ab_type;
	uint8_t rxlev;
	uint16_t acc_delay;
	uint16_t data;

	union {
		struct {
			uint8_t crc;
			uint8_t burst_qual;
			uint8_t frame_qual;
		} type_1;
		struct {
			uint8_t abi;
			uint8_t type;
		} type_2;
	} u;
};

/* CCU->PCU SYNC */
struct er_ccu_sync_ind {
	/* Time Adjustment Value (requested) */
	enum time_adj_val tav;

	/* Downlink Frame Error */
	bool dfe;

	/* Downlink Block Error (bad MAC block from PCU) */
	bool dbe;

	/* PCU Sequene Counter (will lag, must be compensated) */
	uint32_t pseq;

	/* Adjusted Frame Number, Uplink */
	uint32_t afn_ul;

	/* Adjusted Frame Number, Downlink */
	uint32_t afn_dl;
};

/* PCU->CCU SYNC */
struct er_pcu_sync_ind {
	/* Time Adjustment Value (acknowledged) */
	enum time_adj_val tav;
	bool ul_frame_err;

	/* PCU Sequene Counter (PCU will sync to this value) */
	uint32_t pseq;

	/* Optional, ignored by CCU */
	uint32_t ss;

	/* Optional, ignored by CCU */
	uint32_t fn_ul;

	/* Optional, ignored by CCU */
	uint32_t fn_ss;

	/* Optional, ignored by CCU */
	uint32_t fn_dl;

	/* Optional, ignored by CCU */
	uint32_t ls;
};

/* PCU->CCU DATA */
struct er_pcu_data_ind {
	/* Time Adjustment Value (acknowledged) */
	enum time_adj_val tav;

	/* Uplink Frame Error */
	bool ul_frame_err;

	/* Coding Scheme */
	enum er_cs_or_hdr cs_hdr;

	/* Modulation to be applied on coresponding uplink frame, also used to retrieve
	 * extraordinary ccu_sync_ind frames and ccu_data_ind frames wth Access bursts data. */
	enum er_ul_chan_mode ul_chan_mode;

	/* Attenuation of transmission power, 16 steps, 2dB each */
	uint8_t atten_db;

	/* Timing offset (timing advance) */
	uint8_t timing_offset;

	/* MAC block */
	uint8_t data[155];
};

/* CCU->PCU DATA */
struct er_ccu_data_ind {
	/* Time Adjustment Value (requested) */
	enum time_adj_val tav;

	/* Downlink Block Error (bad MAC block from PCU) */
	bool dbe;

	/* Coding Scheme */
	enum er_cs_or_hdr cs_hdr;

	/* Receive level 63 steps, see also GSM 05.08, section 8.1.4 */
	uint8_t rx_lev;

	/* Estimated Access Delay Deviation (bits, uplink only) */
	int8_t est_acc_del_dev;

	/* Data integrity checks (uplink only) */
	union {
		struct {
			/* Soft Frame Quality */
			enum er_block_qual block_qual;
			/* Parity Check */
			bool parity_ok;
		} gprs;
		struct {
			uint8_t mean_bep;
			uint8_t cv_bep;
			bool hdr_good;
			bool data_good[2];
		} egprs;
	} u;

	/* MAC block */
	uint8_t data[155];

	/* MAC block length
	 * (uplink only, for downlink cs_hdr and MAC block header will be used
	 * to determine the length.) */
	uint8_t data_len;

	/* Access burst data (in case an access block was received) */
	struct er_gprs_ab ab[4];
};

enum er_gprs_trau_frame_type {
	ER_GPRS_TRAU_FT_NONE,
	ER_GPRS_TRAU_FT_SYNC,
	ER_GPRS_TRAU_FT_DATA,
};

struct er_gprs_trau_frame {
	enum er_gprs_trau_frame_type type;
	union {
		struct er_ccu_sync_ind ccu_sync_ind;
		struct er_ccu_data_ind ccu_data_ind;
		struct er_pcu_sync_ind pcu_sync_ind;
		struct er_pcu_data_ind pcu_data_ind;
	} u;
};

#define ER_GPRS_TRAU_FRAME_LEN_16K 324	/* 320 +/-4 bit for time alignment */
#define ER_GPRS_TRAU_FRAME_LEN_64K 1296	/* 1280 +/-16 bit for time alignment */
int er_gprs_trau_frame_encode_16k(ubit_t *bits, struct er_gprs_trau_frame *fr);
int er_gprs_trau_frame_decode_16k(struct er_gprs_trau_frame *fr, const ubit_t *bits);
int er_gprs_trau_frame_encode_64k(ubit_t *bits, struct er_gprs_trau_frame *fr);
int er_gprs_trau_frame_decode_64k(struct er_gprs_trau_frame *fr, const ubit_t *bits);

/*
 * Idle pattern
 * ============
 * An inactive CCU will send an idle pattern, which is a sequence of alternating ones and zeros (101010101...) The
 * idle pattern indicates that the CCU available but in idle state. To leave the idle state a channel activation
 * via RSL must be carried out. It is not possible to "wake up" an idle CCU by just sending TRAU frames to it.
 *
 *
 * Synchronization procedure
 * =========================
 * When the PDCH is activated, the CCU becomes active at its designated E1 timeslot/subslot. It sends out ccu_sync_ind
 * frames and expects pcu_sync_ind frames from the PCU. When the CCU does not receive any pcu_sync_ind frames, the CCU
 * becomes inactive until the PDCH is activated again.
 *
 * CCU is synchronized to the PCU by sending pcu_sync_ind frames with an incrementing PSEQ sequence number to the CCU.
 * The PSEQ has no relation to the actual frame number. It just counts the TRAU frames sent over the E1 link. When
 * the CCU receives a valid pcu_sync_ind it will synchronize its local PSEQ counter to the received PSEQ and respond
 * back with a ccu_sync_ind that echos the PSEQ and also contains the current GSM frame numbers. The synchronization
 * is then complete and the PCU may start sending pcu_data_ind.
 *
 *  PCU                           CCU
 *   |                             |
 *   |--------pcu_sync_ind-------->|
 *   |<-------idle pattern---------|
 *   |--------pcu_sync_ind-------->|
 *   |<-------idle pattern---------|
 *   |--------pcu_sync_ind-------->|
 *   |<-------idle pattern---------|
 *   |--------pcu_sync_ind-------->|
 *   |<-------ccu_sync_ind---------|
 *   |--------pcu_sync_ind-------->|
 *   |<-------ccu_sync_ind---------|
 *   |--------pcu_sync_ind-------->|
 *   |<-------ccu_sync_ind---------|
 *   |--------pcu_data_ind-------->|
 *   |<-------ccu_data_ind---------|
 *   |--------pcu_data_ind-------->|
 *   |<-------ccu_data_ind---------|
 *
 * While pcu_data_ind and ccu_data_ind frames are passed over the link the PCU has no way to get the current PSEQ or
 * any GSM frame number from the CCU. It must calculate those values locally. However, it is possible to request
 * a ccu_sync_ind from the CCU at any time by setting the requested demodulation to ER_UL_CHMOD_VOID in one
 * pcu_data_ind. It should be pointed out that the requested ccu_sync_ind will steal one pcu_data_ind. The requested
 * ccu_sync_ind message will also only contain the GSM frame numbers but not the PSEQ value. If the GSM frame numbers
 * in the received ccu_sync_ind are deviating from the local frame numbers, the synchronization procedure can be
 * carried out again.
 *
 *  PCU                           CCU
 *   |                             |
 *   |--------pcu_data_ind-------->|
 *   |<-------ccu_data_ind---------|
 *   |--------pcu_data_ind-------->|
 *   |<-------ccu_data_ind---------|
 *   |--------pcu_data_ind-------->| (ER_UL_CHMOD_VOID)
 *   |<-------ccu_sync_ind---------|
 *   |--------pcu_data_ind-------->|
 *   |<-------ccu_data_ind---------|
 *
 * It also should be mentioned that due to link latency, there will naturally be a gap between the PCU local PSEQ and
 * the PSEQ reported by the CCU. This information is important since it must be used to compensate the GSM frame
 * numbers.
 *
 * The procedure is very similar to what is described in US Patent No. 5,978,368 from Nov.2, 1999
 *
 * Usage of Timing Adjustment (TAV) parameter
 * ==========================================
 * The time adjustment is controlled by the CCU. The TAV value received in TRAU frames that originate from the CCU is
 * the ordered timing adjustment. The TAV value in TRAU frames that originate from the PCU is the acknowledged time
 * adjustment. When a timing adjustment is requested by the CCU, the PCU acknowledges it by sending the requested
 * value back. The timing adjustment is performed immediately within the same frame by appending or removing bits
 * at the end. This delays or advances the timing of the subsequent frame.
 *
 * The procedure is very similar to the one described in: GSM 08.60, section 4.6.1.2
 *
 *
 * Usable coding schemes in uplink
 * ===============================
 * To understand which coding schemes are usable one must understand that the first CCU implementation for Ericsson
 * RBS only supported CS1 and CS2 on 16K I.460 subslots. CS1 and CS2 were not supported due to the bandwidth limitation
 * of the 16k subslots. In order to support higher bandwidth, in particular EDGE, a new CCU implementation was
 * introduced. This newer implementation uses full 64K E1 timeslots and has full EDGE support MCS1-MCS9.
 *
 * Interestingly though, the newer CCU implementation seems to lack support for CS3-CS4. At least it was not possible
 * To receive any CS3 or CS4 blocks from the CCU, while CS1 and CS2 work fine. In case the PCU is instructed to use
 * CS3 or CS4 in downlink nothing is received. Since the CCU also demodulates noise and even sends the demodulation
 * results back to the PCU there should have been at least some invalid CS3 and CS4 blocks in the demodulation results
 * from time to time but also here no CS3 or CS4 blocks were observed. This leads to the vague assumption that the GPRS
 * part of the CCU presumably never got updated and only EDGE support was added.
 *
 * It was also discovered that CS1 and CS2 only appear when the CCU is instructed to receive GMSK only
 * (ER_UL_CHMOD_NB_GMSK). When the CCU is instructed to receive GMSK and PSK (ER_UL_CHMOD_NB_UNKN) CS2 does not appear.
 * EGPRS related blocks (CS_OR_HDR_HDR1-3) appear in both situations. Since CS1 also plays a role in EGPRS it might be
 * that the CCU is filtering non EDGE related blocks when ER_UL_CHMOD_NB_UNKN is used.
 *
 * The observations above were made with an RRUS 02 B3
 */

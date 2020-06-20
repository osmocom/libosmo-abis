#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <dahdi/user.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/application.h>
#include <osmocom/gsm/i460_mux.h>
#include <osmocom/trau/trau_sync.h>
#include <osmocom/trau/trau_frame.h>
#include <osmocom/trau/trau_rtp.h>
#include <osmocom/trau/osmo_ortp.h>

#define D_BCHAN_TX_GRAN 160

/***********************************************************************
 * BEGIN CONFIGURATION
 ***********************************************************************/
/* HACK: Those do not have getopt but need to be modified in source code to match your enviroment */

/* do we just locally loop the calls from 1->2, or do we interface with RTP? */
static bool g_local_loop = false;

/* remote IP address to which we send RTP data (unless g_local_loop mode) */
const char *g_remote_host = "192.168.101.131";

/* remote UDP base port to which we send RTP data: +2 for every sub-slot */
const int g_remote_port = 9000;

/* local UDP base port on which we receive RTP data: +2 for every sub-slot */
const int g_local_port = 8000;

/* codec; can be OSMO_TRAU16_FT_FR or OSMO_TRAU16_FT_EFR */
const enum osmo_trau_frame_type g_ftype = OSMO_TRAU16_FT_EFR;

/***********************************************************************
 * END CONFIGURATION
 ***********************************************************************/

/* do we have a DAHDI chardev to which we can write RTP->TRAU conversion data/ */
static bool g_ts_is_writable = false;

struct sc_state {
	unsigned int num;
	struct osmo_i460_subchan *i460_sc;
	struct osmo_fsm_inst *sync_fi;
	struct osmo_rtp_socket *rtps;
	struct osmo_trau2rtp_state t2r;
};

struct state {
	int in_fd;
	struct osmo_i460_timeslot i460_ts;
	struct sc_state sc[4];
};
static struct state g_st;

#define LOGSC(sc, fmt, args...) printf("SC%u: " fmt, (sc)->num, ## args)

static struct sc_state *opposite_schan(struct sc_state *sc)
{
	/* we blindly assume that there is a call between sub-channel 1 + sub-channel 2 */
	switch (sc->num) {
	case 1:
		return &g_st.sc[2];
	case 2:
		return &g_st.sc[1];
	default:
		OSMO_ASSERT(0);
	}
}

/* called by I.460 de-multeiplexer; feed output of I.460 demux into TRAU frame sync */
static void i460_demux_bits_cb(struct osmo_i460_subchan *schan, void *user_data,
				const ubit_t *bits, unsigned int num_bits)
{
	struct sc_state *sc = user_data;
	//printf("I460: %s\n", osmo_ubit_dump(bits, num_bits));
	osmo_trau_sync_rx_ubits(sc->sync_fi, bits, num_bits);
}

/* called for each synchronized TRAU frame received; decode frame + convert to RTP */
static void sync_frame_out_cb(void *user_data, const ubit_t *bits, unsigned int num_bits)
{
	struct sc_state *sc = user_data;
	struct osmo_trau_frame fr;
	int rc;

	LOGSC(sc, "Rx TRAU: %s\n", osmo_ubit_dump(bits, num_bits));
	if (!bits)
		goto skip;

	rc = osmo_trau_frame_decode_16k(&fr, bits, OSMO_TRAU_DIR_UL);
	if (rc != 0)
		goto skip;

	uint8_t sid;
	switch (fr.type) {
	case OSMO_TRAU16_FT_FR:
	case OSMO_TRAU16_FT_EFR:
		sid = (fr.c_bits[13-1]) << 1 | (fr.c_bits[14-1] << 0);
		LOGSC(sc, "-> FT=%s, BFI=%u, SID=%u, TAF=%u DTXd=%u\n",
			osmo_trau_frame_type_name(fr.type), fr.c_bits[12-1], sid, fr.c_bits[15-1], fr.c_bits[17-1]);
		break;
	default:
		LOGSC(sc, "-> FT=%s\n", osmo_trau_frame_type_name(fr.type));
		break;
	}

	if (g_local_loop) {
		/* Mirror back to other sub-slot */
		struct sc_state *peer = opposite_schan(sc);
		if (peer) {
			struct msgb *msg = msgb_alloc(2*40*8, "mirror");
			fr.c_bits[12-1] = 1; /* C12 = good u-link frame */
			memset(&fr.c_bits[13-1], 1, 3); /* C13..C15: spare  */
			fr.c_bits[16-1] = 1; /* C16 = SP[eech]; no DTX */
			memset(&fr.c_bits[6-1], 0, 6); /* C6..C11: tie alignment */
			fr.dir = OSMO_TRAU_DIR_DL;
			rc = osmo_trau_frame_encode(msgb_data(msg), 2*40*8, &fr);
			OSMO_ASSERT(rc >= 0);
			msgb_put(msg, rc);
			osmo_i460_mux_enqueue(peer->i460_sc, msg);
		}
	} else {
		/* Convert to RTP */
		if (fr.type != OSMO_TRAU16_FT_FR && fr.type != OSMO_TRAU16_FT_EFR)
			goto skip;

		uint8_t rtpbuf[35];
		struct osmo_trau2rtp_state t2rs = {
			.type = fr.type,
		};
		memset(rtpbuf, 0, sizeof(rtpbuf));
		rc = osmo_trau2rtp(rtpbuf, sizeof(rtpbuf), &fr, &t2rs);
		LOGSC(sc, "Tx RTP: %s\n", osmo_hexdump(rtpbuf, rc));
		if (rc)
			osmo_rtp_send_frame_ext(sc->rtps, rtpbuf, rc, 160, false);
		else {
			osmo_rtp_skipped_frame(sc->rtps, 160);
		}
		return;
	}
skip:
	if (!g_local_loop)
		osmo_rtp_skipped_frame(sc->rtps, 160);
}

static int dahdi_set_bufinfo(int fd, int as_sigchan)
{
	struct dahdi_bufferinfo bi;
	int x = 0;

	if (ioctl(fd, DAHDI_GET_BUFINFO, &bi)) {
		LOGP(DLINP, LOGL_ERROR, "Error getting bufinfo\n");
		return -EIO;
	}

	if (as_sigchan) {
		bi.numbufs = 4;
		bi.bufsize = 512;
	} else {
		bi.numbufs = 8;
		bi.bufsize = D_BCHAN_TX_GRAN;
		bi.txbufpolicy = DAHDI_POLICY_WHEN_FULL;
	}

	if (ioctl(fd, DAHDI_SET_BUFINFO, &bi)) {
		fprintf(stderr, "Error setting DAHDI bufinfo\n");
		return -EIO;
	}

	if (!as_sigchan) {
		if (ioctl(fd, DAHDI_AUDIOMODE, &x)) {
			fprintf(stderr, "Error setting DAHDI bufinfo\n");
			return -EIO;
		}
	} else {
		int one = 1;
		ioctl(fd, DAHDI_HDLCFCSMODE, &one);
		/* we cannot reliably check for the ioctl return value here
		 * as this command will fail if the slot _already_ was a
		 * signalling slot before :( */
	}
	return 0;
}

static void mux_q_empty_cb(struct osmo_i460_subchan *schan, void *user_data);

/* RTP data was received on the socket */
static void ortp_rx_cb(struct osmo_rtp_socket *rs, const uint8_t *payload,
			unsigned int payload_len, uint16_t seq_number, uint32_t timestamp, bool marker)
{
	struct sc_state *sc = rs->priv;
	struct osmo_trau_frame fr;
	int rc;

	LOGSC(sc, "RTP Rx: %s\n", osmo_hexdump_nospc(payload, payload_len));

	sc->t2r.type = g_ftype;

	memset(&fr, 0, sizeof(fr));
	fr.dir = OSMO_TRAU_DIR_DL;
	rc = osmo_rtp2trau(&fr, payload, payload_len, &sc->t2r);
	if (rc < 0) {
		LOGSC(sc, "Failed to convert RTP to TRAU");
		return;
	}

	struct msgb *msg = msgb_alloc(2*40*8, "rtp2trau");
	rc = osmo_trau_frame_encode(msgb_data(msg), 2*40*8, &fr);
	OSMO_ASSERT(rc >= 0);
	msgb_put(msg, rc);
	osmo_i460_mux_enqueue(sc->i460_sc, msg);
}

static void init(const char *fname)
{
	struct stat st;
	int rc;

	struct osmo_i460_schan_desc scd16_0 = {
		.rate = OSMO_I460_RATE_16k,
		.demux = {
			.num_bits = 40*8,
			.out_cb_bytes = NULL,
		},
	};

	rc = stat(fname, &st);
	OSMO_ASSERT(rc == 0);
	if (S_ISCHR(st.st_mode)) {
		/* if this is a char device, we assume DAHDI */
		g_ts_is_writable = true;
		g_st.in_fd = open(fname, O_RDWR);
		OSMO_ASSERT(g_st.in_fd >= 0);
		dahdi_set_bufinfo(g_st.in_fd, false);
	} else {
		g_st.in_fd = open(fname, O_RDONLY);
		OSMO_ASSERT(g_st.in_fd >= 0);
	}

	osmo_i460_ts_init(&g_st.i460_ts);

	//for (i = 0; i < ARRAY_SIZE(g_st.sc); i++) {
	for (int i = 1; i < 3; i++) {
		struct sc_state *sc = &g_st.sc[i];
		sc->num = i;

		scd16_0.bit_offset = i * 2;
		scd16_0.demux.user_data = sc;
		scd16_0.demux.out_cb_bits = i460_demux_bits_cb,
		scd16_0.mux.in_cb_queue_empty = mux_q_empty_cb;
		scd16_0.mux.user_data = sc;
		sc->i460_sc = osmo_i460_subchan_add(NULL, &g_st.i460_ts, &scd16_0);
		OSMO_ASSERT(sc->i460_sc != NULL);

		char strbuf[16];
		snprintf(strbuf, sizeof(strbuf), "SC%u", sc->num);
		sc->sync_fi = osmo_trau_sync_alloc(NULL, strbuf, sync_frame_out_cb, OSMO_TRAU_SYNCP_16_FR_EFR, sc);
		OSMO_ASSERT(sc->sync_fi);
		sc->rtps = osmo_rtp_socket_create(NULL, OSMO_RTP_F_POLL);
		OSMO_ASSERT(sc->rtps);
		osmo_rtp_socket_set_pt(sc->rtps, RTP_PT_GSM_FULL);
		sc->rtps->rx_cb = ortp_rx_cb;
		sc->rtps->priv = sc;
		osmo_rtp_socket_bind(sc->rtps, "0.0.0.0", g_local_port + i*2);
		osmo_rtp_socket_connect(sc->rtps, g_remote_host, g_remote_port + i*2);
		//osmo_rtp_socket_autoconnect(sc->rtps);
	}
}

static void mux_q_empty_cb(struct osmo_i460_subchan *schan, void *user_data)
{
	struct sc_state *sc = user_data;
	struct msgb *msg = msgb_alloc(2*40*8, "mux-enq");
	struct osmo_trau_frame traufr = {
		.type = g_ftype,
		.dir = OSMO_TRAU_DIR_DL,
	};
	int rc;

	LOGSC(sc, "EMPTY -> Generating Tx\n");

	if (traufr.type == OSMO_TRAU16_FT_EFR) {
		traufr.c_bits[12-1] = 1; /* C12 = good u-link frame */
		traufr.c_bits[16-1] = 1; /* C16 = SP[eech]; no DTX */
	}

	rc = osmo_trau_frame_encode(msgb_data(msg), 2*40*8, &traufr);
	OSMO_ASSERT(rc >= 0);
	msgb_put(msg, rc);
	//LOGSC(sc, "Tx TRAU: %s\n", osmo_ubit_dump(cur, 40*8));
	osmo_i460_mux_enqueue(sc->i460_sc, msg);
}

static void process(void)
{
	uint8_t buf[D_BCHAN_TX_GRAN];
	int rc, nread;

	while (rc = read(g_st.in_fd, buf, sizeof(buf))) {
		OSMO_ASSERT(rc == sizeof(buf));
		nread = rc;
		osmo_i460_demux_in(&g_st.i460_ts, buf, nread);

		for (int i = 1; i < 3; i++) {
			struct sc_state *sc = &g_st.sc[i];
			int rc2 = osmo_rtp_socket_poll(sc->rtps);
			sc->rtps->rx_user_ts += 160;
			//printf("rtp_recv=%d (flags=%x)\n",rc2, sc->rtps->flags);
		}

		/* write as many bytes as we just received */
		osmo_i460_mux_out(&g_st.i460_ts, buf, nread);
		if (g_ts_is_writable) {
			rc = write(g_st.in_fd, buf, nread);
			if (rc != nread)
				printf("rc=%d, nread=%d (%s)\n", rc, nread, strerror(errno));
			OSMO_ASSERT(rc == nread);
		}
		//usleep(20000);
	}
}

int main(int argc, char **argv)
{
	osmo_init_logging2(NULL, NULL);
	osmo_fsm_log_addr(false);
	log_set_print_filename2(osmo_stderr_target, LOG_FILENAME_BASENAME);
	log_set_category_filter(osmo_stderr_target, DLMIB, true, LOGL_DEBUG);
	osmo_rtp_init(NULL);

	init(argv[1]);
	process();
}

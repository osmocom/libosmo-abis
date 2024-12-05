#ifndef _E1_INPUT_H
#define _E1_INPUT_H

#include <stdlib.h>
#include <netinet/in.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/use_count.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/select.h>
#include <osmocom/netif/ipa.h>
#include <osmocom/gsm/i460_mux.h>
#include <osmocom/abis/subchan_demux.h>
#include <osmocom/abis/lapd.h>

#define NUM_E1_TS   32
#define E1INP_USE_DEFAULT (-1)

#define LOGPITS(e1ts, ss, level, fmt, args ...) \
       LOGP(ss, level, "E1TS(%u:%u) " fmt, (e1ts)->line->num, (e1ts)->num, ## args)

#define LOGPIL(e1l, ss, level, fmt, args ...) \
       LOGP(ss, level, "E1L(%u) " fmt, (e1l)->num, ## args)

enum e1inp_sign_type {
	E1INP_SIGN_NONE,
	E1INP_SIGN_OML,
	E1INP_SIGN_RSL,
	E1INP_SIGN_OSMO,	/* IPA CCM OSMO sub-type */
};
const char *e1inp_signtype_name(enum e1inp_sign_type tp);
extern const struct value_string e1inp_sign_type_names[5];

enum e1inp_ctr {
	E1I_CTR_HDLC_ABORT,
	E1I_CTR_HDLC_BADFCS,
	E1I_CTR_HDLC_OVERR,
	E1I_CTR_ALARM,
	E1I_CTR_REMOVED,
};

struct e1inp_ts;
struct vty;

struct e1inp_sign_link {
	/* list of signalling links */
	struct llist_head list;

	/* to which timeslot do we belong? */
	struct e1inp_ts *ts;

	enum e1inp_sign_type type;

	/* trx for msg->trx of received msgs */
	struct gsm_bts_trx *trx;

	/* msgb queue of to-be-transmitted msgs */
	struct llist_head tx_list;

	/* SAPI and TEI on the E1 TS */
	uint8_t sapi;
	uint8_t tei;

	union {
		struct {
			uint8_t channel;
		} misdn;
	} driver;
};

enum e1inp_ts_type {
	E1INP_TS_TYPE_NONE,
	E1INP_TS_TYPE_SIGN,
	E1INP_TS_TYPE_TRAU,
	E1INP_TS_TYPE_RAW,
	E1INP_TS_TYPE_HDLC,
	E1INP_TS_TYPE_I460,
};
const char *e1inp_tstype_name(enum e1inp_ts_type tp);
extern const struct value_string e1inp_ts_type_names[];

/* A timeslot in the E1 interface */
struct e1inp_ts {
	enum e1inp_ts_type type;
	int num;

	/* to which line do we belong ? */
	struct e1inp_line *line;

	/* LAPD instance, if any */
	struct lapd_instance *lapd;

	union {
		struct {
			/* list of all signalling links on this TS */
			struct llist_head sign_links;
			/* delay for the queue */
			int delay;
			/* timer when to dequeue next frame */
			struct osmo_timer_list tx_timer;
		} sign;
		struct {
			/* subchannel demuxer for frames from E1 */
			struct subch_demux demux;
			/* subchannel muxer for frames to E1 */
			struct subch_mux mux;
		} trau;
		struct {
			/* call-back for every received frame */
			void (*recv_cb)(struct e1inp_ts *ts, struct msgb *msg);
			/* queue of pending to-be-transmitted msgbs */
			struct llist_head tx_queue;
		} raw;
		struct {
			/* call-back for every received frame */
			void (*recv_cb)(struct e1inp_ts *ts, struct msgb *msg);
			/* queue of pending to-be-transmitted msgbs */
			struct llist_head tx_queue;
		} hdlc;
		struct {
			struct osmo_i460_timeslot i460_ts;
		} i460;
	};
	union {
		struct {
			/* mISDN driver has one fd for each ts */
			struct osmo_fd fd;
		} misdn;
		struct {
			/* ip.access driver has one fd for each ts */
			struct osmo_fd fd;
			/* ipa keep-alive */
			struct osmo_ipa_ka_fsm_inst *ka_fsm;
		} ipaccess;
		struct {
			/* DAHDI driver has one fd for each ts */
			struct osmo_fd fd;
		} dahdi;
		struct {
			/* osmo-e1d driver has one fd for each ts */
			struct osmo_fd fd;
		} e1d;
		struct {
			struct osmo_fd fd;
		} rs232;
	} driver;

	struct msgb *pending_msg;
};

#define E1_SUBSLOT_FULL 0xff

struct gsm_e1_subslot {
	/* Number of E1 link */
	uint8_t e1_nr;
	/* Number of E1 TS inside E1 link */
	uint8_t e1_ts;
	/* Sub-slot within the E1 TS, 0xff (E1_SUBSLOT_FULL) if full TS */
	uint8_t e1_ts_ss;
};

enum e1inp_line_role {
	E1INP_LINE_R_NONE,
	E1INP_LINE_R_BSC,
	E1INP_LINE_R_BTS,
	E1INP_LINE_R_MAX
};

/* Notes on setting/getting Sa bits (TABLE 5A and 5B of ITU-T G.704)
 * The sa_bits byte contains the Sa bits in TS 0, if multi frame is used:
 * bit 7 = Sa8, bit 6 = Sa7, bit 5 = Sa5, bit 4 = Sa4, bit 3 = Sa64, bit 2 = Sa63, bit 1 = Sa62, bit 0 = Sa61.
 * The sa_bits byte contains the Sa bits in TS 0, if double frame is used:
 * bit 7 = Sa8, bit 6 = Sa7, bit 5 = Sa5, bit 4 = Sa4, bit 0 = Sa6.
 * The received Sa bits are reported whenever they change (and are stable for some frames, depending on the
 * implementation).  Before any report, all Sa bits are assumed to be set to 1.
 * Setting sa_bits byte will change the transmitted Sa bits on TS0.
 * Before setting them, all Sa bits are transmitted as 1.
 */

struct e1inp_driver {
	struct llist_head list;
	const char *name;
	int (*want_write)(struct e1inp_ts *ts);
	int (*line_update)(struct e1inp_line *line);
	void (*close)(struct e1inp_sign_link *link);
	void (*vty_show)(struct vty *vty, struct e1inp_line *line);
	int default_delay;
	int has_keepalive;
	const char *bind_addr;

	/* Set Sa bits to transmit in TS0 (MSB to LSB): Sa8 Sa7 Sa5 Sa4 Sa64 Sa63 Sa62 Sa61/Sa6 */
	int (*set_sa_bits)(struct e1inp_line *line, uint8_t sa_bits);

	/* Optional callback to perform driver specific initialization when the line is created. */
	int (*line_create)(struct e1inp_line *line);
};

struct e1inp_line_ops {
	union {
		struct {
			enum e1inp_line_role role;	/* BSC or BTS mode. */
			const char *addr;		/* IP address .*/
			void *dev;			/* device parameters. */
		} ipa;
		struct {
			const char *port;		/* e.g. /dev/ttyUSB0 */
			unsigned int delay;
		} rs232;
	} cfg;

	struct e1inp_sign_link *	(*sign_link_up)(void *unit_info, struct e1inp_line *line, enum e1inp_sign_type type);
	void	(*sign_link_down)(struct e1inp_line *line);
	/* Called when a new message arrives. -EBADF must be returned if the osmo_fd in link (msg->dst) is destroyed. */
	int	(*sign_link)(struct msgb *msg);
};

struct e1inp_line {
	struct llist_head list;
	int refcnt; /* unusued, kept for ABI compat, use_count is used instead */

	unsigned int num;
	const char *name;
	unsigned int port_nr;
	char *sock_path;
	struct rate_ctr_group *rate_ctr;

	/* tcp keepalive configuration */
	int keepalive_num_probes; /* 0: disable, num, or E1INP_USE_DEFAULT */
	int keepalive_idle_timeout; /* secs, or E1INP_USE_DEFAULT */
	int keepalive_probe_interval; /* secs or E1INP_USE_DEFAULT */

	/* ipa ping/pong keepalive params */
	struct ipa_keepalive_params *ipa_kap;

	/* array of timestlots */
	struct e1inp_ts ts[NUM_E1_TS];
	unsigned int num_ts;

	const struct e1inp_line_ops *ops;

	struct e1inp_driver *driver;
	void *driver_data;

	struct osmo_use_count use_count;

	/* file name and file descriptor of pcap for this line */
	char *pcap_file;
	int pcap_fd;

	unsigned int connect_timeout;
};
#define e1inp_line_ipa_oml_ts(line) (&line->ts[0])
#define e1inp_line_ipa_rsl_ts(line, trx_id) (((1 + (trx_id)) < NUM_E1_TS) ? (&line->ts[1 + (trx_id)]) : NULL)

/* SS_L_INPUT signals */
enum e1inp_signal_input {
	S_L_INP_NONE,
	S_L_INP_TEI_UP,
	S_L_INP_TEI_DN,
	S_L_INP_TEI_UNKNOWN,
	S_L_INP_LINE_INIT,
	S_L_INP_LINE_ALARM,
	S_L_INP_LINE_NOALARM,
	S_L_INP_LINE_LOS,
	S_L_INP_LINE_NOLOS,
	S_L_INP_LINE_AIS,
	S_L_INP_LINE_NOAIS,
	S_L_INP_LINE_RAI,
	S_L_INP_LINE_NORAI,
	S_L_INP_LINE_SLIP_RX,
	S_L_INP_LINE_SLIP_TX,
	S_L_INP_LINE_SA_BITS,
	S_L_INP_LINE_LOF,
	S_L_INP_LINE_NOLOF,
};

extern const struct value_string e1inp_signal_names[];

/* register a driver with the E1 core */
int e1inp_driver_register(struct e1inp_driver *drv);

/* fine a previously registered driver */
struct e1inp_driver *e1inp_driver_find(const char *name);

/* get a line by its ID */
struct e1inp_line *e1inp_line_find(uint8_t e1_nr);

/* create a line in the E1 input core */
struct e1inp_line *e1inp_line_create(uint8_t e1_nr, const char *driver_name);

/* clone one existing E1 input line */
struct e1inp_line *e1inp_line_clone(void *ctx, struct e1inp_line *line, const char *use);

/* increment refcount use of E1 input line */
void e1inp_line_get(struct e1inp_line *line) OSMO_DEPRECATED("Use e1inp_line_get2() instead");

/* decrement refcount use of E1 input line, release if unused */
void e1inp_line_put(struct e1inp_line *line) OSMO_DEPRECATED("Use e1inp_line_put2() instead");

/* Convenience macros for struct foo instances. These are strict about use count errors. */
#define e1inp_line_get2(line, USE) OSMO_ASSERT( osmo_use_count_get_put(&(line)->use_count, USE, 1) == 0 );
#define e1inp_line_put2(line, USE) OSMO_ASSERT( osmo_use_count_get_put(&(line)->use_count, USE, -1) == 0 );

/* bind operations to one E1 input line */
void e1inp_line_bind_ops(struct e1inp_line *line, const struct e1inp_line_ops *ops);

/* find a sign_link for given TEI and SAPI in a TS */
struct e1inp_sign_link *
e1inp_lookup_sign_link(struct e1inp_ts *ts, uint8_t tei,
			uint8_t sapi);

/* create a new signalling link in a E1 timeslot */
struct e1inp_sign_link *
e1inp_sign_link_create(struct e1inp_ts *ts, enum e1inp_sign_type type,
			struct gsm_bts_trx *trx, uint8_t tei,
			uint8_t sapi);

/* configure and initialize one signalling e1inp_ts */
int e1inp_ts_config_sign(struct e1inp_ts *ts, struct e1inp_line *line);

/* configure and initialize one timeslot dedicated to TRAU frames. */
int e1inp_ts_config_trau(struct e1inp_ts *ts, struct e1inp_line *line,
                         int (*trau_rcv_cb)(struct subch_demux *dmx, int ch,
					    const ubit_t *data, int len, void *_priv));

/* configure and initialize one timeslot dedicated to RAW frames */
int e1inp_ts_config_raw(struct e1inp_ts *ts, struct e1inp_line *line,
			void (*raw_recv_cb)(struct e1inp_ts *ts,
					    struct msgb *msg));

/* configure and initialize one timeslot dedicated to HDLC frames */
int e1inp_ts_config_hdlc(struct e1inp_ts *ts, struct e1inp_line *line,
			 void (*hdlc_recv_cb)(struct e1inp_ts *ts,
					      struct msgb *msg));

/* configure and initialize one timeslot dedicated to nothing */
int e1inp_ts_config_none(struct e1inp_ts *ts, struct e1inp_line *line);

/*
 * configure Sa bits on TS0, if supported by driver (TABLE 5A and 5B of ITU-T G.704)
 * sa_bits (MSB to LSB): Sa8 Sa7 Sa5 Sa4 Sa64 Sa63 Sa62 Sa61/Sa6
 */
int e1inp_ts_set_sa_bits(struct e1inp_line *line, uint8_t sa_bits);

/* obtain a string identifier/name for the given timeslot */
void e1inp_ts_name(char *out, size_t out_len, const struct e1inp_ts *ts);

/* Receive a packet from the E1 driver */
int e1inp_rx_ts(struct e1inp_ts *ts, struct msgb *msg,
		uint8_t tei, uint8_t sapi);
int e1inp_rx_ts_lapd(struct e1inp_ts *e1i_ts, struct msgb *msg);

/* called by driver if it wants to transmit on a given TS */
struct msgb *e1inp_tx_ts(struct e1inp_ts *e1i_ts,
			 struct e1inp_sign_link **sign_link);

/* called by driver in case some kind of link state event */
int e1inp_event(struct e1inp_ts *ts, int evt, uint8_t tei, uint8_t sapi);

/* L2->L3 */
void e1inp_dlsap_up(struct osmo_dlsap_prim *odp, uint8_t tei, uint8_t sapi,
        void *rx_cbdata);

/* Write LAPD frames to the fd. */
OSMO_DEPRECATED("Use e1_set_pcap_fd2() instead")
int e1_set_pcap_fd(int fd);

int e1_set_pcap_fd2(struct e1inp_line *line, int fd);

/* called by TRAU muxer to obtain the destination mux entity */
struct subch_mux *e1inp_get_mux(uint8_t e1_nr, uint8_t ts_nr);

/* on an IPA BTS, the BTS needs to establish the RSL connection much
 * later than the OML connection. */
int e1inp_ipa_bts_rsl_connect(struct e1inp_line *line,
			      const char *rem_addr, uint16_t rem_port);

int e1inp_ipa_bts_rsl_connect_n(struct e1inp_line *line,
			      const char *rem_addr, uint16_t rem_port,
			      uint8_t trx_nr);
int e1inp_ipa_bts_rsl_close_n(struct e1inp_line *line, uint8_t trx_nr);

void e1inp_sign_link_destroy(struct e1inp_sign_link *link);
int e1inp_line_update(struct e1inp_line *line);

int e1inp_vty_init(void);

/* activate superchannel or deactive to use timeslots. only valid for unixsocket driver */
void e1inp_ericsson_set_altc(struct e1inp_line *unixlinue, int superchannel);

extern struct llist_head e1inp_driver_list;
extern struct llist_head e1inp_line_list;

/* XXX */
struct input_signal_data {
	enum e1inp_sign_type link_type;
	uint8_t tei;
	uint8_t sapi;
	uint8_t ts_nr;
	/* received Sa bits in TS0 (MSB to LSB): Sa8 Sa7 Sa5 Sa4 Sa64 Sa63 Sa62 Sa61/Sa6 */
	uint8_t sa_bits;
	struct gsm_bts_trx *trx;
	struct e1inp_line *line;
};

int abis_sendmsg(struct msgb *msg);
int abis_rsl_sendmsg(struct msgb *msg);

int e1inp_ts_send_raw(struct e1inp_ts *ts, struct msgb *msg);
int e1inp_ts_send_hdlc(struct e1inp_ts *ts, struct msgb *msg);

#endif /* _E1_INPUT_H */

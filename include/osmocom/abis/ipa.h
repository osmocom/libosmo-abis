#ifndef _OSMO_IPA_H_
#define _OSMO_IPA_H_

#include <stdint.h>
#include <osmocom/core/defs.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/select.h>
#include <osmocom/core/fsm.h>
#include <osmocom/gsm/ipa.h>

struct e1inp_line;
struct e1inp_ts;
struct msgb;

struct ipa_server_link {
	struct e1inp_line		*line;
	struct osmo_fd			ofd;
	const char			*addr;
	uint16_t			port;
	int (*accept_cb)(struct ipa_server_link *link, int fd);
	void				*data;
	uint8_t				dscp;
	uint8_t				priority;
};

struct ipa_server_link *
ipa_server_link_create(void *ctx, struct e1inp_line *line, const char *addr,
		       uint16_t port,
		       int (*accept_cb)(struct ipa_server_link *link, int fd),
		       void *data);
void ipa_server_link_destroy(struct ipa_server_link *link);

int ipa_server_link_open(struct ipa_server_link *link);
void ipa_server_link_close(struct ipa_server_link *link);

struct ipa_server_conn {
	struct ipa_server_link		*server;
	struct osmo_fd			ofd;
	struct llist_head		tx_queue;
	int (*closed_cb)(struct ipa_server_conn *peer);
	int (*ccm_cb)(struct ipa_server_conn *peer, struct msgb *msg,
			struct tlv_parsed *tlvp, struct ipaccess_unit *ud);
	/* Callback when ofd has something to be read. -EBADF must be returned if the osmo_fd is destroyed. */
	int (*cb)(struct ipa_server_conn *peer, struct msgb *msg);
	void				*data;
	struct msgb			*pending_msg;
	/* remote address information */
	const char			*addr;
	uint16_t			port;
};

struct ipa_server_conn *
ipa_server_conn_create(void *ctx, struct ipa_server_link *link, int fd,
		       int (*cb)(struct ipa_server_conn *peer, struct msgb *msg),
		       int (*closed_cb)(struct ipa_server_conn *peer),
		       void *data);
void ipa_server_conn_destroy(struct ipa_server_conn *peer);

void ipa_server_conn_send(struct ipa_server_conn *peer, struct msgb *msg);
int ipa_server_conn_ccm(struct ipa_server_conn *conn, struct msgb *msg);

enum ipa_client_conn_state {
	IPA_CLIENT_LINK_STATE_NONE         = 0,
	IPA_CLIENT_LINK_STATE_CONNECTING   = 1,
	IPA_CLIENT_LINK_STATE_CONNECTED    = 2,
	IPA_CLIENT_LINK_STATE_MAX
};

struct ipa_client_conn {
	struct e1inp_line		*line;
	struct osmo_fd			*ofd;
	struct llist_head		tx_queue;
	struct osmo_timer_list		timer;
	enum ipa_client_conn_state	state;
	const char			*addr;
	uint16_t			port;
	void (*updown_cb)(struct ipa_client_conn *link, int up);
	/* Callback when ofd has something to be read. -EBADF must be returned if the osmo_fd is destroyed. */
	int (*read_cb)(struct ipa_client_conn *link, struct msgb *msg);
	int (*write_cb)(struct ipa_client_conn *link);
	void				*data;
	struct msgb			*pending_msg;
	const char			*local_addr;
	uint16_t			local_port;
	uint8_t				dscp;
	uint8_t				priority;
};

struct ipa_client_conn *
ipa_client_conn_create(void *ctx, struct e1inp_ts *ts, int priv_nr,
			const char *addr, uint16_t port,
			void (*updown)(struct ipa_client_conn *link, int),
			int (*read_cb)(struct ipa_client_conn *link, struct msgb *msgb),
			int (*write_cb)(struct ipa_client_conn *link),
			void *data) OSMO_DEPRECATED("Use ipa_client_conn_create2() instead");
struct ipa_client_conn *
ipa_client_conn_create2(void *ctx, struct e1inp_ts *ts,
		       int priv_nr, const char *loc_addr, uint16_t loc_port,
		       const char *rem_addr, uint16_t rem_port,
		       void (*updown_cb)(struct ipa_client_conn *link, int up),
		       int (*read_cb)(struct ipa_client_conn *link,
				      struct msgb *msgb),
		       int (*write_cb)(struct ipa_client_conn *link),
		       void *data);
void ipa_client_conn_destroy(struct ipa_client_conn *link);

int ipa_client_conn_open(struct ipa_client_conn *link);
int ipa_client_conn_open2(struct ipa_client_conn *link, unsigned int connect_timeout);
void ipa_client_conn_close(struct ipa_client_conn *link);

void ipa_client_conn_send(struct ipa_client_conn *link, struct msgb *msg);
size_t ipa_client_conn_clear_queue(struct ipa_client_conn *link);

int ipaccess_bts_handle_ccm(struct ipa_client_conn *link,
			    struct ipaccess_unit *dev, struct msgb *msg);

void ipa_msg_push_header(struct msgb *msg, uint8_t proto) OSMO_DEPRECATED("Use libosmo-netif/osmo_ipa_msg_push_header() or libosmocore/ipa_prepend_header() instead");

/***********************************************************************
 * IPA Keep-Alive FSM
 ***********************************************************************/

/*! parameters describing the keep-alive FSM (timeouts). */
struct ipa_keepalive_params {
	/*! interval in which to send IPA CCM PING requests to the peer. */
	unsigned int interval;
	/*! time to wait for an IPA CCM PONG in response to a IPA CCM PING before giving up. */
	unsigned int wait_for_resp;
};

typedef int ipa_keepalive_timeout_cb_t(struct osmo_fsm_inst *fi, void *conn);

typedef void ipa_keepalive_send_cb_t(struct osmo_fsm_inst *fi, void *conn, struct msgb *msg);

struct osmo_fsm_inst *ipa_client_conn_alloc_keepalive_fsm(struct ipa_client_conn *client,
							  const struct ipa_keepalive_params *params,
							  const char *id);

struct osmo_fsm_inst *ipa_server_conn_alloc_keepalive_fsm(struct ipa_server_conn *server,
							  const struct ipa_keepalive_params *params,
							  const char *id);

struct osmo_fsm_inst *ipa_generic_conn_alloc_keepalive_fsm(void *ctx, void* data,
							  const struct ipa_keepalive_params *params,
							  const char *id);

void ipa_keepalive_fsm_set_timeout_cb(struct osmo_fsm_inst *fi, ipa_keepalive_timeout_cb_t *cb);

void ipa_keepalive_fsm_set_send_cb(struct osmo_fsm_inst *fi, ipa_keepalive_send_cb_t *fn);

void ipa_keepalive_fsm_start(struct osmo_fsm_inst *fi);

void ipa_keepalive_fsm_stop(struct osmo_fsm_inst *fi);

void ipa_keepalive_fsm_pong_received(struct osmo_fsm_inst *fi);

#endif

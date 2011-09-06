#ifndef _OSMO_ORTP_H
#define _OSMO_ORTP_H

#include <stdint.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/select.h>

//#include <ortp/ortp.h>
struct _RtpSession;


#define RTP_PT_GSM_FULL 3
#define RTP_PT_GSM_HALF 96
#define RTP_PT_GSM_EFR 97
#define RTP_PT_AMR 98

#define OSMO_RTP_F_POLL		0x0001

struct osmo_rtp_socket {
	/*! \biref list header for global list of sockets */
	struct llist_head list;

	/*! \brief libortp RTP session pointer */
	struct _RtpSession *sess;
	/*! \brief Osmo file descriptor for RTP socket FD */
	struct osmo_fd rtp_bfd;
	/*! \brief Osmo file descriptor for RTCP socket FD */
	struct osmo_fd rtcp_bfd;

	/*! \brief callback for incoming data */
	void (*rx_cb)(struct osmo_rtp_socket *rs, const uint8_t *payload,
		      unsigned int payload_len);

	/* Rx related */
	uint32_t rx_user_ts;

	/* Tx related */
	uint32_t tx_timestamp;

	unsigned int flags;

	void *priv;
};

void osmo_rtp_init(void *ctx);
struct osmo_rtp_socket *osmo_rtp_socket_create(void *talloc_ctx, unsigned int flags);
int osmo_rtp_socket_bind(struct osmo_rtp_socket *rs, const char *ip, int port);
int osmo_rtp_socket_connect(struct osmo_rtp_socket *rs, const char *ip, uint16_t port);
int osmo_rtp_socket_set_pt(struct osmo_rtp_socket *rs, int payload_type);
int osmo_rtp_socket_free(struct osmo_rtp_socket *rs);
int osmo_rtp_send_frame(struct osmo_rtp_socket *rs, const uint8_t *payload,
			unsigned int payload_len, unsigned int duration);
int osmo_rtp_socket_poll(struct osmo_rtp_socket *rs);

int osmo_rtp_get_bound_ip_port(struct osmo_rtp_socket *rs,
			       uint32_t *ip, int *port);
int osmo_rtp_get_bound_addr(struct osmo_rtp_socket *rs,
			    const char **addr, int *port);

#endif /* _OSMO_ORTP_H */

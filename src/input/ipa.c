#include "internal.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <osmocom/core/select.h>
#include <osmocom/core/timer.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>
#include <osmocom/core/backtrace.h>

#include <osmocom/abis/ipa.h>

#define LOGIPA(link, level, fmt, args...) LOGP(DLINP, level, "%s:%u " fmt, link->addr, link->port, ## args)

void ipa_msg_push_header(struct msgb *msg, uint8_t proto)
{
	struct ipaccess_head *hh;

	msg->l2h = msg->data;
	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh));
	hh->proto = proto;
	hh->len = htons(msgb_l2len(msg));
}

void ipa_client_conn_close(struct ipa_client_conn *link)
{
	osmo_timer_del(&link->timer);

	/* be safe against multiple calls */
	if (link->ofd->fd != -1) {
		osmo_fd_unregister(link->ofd);
		close(link->ofd->fd);
		link->ofd->fd = -1;
	}
	msgb_free(link->pending_msg);
	link->pending_msg = NULL;
}

static int ipa_client_read(struct ipa_client_conn *link)
{
	struct osmo_fd *ofd = link->ofd;
	struct msgb *msg;
	int ret;

	LOGIPA(link, LOGL_DEBUG, "message received\n");

	ret = ipa_msg_recv_buffered(ofd->fd, &msg, &link->pending_msg);
	if (ret <= 0) {
		if (ret == -EAGAIN)
			return 0;
		else if (ret == -EPIPE || ret == -ECONNRESET)
			LOGIPA(link, LOGL_ERROR, "lost connection with server\n");
		else if (ret == 0)
			LOGIPA(link, LOGL_ERROR, "connection closed with server\n");
		ipa_client_conn_close(link);
		if (link->updown_cb)
			link->updown_cb(link, 0);
		return -EBADF;
	}
	if (link->read_cb)
		return link->read_cb(link, msg);
	return 0;
}

static void ipa_client_write(struct ipa_client_conn *link)
{
	if (link->write_cb)
		link->write_cb(link);
}

static int ipa_client_write_default_cb(struct ipa_client_conn *link)
{
	struct osmo_fd *ofd = link->ofd;
	struct msgb *msg;
	struct llist_head *lh;
	int ret;

	LOGIPA(link, LOGL_DEBUG, "sending data\n");

	if (llist_empty(&link->tx_queue)) {
		osmo_fd_write_disable(ofd);
		return 0;
	}
	lh = link->tx_queue.next;
	llist_del(lh);
	msg = llist_entry(lh, struct msgb, list);

	ret = send(link->ofd->fd, msg->data, msg->len, 0);
	if (ret < 0) {
		if (errno == EPIPE || errno == ENOTCONN) {
			ipa_client_conn_close(link);
			if (link->updown_cb)
				link->updown_cb(link, 0);
		}
		LOGIPA(link, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
	return 0;
}

static void ipa_connect_failure(struct ipa_client_conn *link)
{
	ipa_client_conn_close(link);
	if (link->updown_cb)
		link->updown_cb(link, 0);
}

static int ipa_client_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_client_conn *link = ofd->data;
	int error, ret = 0;
	socklen_t len = sizeof(error);

	switch(link->state) {
	case IPA_CLIENT_LINK_STATE_CONNECTING:
		ret = getsockopt(ofd->fd, SOL_SOCKET, SO_ERROR, &error, &len);
		if (ret >= 0 && error > 0) {
			ipa_connect_failure(link);
			return 0;
		}

		/* Stop the timer when connection succeeds, on failure it's deleted in
		   ipa_client_conn_close() called by ipa_connect_failure() above */
		osmo_timer_del(&link->timer);

		osmo_fd_write_disable(ofd);
		LOGIPA(link, LOGL_NOTICE, "connection done\n");
		link->state = IPA_CLIENT_LINK_STATE_CONNECTED;
		if (link->updown_cb)
			link->updown_cb(link, 1);
		break;
	case IPA_CLIENT_LINK_STATE_CONNECTED:
		if (what & OSMO_FD_READ) {
			LOGIPA(link, LOGL_DEBUG, "connected read\n");
			ret = ipa_client_read(link);
		}
		if (ret != -EBADF && (what & OSMO_FD_WRITE)) {
			LOGIPA(link, LOGL_DEBUG, "connected write\n");
			ipa_client_write(link);
		}
		break;
	default:
		break;
	}
	return 0;
}

/* Treat the connect timeout exactly like a connect failure */
static void ipa_connect_timeout_cb(void *data)
{
	struct ipa_client_conn *link = (struct ipa_client_conn *)data;

	LOGIPA(link, LOGL_NOTICE, "Connect timeout reached\n");
	ipa_connect_failure(link);
}

struct ipa_client_conn *
ipa_client_conn_create(void *ctx, struct e1inp_ts *ts,
		       int priv_nr, const char *addr, uint16_t port,
		       void (*updown_cb)(struct ipa_client_conn *link, int up),
		       int (*read_cb)(struct ipa_client_conn *link,
				      struct msgb *msgb),
		       int (*write_cb)(struct ipa_client_conn *link),
		       void *data)
{
	return ipa_client_conn_create2(ctx, ts, priv_nr, NULL, 0, addr, port,
				       updown_cb, read_cb, write_cb, data);
}

struct ipa_client_conn *
ipa_client_conn_create2(void *ctx, struct e1inp_ts *ts,
		       int priv_nr, const char *loc_addr, uint16_t loc_port,
		       const char *rem_addr, uint16_t rem_port,
		       void (*updown_cb)(struct ipa_client_conn *link, int up),
		       int (*read_cb)(struct ipa_client_conn *link,
				      struct msgb *msgb),
		       int (*write_cb)(struct ipa_client_conn *link),
		       void *data)
{
	struct ipa_client_conn *ipa_link;

	ipa_link = talloc_zero(ctx, struct ipa_client_conn);
	if (!ipa_link)
		return NULL;

	if (ts) {
		if (ts->line->driver == NULL) {
			talloc_free(ipa_link);
			return NULL;
		}
		ipa_link->ofd = &ts->driver.ipaccess.fd;
	} else {
		ipa_link->ofd = talloc_zero(ctx, struct osmo_fd);
		if (ipa_link->ofd == NULL) {
			talloc_free(ipa_link);
			return NULL;
		}
	}

	osmo_fd_setup(ipa_link->ofd, -1, OSMO_FD_READ|OSMO_FD_WRITE, ipa_client_fd_cb, ipa_link, priv_nr);
	ipa_link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
	ipa_link->local_addr = talloc_strdup(ipa_link, loc_addr);
	ipa_link->local_port = loc_port;
	ipa_link->addr = talloc_strdup(ipa_link, rem_addr);
	ipa_link->port = rem_port;
	ipa_link->updown_cb = updown_cb;
	ipa_link->read_cb = read_cb;
	osmo_timer_setup(&ipa_link->timer, ipa_connect_timeout_cb, ipa_link);

	/* default to generic write callback if not set. */
	if (write_cb == NULL)
		ipa_link->write_cb = ipa_client_write_default_cb;
	else
		ipa_link->write_cb = write_cb;

	if (ts)
		ipa_link->line = ts->line;
	ipa_link->data = data;
	INIT_LLIST_HEAD(&ipa_link->tx_queue);

	return ipa_link;
}

void ipa_client_conn_destroy(struct ipa_client_conn *link)
{
	talloc_free(link);
}

int ipa_client_conn_open(struct ipa_client_conn *link)
{
	return ipa_client_conn_open2(link, 30);
}

int ipa_client_conn_open2(struct ipa_client_conn *link, unsigned int connect_timeout)
{
	int ret;

	if (link->ofd->fd != -1)
		return -EINVAL;

	link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
	ret = osmo_sock_init2(AF_INET, SOCK_STREAM, IPPROTO_TCP,
			     link->local_addr, link->local_port,
			     link->addr, link->port,
			     OSMO_SOCK_F_BIND|OSMO_SOCK_F_CONNECT|OSMO_SOCK_F_NONBLOCK|
			     OSMO_SOCK_F_DSCP(link->dscp) | OSMO_SOCK_F_PRIO(link->priority));
	if (ret < 0)
		return ret;
	link->ofd->fd = ret;
	osmo_fd_write_enable(link->ofd);
	if (osmo_fd_register(link->ofd) < 0) {
		close(ret);
		link->ofd->fd = -1;
		return -EIO;
	}

	if (connect_timeout > 0)
		osmo_timer_schedule(&link->timer, connect_timeout, 0);

	return 0;
}

void ipa_client_conn_send(struct ipa_client_conn *link, struct msgb *msg)
{
	msgb_enqueue(&link->tx_queue, msg);
	osmo_fd_write_enable(link->ofd);
}

size_t ipa_client_conn_clear_queue(struct ipa_client_conn *link)
{
	size_t deleted = 0;

	while (!llist_empty(&link->tx_queue)) {
		struct msgb *msg = msgb_dequeue(&link->tx_queue);
		msgb_free(msg);
		deleted += 1;
	}

	osmo_fd_write_disable(link->ofd);
	return deleted;
}

static int ipa_server_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	int fd, ret;
	char ipbuf[INET6_ADDRSTRLEN + 1];
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);
	struct ipa_server_link *link = ofd->data;

	fd = accept(ofd->fd, (struct sockaddr *)&sa, &sa_len);
	if (fd < 0) {
		LOGP(DLINP, LOGL_ERROR, "failed to accept from origin "
			"peer, reason=`%s'\n", strerror(errno));
		return fd;
	}

	if (!link->addr) {
		ret = osmo_sock_get_local_ip(fd, ipbuf, INET6_ADDRSTRLEN + 1);
		if (ret == 0)
			link->addr = talloc_strdup(link, ipbuf);
	}

	LOGIPA(link, LOGL_NOTICE, "accept()ed new link from %s:%u\n",
		inet_ntoa(sa.sin_addr), ntohs(sa.sin_port));

	/* make new fd inherit DSCP + priority of listen-socket */
	osmo_sock_set_dscp(fd, link->dscp);
	osmo_sock_set_priority(fd, link->priority);

	ret = link->accept_cb(link, fd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR,
		     "failed to process accept()ed new link, "
		     "reason=`%s'\n", strerror(-ret));
		close(fd);
		return ret;
	}

	return 0;
}

struct ipa_server_link *
ipa_server_link_create(void *ctx, struct e1inp_line *line,
		       const char *addr, uint16_t port,
		       int (*accept_cb)(struct ipa_server_link *link, int fd),
		       void *data)
{
	struct ipa_server_link *ipa_link;

	OSMO_ASSERT(accept_cb != NULL);

	ipa_link = talloc_zero(ctx, struct ipa_server_link);
	if (!ipa_link)
		return NULL;

	osmo_fd_setup(&ipa_link->ofd, -1, OSMO_FD_READ|OSMO_FD_WRITE, ipa_server_fd_cb, ipa_link, 0);
	if (addr)
		ipa_link->addr = talloc_strdup(ipa_link, addr);
	ipa_link->port = port;
	ipa_link->accept_cb = accept_cb;
	ipa_link->line = line;
	ipa_link->data = data;

	return ipa_link;

}

void ipa_server_link_destroy(struct ipa_server_link *link)
{
	talloc_free(link);
}

int ipa_server_link_open(struct ipa_server_link *link)
{
	int ret;

	ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
			     link->addr, link->port, OSMO_SOCK_F_BIND|
			     OSMO_SOCK_F_DSCP(link->dscp) | OSMO_SOCK_F_PRIO(link->priority));
	if (ret < 0)
		return ret;

	link->ofd.fd = ret;
	if (osmo_fd_register(&link->ofd) < 0) {
		close(ret);
		link->ofd.fd = -1;
		return -EIO;
	}
	return 0;
}

void ipa_server_link_close(struct ipa_server_link *link)
{
	if (link->ofd.fd == -1)
		return;

	osmo_fd_unregister(&link->ofd);
	close(link->ofd.fd);
	link->ofd.fd = -1;
}

static int ipa_server_conn_read(struct ipa_server_conn *conn)
{
	struct osmo_fd *ofd = &conn->ofd;
	struct msgb *msg;
	int ret;

	LOGIPA(conn, LOGL_DEBUG, "message received\n");

	ret = ipa_msg_recv_buffered(ofd->fd, &msg, &conn->pending_msg);
	if (ret <= 0) {
		if (ret == -EAGAIN)
			return 0;
		else if (ret == -EPIPE || ret == -ECONNRESET)
			LOGIPA(conn, LOGL_ERROR, "lost connection with server\n");
		else if (ret == 0)
			LOGIPA(conn, LOGL_ERROR, "connection closed with server\n");
		ipa_server_conn_destroy(conn);
		return -EBADF;
	}
	if (conn->cb)
		return conn->cb(conn, msg);

	return 0;
}

static void ipa_server_conn_write(struct ipa_server_conn *conn)
{
	struct msgb *msg;
	int ret;

	LOGIPA(conn, LOGL_DEBUG, "sending data\n");
	msg = msgb_dequeue(&conn->tx_queue);

	if (!msg) {
		osmo_fd_write_disable(&conn->ofd);
		return;
	}

	ret = send(conn->ofd.fd, msg->data, msg->len, 0);
	if (ret < 0) {
		LOGIPA(conn, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
}

static int ipa_server_conn_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_server_conn *conn = ofd->data;
	int rc = 0;

	LOGP(DLINP, LOGL_DEBUG, "connected read/write\n");
	if (what & OSMO_FD_READ)
		rc = ipa_server_conn_read(conn);
	if (rc != -EBADF && (what & OSMO_FD_WRITE))
		ipa_server_conn_write(conn);

	return 0;
}

struct ipa_server_conn *
ipa_server_conn_create(void *ctx, struct ipa_server_link *link, int fd,
		int (*cb)(struct ipa_server_conn *conn, struct msgb *msg),
		int (*closed_cb)(struct ipa_server_conn *conn), void *data)
{
	struct ipa_server_conn *conn;
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);

	conn = talloc_zero(ctx, struct ipa_server_conn);
	if (conn == NULL) {
		LOGP(DLINP, LOGL_ERROR, "cannot allocate new peer in server, "
			"reason=`%s'\n", strerror(errno));
		return NULL;
	}
	conn->server = link;
	osmo_fd_setup(&conn->ofd, fd, OSMO_FD_READ, ipa_server_conn_cb, conn, 0);
	conn->cb = cb;
	conn->closed_cb = closed_cb;
	conn->data = data;
	INIT_LLIST_HEAD(&conn->tx_queue);

	if (!getpeername(fd, (struct sockaddr *)&sa, &sa_len)) {
		char *str = inet_ntoa(sa.sin_addr);
		conn->addr = talloc_strdup(conn, str);
		conn->port = ntohs(sa.sin_port);
	}

	if (osmo_fd_register(&conn->ofd) < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		talloc_free(conn);
		return NULL;
	}
	return conn;
}

int ipa_server_conn_ccm(struct ipa_server_conn *conn, struct msgb *msg)
{
	struct tlv_parsed tlvp;
	uint8_t msg_type = *(msg->l2h);
	struct ipaccess_unit unit_data = {};
	char *unitid;
	int len, rc;

	/* shared CCM handling on both server and client */
	rc = ipa_ccm_rcvmsg_base(msg, &conn->ofd);
	switch (rc) {
	case -1:
		/* error in IPA CCM processing */
		goto err;
	case 1:
		/* IPA CCM message that was handled in _base */
		return 0;
	case 0:
		/* IPA CCM message that we need to handle */
		break;
	default:
		/* Error */
		LOGIPA(conn, LOGL_ERROR, "Unexpected return from "
		     "ipa_ccm_rcvmsg_base: %d\n", rc);
		goto err;
	}

	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		rc = ipa_ccm_id_resp_parse(&tlvp, (const uint8_t *)msg->l2h+1, msgb_l2len(msg)-1);
		if (rc < 0) {
			LOGIPA(conn, LOGL_ERROR, "IPA CCM RESPonse with "
				"malformed TLVs\n");
			goto err;
		}
		if (!TLVP_PRESENT(&tlvp, IPAC_IDTAG_UNIT)) {
			LOGIPA(conn, LOGL_ERROR, "IPA CCM RESP without "
				"unit ID\n");
			goto err;
		}
		len = TLVP_LEN(&tlvp, IPAC_IDTAG_UNIT);
		if (len < 1) {
			LOGIPA(conn, LOGL_ERROR, "IPA CCM RESP with short"
				"unit ID\n");
			goto err;
		}
		unitid = (char *) TLVP_VAL(&tlvp, IPAC_IDTAG_UNIT);
		unitid[len-1] = '\0';
		ipa_parse_unitid(unitid, &unit_data);

		/* FIXME */
		rc = conn->ccm_cb(conn, msg, &tlvp, &unit_data);
		if (rc < 0)
			goto err;
		break;
	default:
		LOGIPA(conn, LOGL_ERROR, "Unknown IPA message type\n");
		break;
	}
	return 0;
err:
	/* in case of any error, we close the connection */
	ipa_server_conn_destroy(conn);
	return -1;
}

void ipa_server_conn_destroy(struct ipa_server_conn *conn)
{
	/* make the function re-entrant in case closed_cb() below somehow
	 * calls again into this destructor */
	if (conn->ofd.fd == -1)
		return;
	close(conn->ofd.fd);
	conn->ofd.fd = -1;
	msgb_free(conn->pending_msg);
	osmo_fd_unregister(&conn->ofd);
	if (conn->closed_cb)
		conn->closed_cb(conn);
	talloc_free(conn);
}

void ipa_server_conn_send(struct ipa_server_conn *conn, struct msgb *msg)
{
	msgb_enqueue(&conn->tx_queue, msg);
	osmo_fd_write_enable(&conn->ofd);
}

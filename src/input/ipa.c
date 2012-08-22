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
#include <osmocom/gsm/tlv.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>

#include <osmocom/abis/ipa.h>

#define IPA_ALLOC_SIZE 1200

struct msgb *ipa_msg_alloc(int headroom)
{
	struct msgb *nmsg;

	headroom += sizeof(struct ipaccess_head);

	nmsg = msgb_alloc_headroom(1200 + headroom, headroom, "Abis/IP");
	if (!nmsg)
		return NULL;
	return nmsg;
}

void ipa_msg_push_header(struct msgb *msg, uint8_t proto)
{
	struct ipaccess_head *hh;

	msg->l2h = msg->data;
	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh));
	hh->proto = proto;
	hh->len = htons(msgb_l2len(msg));
}

int ipa_msg_recv(int fd, struct msgb **rmsg)
{
	struct msgb *msg;
	struct ipaccess_head *hh;
	int len, ret;

	msg = ipa_msg_alloc(0);
	if (msg == NULL)
		return -ENOMEM;

	/* first read our 3-byte header */
	hh = (struct ipaccess_head *) msg->data;
	ret = recv(fd, msg->data, sizeof(*hh), 0);
	if (ret <= 0) {
		msgb_free(msg);
		return ret;
	} else if (ret != sizeof(*hh)) {
		LOGP(DLINP, LOGL_ERROR, "too small message received\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_put(msg, ret);

	/* then read the length as specified in header */
	msg->l2h = msg->data + sizeof(*hh);
	len = ntohs(hh->len);

	if (len < 0 || IPA_ALLOC_SIZE < len + sizeof(*hh)) {
		LOGP(DLINP, LOGL_ERROR, "bad message length of %d bytes, "
					"received %d bytes\n", len, ret);
		msgb_free(msg);
		return -EIO;
	}

	ret = recv(fd, msg->l2h, len, 0);
	if (ret <= 0) {
		msgb_free(msg);
		return ret;
	} else if (ret < len) {
		LOGP(DLINP, LOGL_ERROR, "trunked message received\n");
		msgb_free(msg);
		return -EIO;
	}
	msgb_put(msg, ret);
	*rmsg = msg;
	return ret;
}

void ipa_client_conn_close(struct ipa_client_conn *link);

static void ipa_client_retry(struct ipa_client_conn *link)
{
	LOGP(DLINP, LOGL_NOTICE, "connection closed\n");
	ipa_client_conn_close(link);
	LOGP(DLINP, LOGL_NOTICE, "retrying in 5 seconds...\n");
	osmo_timer_schedule(&link->timer, 5, 0);
	link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
}

void ipa_client_conn_close(struct ipa_client_conn *link)
{
	osmo_fd_unregister(link->ofd);
	close(link->ofd->fd);
}

static void ipa_client_read(struct ipa_client_conn *link)
{
	struct osmo_fd *ofd = link->ofd;
	struct msgb *msg;
	int ret;

	LOGP(DLINP, LOGL_DEBUG, "message received\n");

	ret = ipa_msg_recv(ofd->fd, &msg);
	if (ret < 0) {
		if (errno == EPIPE || errno == ECONNRESET) {
			LOGP(DLINP, LOGL_ERROR, "lost connection with server\n");
		}
		ipa_client_retry(link);
		return;
	} else if (ret == 0) {
		LOGP(DLINP, LOGL_ERROR, "connection closed with server\n");
		ipa_client_retry(link);
		return;
	}
	if (link->read_cb)
		link->read_cb(link, msg);
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

	LOGP(DLINP, LOGL_DEBUG, "sending data\n");

	if (llist_empty(&link->tx_queue)) {
		ofd->when &= ~BSC_FD_WRITE;
		return 0;
	}
	lh = link->tx_queue.next;
	llist_del(lh);
	msg = llist_entry(lh, struct msgb, list);

	ret = send(link->ofd->fd, msg->data, msg->len, 0);
	if (ret < 0) {
		if (errno == EPIPE || errno == ENOTCONN) {
			ipa_client_retry(link);
		}
		LOGP(DLINP, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
	return 0;
}

static int ipa_client_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_client_conn *link = ofd->data;
	int error, ret;
	socklen_t len = sizeof(error);

	switch(link->state) {
	case IPA_CLIENT_LINK_STATE_CONNECTING:
		ret = getsockopt(ofd->fd, SOL_SOCKET, SO_ERROR, &error, &len);
		if (ret >= 0 && error > 0) {
			ipa_client_retry(link);
			return 0;
		}
		ofd->when &= ~BSC_FD_WRITE;
		LOGP(DLINP, LOGL_NOTICE, "connection done.\n");
		link->state = IPA_CLIENT_LINK_STATE_CONNECTED;
		if (link->connect_cb)
			link->connect_cb(link);
		break;
	case IPA_CLIENT_LINK_STATE_CONNECTED:
		if (what & BSC_FD_READ) {
			LOGP(DLINP, LOGL_DEBUG, "connected read\n");
			ipa_client_read(link);
		}
		if (what & BSC_FD_WRITE) {
			LOGP(DLINP, LOGL_DEBUG, "connected write\n");
			ipa_client_write(link);
		}
		break;
	default:
		break;
	}
        return 0;
}

static void ipa_link_timer_cb(void *data);

struct ipa_client_conn *
ipa_client_conn_create(void *ctx, struct e1inp_ts *ts,
		       int priv_nr, const char *addr, uint16_t port,
		       int (*connect_cb)(struct ipa_client_conn *link),
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

	ipa_link->ofd->when |= BSC_FD_READ | BSC_FD_WRITE;
	ipa_link->ofd->priv_nr = priv_nr;
	ipa_link->ofd->cb = ipa_client_fd_cb;
	ipa_link->ofd->data = ipa_link;
	ipa_link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
	ipa_link->timer.cb = ipa_link_timer_cb;
	ipa_link->timer.data = ipa_link;
	ipa_link->addr = talloc_strdup(ipa_link, addr);
	ipa_link->port = port;
	ipa_link->connect_cb = connect_cb;
	ipa_link->read_cb = read_cb;
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
	int ret;

	ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
			     link->addr, link->port,
			     OSMO_SOCK_F_CONNECT|OSMO_SOCK_F_NONBLOCK);
	if (ret < 0) {
		if (errno != EINPROGRESS)
			return ret;
	}
	link->ofd->fd = ret;
	if (osmo_fd_register(link->ofd) < 0) {
		close(ret);
		return -EIO;
	}
	return 0;
}

static void ipa_link_timer_cb(void *data)
{
	struct ipa_client_conn *link = data;

	LOGP(DLINP, LOGL_NOTICE, "reconnecting.\n");

	switch(link->state) {
	case IPA_CLIENT_LINK_STATE_CONNECTING:
		ipa_client_conn_open(link);
	        break;
	default:
		break;
	}
}

void ipa_client_conn_send(struct ipa_client_conn *link, struct msgb *msg)
{
	msgb_enqueue(&link->tx_queue, msg);
	link->ofd->when |= BSC_FD_WRITE;
}

static int ipa_server_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	int ret;
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);
	struct ipa_server_link *link = ofd->data;

	ret = accept(ofd->fd, (struct sockaddr *)&sa, &sa_len);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "failed to accept from origin "
			"peer, reason=`%s'\n", strerror(errno));
		return ret;
	}
	LOGP(DLINP, LOGL_NOTICE, "accept()ed new link from %s to port %u\n",
		inet_ntoa(sa.sin_addr), link->port);

	if (link->accept_cb)
		link->accept_cb(link, ret);

	return 0;
}

struct ipa_server_link *
ipa_server_link_create(void *ctx, struct e1inp_line *line,
		       const char *addr, uint16_t port,
		       int (*accept_cb)(struct ipa_server_link *link, int fd),
		       void *data)
{
	struct ipa_server_link *ipa_link;

	ipa_link = talloc_zero(ctx, struct ipa_server_link);
	if (!ipa_link)
		return NULL;

	ipa_link->ofd.when |= BSC_FD_READ | BSC_FD_WRITE;
	ipa_link->ofd.cb = ipa_server_fd_cb;
	ipa_link->ofd.data = ipa_link;
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
			     link->addr, link->port, OSMO_SOCK_F_BIND);
	if (ret < 0)
		return ret;

	link->ofd.fd = ret;
	if (osmo_fd_register(&link->ofd) < 0) {
		close(ret);
		return -EIO;
	}
	return 0;
}

void ipa_server_link_close(struct ipa_server_link *link)
{
	osmo_fd_unregister(&link->ofd);
	close(link->ofd.fd);
}

static void ipa_server_conn_read(struct ipa_server_conn *conn)
{
	struct osmo_fd *ofd = &conn->ofd;
	struct msgb *msg;
	int ret;

	LOGP(DLINP, LOGL_DEBUG, "message received\n");

	ret = ipa_msg_recv(ofd->fd, &msg);
	if (ret < 0) {
		if (errno == EPIPE || errno == ECONNRESET) {
			LOGP(DLINP, LOGL_ERROR, "lost connection with server\n");
		}
		ipa_server_conn_destroy(conn);
		return;
	} else if (ret == 0) {
		LOGP(DLINP, LOGL_ERROR, "connection closed with server\n");
		ipa_server_conn_destroy(conn);
		return;
	}
	if (conn->cb)
		conn->cb(conn, msg);

	return;
}

static void ipa_server_conn_write(struct ipa_server_conn *conn)
{
	struct osmo_fd *ofd = &conn->ofd;
	struct msgb *msg;
	struct llist_head *lh;
	int ret;

	LOGP(DLINP, LOGL_DEBUG, "sending data\n");

	if (llist_empty(&conn->tx_queue)) {
		ofd->when &= ~BSC_FD_WRITE;
		return;
	}
	lh = conn->tx_queue.next;
	llist_del(lh);
	msg = llist_entry(lh, struct msgb, list);

	ret = send(conn->ofd.fd, msg->data, msg->len, 0);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
}

static int ipa_server_conn_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_server_conn *conn = ofd->data;

	LOGP(DLINP, LOGL_DEBUG, "connected read/write\n");
	if (what & BSC_FD_READ)
		ipa_server_conn_read(conn);
	if (what & BSC_FD_WRITE)
		ipa_server_conn_write(conn);

	return 0;
}

struct ipa_server_conn *
ipa_server_conn_create(void *ctx, struct ipa_server_link *link, int fd,
		int (*cb)(struct ipa_server_conn *conn, struct msgb *msg),
		int (*closed_cb)(struct ipa_server_conn *conn), void *data)
{
	struct ipa_server_conn *conn;

	conn = talloc_zero(ctx, struct ipa_server_conn);
	if (conn == NULL) {
		LOGP(DLINP, LOGL_ERROR, "cannot allocate new peer in server, "
			"reason=`%s'\n", strerror(errno));
		return NULL;
	}
	conn->server = link;
	conn->ofd.fd = fd;
	conn->ofd.data = conn;
	conn->ofd.cb = ipa_server_conn_cb;
	conn->ofd.when = BSC_FD_READ;
	conn->cb = cb;
	conn->closed_cb = closed_cb;
	conn->data = data;
	INIT_LLIST_HEAD(&conn->tx_queue);

	if (osmo_fd_register(&conn->ofd) < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		talloc_free(conn);
		return NULL;
	}
	return conn;
}

void ipa_server_conn_destroy(struct ipa_server_conn *conn)
{
	close(conn->ofd.fd);
	osmo_fd_unregister(&conn->ofd);
	if (conn->closed_cb)
		conn->closed_cb(conn);
	talloc_free(conn);
}

void ipa_server_conn_send(struct ipa_server_conn *conn, struct msgb *msg)
{
	msgb_enqueue(&conn->tx_queue, msg);
	conn->ofd.when |= BSC_FD_WRITE;
}

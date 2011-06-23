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
#include <talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>
#include <osmocom/abis/logging.h>

#include <osmocom/abis/ipa.h>

#define IPA_ALLOC_SIZE 1200

static struct msgb *ipa_msg_alloc(void)
{
	return msgb_alloc(IPA_ALLOC_SIZE, "Abis/IP");
}

int ipa_msg_recv(int fd, struct msgb **rmsg)
{
	struct msgb *msg;
	struct ipaccess_head *hh;
	int len, ret;

	msg = ipa_msg_alloc();
	if (msg == NULL)
		return -ENOMEM;

	/* first read our 3-byte header */
	hh = (struct ipaccess_head *) msg->data;
	ret = recv(fd, msg->data, sizeof(*hh), 0);
	if (ret <= 0) {
		msgb_free(msg);
		return ret;
	} else if (ret != sizeof(*hh)) {
		msgb_free(msg);
		return -EIO;
	}
	msgb_put(msg, ret);

	/* then read the length as specified in header */
	msg->l2h = msg->data + sizeof(*hh);
	len = ntohs(hh->len);

	if (len < 0 || IPA_ALLOC_SIZE < len + sizeof(*hh)) {
		msgb_free(msg);
		return -EIO;
	}

	ret = recv(fd, msg->l2h, len, 0);
	if (ret <= 0) {
		msgb_free(msg);
		return ret;
	} else if (ret < len) {
		msgb_free(msg);
		return -EIO;
	}
	msgb_put(msg, ret);
	*rmsg = msg;
	return ret;
}

void ipa_client_link_close(struct ipa_client_link *link);

static void ipa_client_retry(struct ipa_client_link *link)
{
	LOGP(DINP, LOGL_NOTICE, "connection closed\n");
	ipa_client_link_close(link);
	LOGP(DINP, LOGL_NOTICE, "retrying in 5 seconds...\n");
	osmo_timer_schedule(&link->timer, 5, 0);
	link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
}

void ipa_client_link_close(struct ipa_client_link *link)
{
	osmo_fd_unregister(&link->ofd);
	close(link->ofd.fd);
}

static void ipa_client_read(struct ipa_client_link *link)
{
	struct osmo_fd *ofd = &link->ofd;
	struct msgb *msg;
	int ret;

	LOGP(DINP, LOGL_NOTICE, "message received\n");

	ret = ipa_msg_recv(ofd->fd, &msg);
	if (ret < 0) {
		if (errno == EPIPE || errno == ECONNRESET) {
			LOGP(DINP, LOGL_ERROR, "lost connection with server\n");
		} else {
			LOGP(DINP, LOGL_ERROR, "unknown error\n");
		}
		ipa_client_retry(link);
		return;
	} else if (ret == 0) {
		LOGP(DINP, LOGL_ERROR, "connection closed with server\n");
		ipa_client_retry(link);
		return;
	}
	if (link->cb)
		link->cb(link, msg);
}

static void ipa_client_write(struct ipa_client_link *link)
{
	struct osmo_fd *ofd = &link->ofd;
	struct msgb *msg;
	struct llist_head *lh;
	int ret;

	LOGP(DINP, LOGL_NOTICE, "sending data\n");

	if (llist_empty(&link->tx_queue)) {
		ofd->when &= ~BSC_FD_WRITE;
		return;
	}
	lh = link->tx_queue.next;
	llist_del(lh);
	msg = llist_entry(lh, struct msgb, list);

	ret = send(link->ofd.fd, msg->data, msg->len, 0);
	if (ret < 0) {
		if (errno == EPIPE || errno == ENOTCONN) {
			ipa_client_retry(link);
		}
		LOGP(DINP, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
}

int ipa_client_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_client_link *link = ofd->data;
	int error, ret;
	size_t len = sizeof(error);

	switch(link->state) {
	case IPA_CLIENT_LINK_STATE_CONNECTING:
		ret = getsockopt(ofd->fd, SOL_SOCKET, SO_ERROR, &error, &len);
		if (ret >= 0 && error > 0) {
			ipa_client_retry(link);
			return 0;
		}
		ofd->when &= ~BSC_FD_WRITE;
		LOGP(DINP, LOGL_NOTICE, "connection done.\n");
		link->state = IPA_CLIENT_LINK_STATE_CONNECTED;
		break;
	case IPA_CLIENT_LINK_STATE_CONNECTED:
		LOGP(DINP, LOGL_NOTICE, "connected read/write\n");
		if (what & BSC_FD_READ)
			ipa_client_read(link);
		if (what & BSC_FD_WRITE)
			ipa_client_write(link);
		break;
	default:
		break;
	}
        return 0;
}

static void ipa_link_timer_cb(void *data);

struct ipa_client_link *
ipa_client_link_create(void *ctx, struct e1inp_line *line,
		       const char *addr, uint16_t port,
		       int (*cb)(struct ipa_client_link *link,
				 struct msgb *msgb), void *data)
{
	struct ipa_client_link *ipa_link;

	ipa_link = talloc_zero(ctx, struct ipa_client_link);
	if (!ipa_link)
		return NULL;

	ipa_link->ofd.when |= BSC_FD_READ | BSC_FD_WRITE;
	ipa_link->ofd.cb = ipa_client_fd_cb;
	ipa_link->ofd.data = ipa_link;
	ipa_link->state = IPA_CLIENT_LINK_STATE_CONNECTING;
	ipa_link->timer.cb = ipa_link_timer_cb;
	ipa_link->timer.data = ipa_link;
	ipa_link->addr = talloc_strdup(ipa_link, addr);
	ipa_link->port = port;
	ipa_link->cb = cb;
	ipa_link->line = line;
	ipa_link->data = data;

	return ipa_link;
}

void ipa_client_link_destroy(struct ipa_client_link *link)
{
	talloc_free(link);
}

int ipa_client_link_open(struct ipa_client_link *link)
{
	int ret;

	ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
			     link->addr, link->port,
			     OSMO_SOCK_F_CONNECT|OSMO_SOCK_F_NONBLOCK);
	if (ret < 0) {
		if (errno != EINPROGRESS)
			return ret;
	}
	link->ofd.fd = ret;
	if (osmo_fd_register(&link->ofd) < 0) {
		close(ret);
		return -EIO;
	}
	return 0;
}

static void ipa_link_timer_cb(void *data)
{
	struct ipa_client_link *link = data;

	LOGP(DINP, LOGL_NOTICE, "reconnecting.\n");

	switch(link->state) {
	case IPA_CLIENT_LINK_STATE_CONNECTING:
		ipa_client_link_open(link);
	        break;
	default:
		break;
	}
}

int ipa_server_fd_cb(struct osmo_fd *ofd, unsigned int what)
{
	int ret;
	struct sockaddr_in sa;
	socklen_t sa_len = sizeof(sa);
	struct ipa_server_link *link = ofd->data;

	ret = accept(ofd->fd, (struct sockaddr *)&sa, &sa_len);
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "failed to accept from origin "
			"peer, reason=`%s'\n", strerror(errno));
		return ret;
	}
	LOGP(DINP, LOGL_NOTICE, "accept()ed new link from %s to port %u\n",
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

static void ipa_server_peer_read(struct ipa_server_peer *peer)
{
	struct osmo_fd *ofd = &peer->ofd;
	struct msgb *msg;
	int ret;

	LOGP(DINP, LOGL_NOTICE, "message received\n");

	ret = ipa_msg_recv(ofd->fd, &msg);
	if (ret < 0) {
		if (errno == EPIPE || errno == ECONNRESET) {
			LOGP(DINP, LOGL_ERROR, "lost connection with server\n");
		} else {
			LOGP(DINP, LOGL_ERROR, "unknown error\n");
		}
		return;
	} else if (ret == 0) {
		LOGP(DINP, LOGL_ERROR, "connection closed with server\n");
		ipa_server_peer_destroy(peer);
		return;
	}
	if (peer->cb)
		peer->cb(peer, msg);

	return;
}

static void ipa_server_peer_write(struct ipa_server_peer *peer)
{
	struct osmo_fd *ofd = &peer->ofd;
	struct msgb *msg;
	struct llist_head *lh;
	int ret;

	LOGP(DINP, LOGL_NOTICE, "sending data\n");

	if (llist_empty(&peer->tx_queue)) {
		ofd->when &= ~BSC_FD_WRITE;
		return;
	}
	lh = peer->tx_queue.next;
	llist_del(lh);
	msg = llist_entry(lh, struct msgb, list);

	ret = send(peer->ofd.fd, msg->data, msg->len, 0);
	if (ret < 0) {
		LOGP(DINP, LOGL_ERROR, "error to send\n");
	}
	msgb_free(msg);
}

static int ipa_server_peer_cb(struct osmo_fd *ofd, unsigned int what)
{
	struct ipa_server_peer *peer = ofd->data;

	LOGP(DINP, LOGL_NOTICE, "connected read/write\n");
	if (what & BSC_FD_READ)
		ipa_server_peer_read(peer);
	if (what & BSC_FD_WRITE)
		ipa_server_peer_write(peer);

	return 0;
}

struct ipa_server_peer *
ipa_server_peer_create(void *ctx, struct ipa_server_link *link, int fd,
		int (*cb)(struct ipa_server_peer *peer, struct msgb *msg),
		void *data)
{
	struct ipa_server_peer *peer;

	peer = talloc_zero(ctx, struct ipa_server_peer);
	if (peer == NULL) {
		LOGP(DINP, LOGL_ERROR, "cannot allocate new peer in server, "
			"reason=`%s'\n", strerror(errno));
		return NULL;
	}
	peer->server = link;
	peer->ofd.fd = fd;
	peer->ofd.data = peer;
	peer->ofd.cb = ipa_server_peer_cb;
	peer->ofd.when = BSC_FD_READ;
	peer->cb = cb;
	peer->data = data;
	if (osmo_fd_register(&peer->ofd) < 0) {
		LOGP(DINP, LOGL_ERROR, "could not register FD\n");
		talloc_free(peer);
		return NULL;
	}
	return peer;
}

void ipa_server_peer_destroy(struct ipa_server_peer *peer)
{
	close(peer->ofd.fd);
	osmo_fd_unregister(&peer->ofd);
	talloc_free(peer);
}

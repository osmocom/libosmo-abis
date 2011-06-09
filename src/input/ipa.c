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

void ipa_client_link_close(struct ipa_link *link);

static void ipa_client_retry(struct ipa_link *link)
{
	LOGP(DINP, LOGL_NOTICE, "connection closed\n");
	ipa_client_link_close(link);
	LOGP(DINP, LOGL_NOTICE, "retrying in 5 seconds...\n");
	osmo_timer_schedule(&link->timer, 5, 0);
	link->state = IPA_LINK_STATE_CONNECTING;
}

void ipa_client_link_close(struct ipa_link *link)
{
	osmo_fd_unregister(&link->ofd);
	close(link->ofd.fd);
}

static void ipa_client_read(struct ipa_link *link)
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
	if (link->process)
		link->process(link, msg);
}

static void ipa_client_write(struct ipa_link *link)
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
	struct ipa_link *link = ofd->data;
	int error, ret;
	size_t len = sizeof(error);

	switch(link->state) {
	case IPA_LINK_STATE_CONNECTING:
		ret = getsockopt(ofd->fd, SOL_SOCKET, SO_ERROR, &error, &len);
		if (ret >= 0 && error > 0) {
			ipa_client_retry(link);
			return 0;
		}
		ofd->when &= ~BSC_FD_WRITE;
		LOGP(DINP, LOGL_NOTICE, "connection done.\n");
		link->state = IPA_LINK_STATE_CONNECTED;
		break;
	case IPA_LINK_STATE_CONNECTED:
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

struct ipa_link *ipa_client_link_create(void *ctx)
{
	struct ipa_link *ipa_link;

	ipa_link = talloc_zero(ctx, struct ipa_link);
	if (!ipa_link)
		return NULL;

	ipa_link->ofd.when |= BSC_FD_READ | BSC_FD_WRITE;
	ipa_link->ofd.cb = ipa_client_fd_cb;
	ipa_link->ofd.data = ipa_link;
	ipa_link->state = IPA_LINK_STATE_CONNECTING;
	ipa_link->timer.cb = ipa_link_timer_cb;
	ipa_link->timer.data = ipa_link;

	return ipa_link;
}

void ipa_client_link_destroy(struct ipa_link *link)
{
	talloc_free(link);
}

int ipa_client_link_open(struct ipa_link *link)
{
	int ret;

	ret = osmo_sock_init(AF_INET, SOCK_STREAM, IPPROTO_TCP,
			     "127.0.0.1", IPA_TCP_PORT_OML,
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
	struct ipa_link *link = data;

	LOGP(DINP, LOGL_NOTICE, "reconnecting.\n");

	switch(link->state) {
	case IPA_LINK_STATE_CONNECTING:
		ipa_client_link_open(link);
	        break;
	default:
		break;
	}
}

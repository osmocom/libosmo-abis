#ifndef _OSMO_IPA_H_
#define _OSMO_IPA_H_

#include <stdint.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/timer.h>

struct ipa_server_link {
	struct e1inp_line		*line;
	struct osmo_fd			ofd;
	struct llist_head		tx_queue;
	const char			*addr;
	uint16_t			port;
	int (*accept_cb)(struct ipa_server_link *link, int fd);
	void				*data;
};

struct ipa_server_link *ipa_server_link_create(void *ctx, struct e1inp_line *line, const char *addr, uint16_t port, int (*accept_cb)(struct ipa_server_link *link, int fd), void *data);
void ipa_server_link_destroy(struct ipa_server_link *link);

int ipa_server_link_open(struct ipa_server_link *link);
void ipa_server_link_close(struct ipa_server_link *link);

enum ipa_client_link_state {
	IPA_CLIENT_LINK_STATE_NONE         = 0,
	IPA_CLIENT_LINK_STATE_CONNECTING   = 1,
	IPA_CLIENT_LINK_STATE_CONNECTED    = 2,
	IPA_CLIENT_LINK_STATE_MAX
};

struct ipa_client_link {
	struct e1inp_line		*line;
	struct osmo_fd			ofd;
	struct llist_head		tx_queue;
	struct osmo_timer_list		timer;
	enum ipa_client_link_state	state;
	const char			*addr;
	uint16_t			port;
	int (*cb)(struct ipa_client_link *link, struct msgb *msg);
	void				*data;
};

struct ipa_client_link *ipa_client_link_create(void *ctx, struct e1inp_line *line, const char *addr, uint16_t port, int (*cb)(struct ipa_client_link *link, struct msgb *msgb), void *data);
void ipa_client_link_destroy(struct ipa_client_link *link);

int ipa_client_link_open(struct ipa_client_link *link);
void ipa_client_link_close(struct ipa_client_link *link);

int ipa_msg_recv(int fd, struct msgb **rmsg);

#endif

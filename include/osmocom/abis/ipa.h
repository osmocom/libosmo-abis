#ifndef _OSMO_IPA_H_
#define _OSMO_IPA_H_

#include <stdint.h>
#include <osmocom/core/linuxlist.h>
#include <osmocom/core/timer.h>

enum ipa_link_state {
	IPA_LINK_STATE_NONE         = 0,
	IPA_LINK_STATE_CONNECTING   = 1,
	IPA_LINK_STATE_CONNECTED    = 2,
	IPA_LINK_STATE_MAX
};

struct ipa_link {
	struct e1inp_line	*line;
	struct osmo_fd		ofd;
	struct llist_head	tx_queue;
	struct osmo_timer_list	timer;
	enum ipa_link_state	state;
	const char		*addr;
	uint16_t		port;
	int (*cb)(struct ipa_link *link, struct msgb *msg);
};

struct ipa_link *ipa_client_link_create(void *ctx, struct e1inp_line *line, const char *addr, uint16_t port, int (*cb)(struct ipa_link *link, struct msgb *msgb));
void ipa_client_link_destroy(struct ipa_link *link);

int ipa_client_link_open(struct ipa_link *link);
void ipa_client_link_close(struct ipa_link *link);

int ipa_msg_recv(int fd, struct msgb **rmsg);

#endif

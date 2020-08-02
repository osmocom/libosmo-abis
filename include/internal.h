#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#include <stdint.h>

/* Amount of data to write to a B-channel in every write() call */
#define D_BCHAN_TX_GRAN 160

struct osmo_fd;
struct e1inp_sign_link;
struct e1inp_ts;

/* talloc context for libosmo-abis. */
extern void *libosmo_abis_ctx;

/* use libosmo_abis_init, this is only for internal use. */
void e1inp_init(void);

void e1inp_ipa_set_bind_addr(const char *ip_bind_addr);
const char *e1inp_ipa_get_bind_addr(void);

/* ipaccess.c requires these functions defined here */
struct msgb;
struct msgb *ipa_msg_alloc(int headroom);

int e1inp_int_snd_event(struct e1inp_ts *ts, struct e1inp_sign_link *link, int evt);


#endif

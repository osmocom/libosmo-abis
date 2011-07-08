#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#include <stdint.h>

/* talloc context for libosmo-abis. */
extern void *libosmo_abis_ctx;

/* use libosmo_abis_init, this is only for internal use. */
void e1inp_init(void);

/* hsl requires these functions defined in ipaccess driver. */
struct msgb;
void ipaccess_prepend_header(struct msgb *msg, int proto);
struct msgb *ipa_msg_alloc(int headroom);
void ipa_msg_push_header(struct msgb *msg, uint8_t proto);

#endif

#ifndef _INTERNAL_H_
#define _INTERNAL_H_

/* XXX: fix this in libosmocore, we need some reserved range */
#define IPA_NODE _LAST_OSMOVTY_NODE + 100
#define E1INP_NODE _LAST_OSMOVTY_NODE + 101

/* talloc context for libosmo-abis. */
extern void *libosmo_abis_ctx;

/* use libosmo_abis_init, this is only for internal use. */
void e1inp_init(void);

/* hsl requires these functions defined in ipaccess driver. */
struct msgb;
void ipaccess_prepend_header(struct msgb *msg, int proto);

#endif

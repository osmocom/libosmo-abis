#ifndef _INTERNAL_H_
#define _INTERNAL_H_

/* talloc context for libosmo-abis. */
extern void *libosmo_abis_ctx;

/* use libosmo_abis_init, this is only for internal use. */
void e1inp_init(void);

/* hsl requires these functions defined in ipaccess driver. */
struct osmo_fd;
struct msgb *ipaccess_read_msg(struct osmo_fd *bfd, int *error);
void ipaccess_prepend_header(struct msgb *msg, int proto);

#endif

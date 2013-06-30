#ifndef _OSMO_ABIS_IPACCESS_H
#define _OSMO_ABIS_IPACCESS_H

#include <stdint.h>
#include <osmocom/gsm/protocol/ipaccess.h>

struct ipaccess_unit {
	uint16_t site_id;
	uint16_t bts_id;
	uint16_t trx_id;
	char *unit_name;
	char *equipvers;
	char *swversion;
	uint8_t mac_addr[6];
	char *location1;
	char *location2;
	char *serno;
};

/* quick solution to get openBSC's ipaccess tools working. */
extern int ipaccess_fd_cb(struct osmo_fd *bfd, unsigned int what);

#endif /* _OSMO_ABIS_IPACCESS_H */

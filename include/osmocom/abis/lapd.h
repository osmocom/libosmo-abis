#ifndef OPENBSC_LAPD_H
#define OPENBSC_LAPD_H

#include <stdint.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/gsm/lapd_core.h>

enum lapd_profile {
	LAPD_PROFILE_ISDN,
	LAPD_PROFILE_ABIS,
	LAPD_PROFILE_ASAT,
};

struct lapd_instance {
	struct llist_head list;		/* list of LAPD instances */
	int network_side;

	void (*transmit_cb)(struct msgb *msg, void *cbdata);
	void *transmit_cbdata;
	void (*receive_cb)(struct osmo_dlsap_prim *odp, uint8_t tei,
		uint8_t sapi, void *rx_cbdata);
	void *receive_cbdata;

	enum lapd_profile profile;
	uint8_t short_address;

	struct llist_head tei_list;	/* list of TEI in this LAPD instance */
};

enum lapd_recv_errors {
	LAPD_ERR_NONE = 0,
	LAPD_ERR_BAD_LEN,
	LAPD_ERR_BAD_ADDR,
	LAPD_ERR_UNKNOWN_S_CMD,
	LAPD_ERR_UNKNOWN_U_CMD,
	LAPD_ERR_UNKNOWN_TEI,
	LAPD_ERR_BAD_CMD,
	LAPD_ERR_NO_MEM,
	__LAPD_ERR_MAX
};

int lapd_receive(struct lapd_instance *li, struct msgb *msg, int *error);

void lapd_transmit(struct lapd_instance *li, uint8_t tei, uint8_t sapi,
		   struct msgb *msg);

struct lapd_instance *lapd_instance_alloc(int network_side,
	void (*tx_cb)(struct msgb *msg, void *cbdata), void *tx_cbdata,
	void (*rx_cb)(struct osmo_dlsap_prim *odp, uint8_t tei, uint8_t sapi, 
			void *rx_cbdata), void *rx_cbdata,
	enum lapd_profile profile);

void lapd_instance_free(struct lapd_instance *li);

/* Start a (user-side) SAP for the specified TEI/SAPI on the LAPD instance */
int lapd_sap_start(struct lapd_instance *li, uint8_t tei, uint8_t sapi);

/* Stop a (user-side) SAP for the specified TEI/SAPI on the LAPD instance */
int lapd_sap_stop(struct lapd_instance *li, uint8_t tei, uint8_t sapi);

#endif /* OPENBSC_LAPD_H */

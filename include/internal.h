#ifndef _INTERNAL_H_
#define _INTERNAL_H_

/* talloc context for libosmo-abis. */
extern void *libosmo_abis_ctx;

/* use libosmo_abis_init, this is only for internal use. */
void e1inp_init(void);

/* things I don't know what to do with yet. */

/* extracted from include/openbsc/debug.h. */
#define BSC_CTX_BTS     2

/* from include/openbsc/signal.h, we need SS_INPUT and S_GLOBAL_SHUTDOWN. */
enum signal_subsystems {
	SS_PAGING,
	SS_SMS,
	SS_ABISIP,
	SS_NM,
	SS_LCHAN,
	SS_SUBSCR,
	SS_SCALL,
	SS_GLOBAL,
	SS_CHALLOC,
	SS_NS,
	SS_IPAC_NWL,
	SS_RF,
	SS_MSC,
	SS_HO,
	SS_INPUT,
};

enum signal_global {
	S_GLOBAL_SHUTDOWN,
	S_GLOBAL_BTS_CLOSE_OM,
};

/* from include/openbsc/vty.h, we need E1INP_NODE */
#include <osmocom/vty/vty.h>
#include <osmocom/vty/buffer.h>
#include <osmocom/vty/command.h>

enum bsc_vty_node {
	GSMNET_NODE = _LAST_OSMOVTY_NODE + 1,
	BTS_NODE,
	TRX_NODE,
	TS_NODE,
	SUBSCR_NODE,
	MGCP_NODE,
	GBPROXY_NODE,
	SGSN_NODE,
	NS_NODE,
	BSSGP_NODE,
	OML_NODE,
	E1INP_NODE,
	NAT_NODE,
	NAT_BSC_NODE,
	MSC_NODE,
	OM2K_NODE,
	TRUNK_NODE,
	PGROUP_NODE,
};

/* from include/openbsc/debug.h */
enum {
        DRLL,
        DCC,
        DMM,
        DRR,
        DRSL,
        DNM,
        DMNCC,
        DSMS,
        DPAG,
        DMEAS,
        DMI,
        DMIB,
        DMUX,
        DINP,
        DSCCP,
        DMSC,
        DMGCP,
        DHO,
        DDB,
        DREF,
        DGPRS,
        DNS,
        DBSSGP,
        DLLC,
        DSNDCP,
        DNAT,
        Debug_LastEntry,
};

struct osmo_fd;
struct msgb *ipaccess_read_msg(struct osmo_fd *bfd, int *error);
void ipaccess_prepend_header(struct msgb *msg, int proto);

#include <stdint.h>

int make_sock(struct osmo_fd *bfd, int proto,
              uint32_t ip, uint16_t port, int priv_nr,
              int (*cb)(struct osmo_fd *fd, unsigned int what), void *data);

uint8_t *trau_idle_frame(void);

#endif

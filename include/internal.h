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

/* things I don't know what to do with yet. */

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

#endif

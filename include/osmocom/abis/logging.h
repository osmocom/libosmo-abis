#ifndef _OSMO_ABIS_LOGGING_H_
#define _OSMO_ABIS_LOGGING_H_

#include <osmocom/core/logging.h>

/* logging subsystems. */
enum {
	DINP	= OSMO_LOG_SS_ABIS_RESERVED,
	DMUX,
	DMI,
	DMIB,
	DRSL,
	DNM,
};

#endif

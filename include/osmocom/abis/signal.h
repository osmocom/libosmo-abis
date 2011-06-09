#ifndef _OSMO_ABIS_SIGNAL_H_
#define _OSMO_ABIS_SIGNAL_H_

#include <osmocom/core/signal.h>

/* signal subsystems. */
enum {
	SS_GLOBAL		= OSMO_SIGNAL_SS_ABIS_RESERVED,
	SS_INPUT,
};

/* signal types. */
enum {
	S_GLOBAL_SHUTDOWN	= OSMO_SIGNAL_T_ABIS_RESERVED,
};

#endif

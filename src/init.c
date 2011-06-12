/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "internal.h"
#include <osmocom/core/utils.h>
#include <osmocom/abis/logging.h>
#include <talloc.h>

void *libosmo_abis_ctx;

struct log_info_cat libosmo_abis_default_categories[] = {
	[DINP] = {
		.name = "DINP",
		.description = "A-bis Intput Subsystem",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DMUX] = {
		.name = "DMUX",
		.description = "A-bis B-Subchannel TRAU Frame Multiplex",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DMI] = {
		.name = "DMI",
		.description = "A-bis Input Driver for Signalling",
		.enabled = 0, .loglevel = LOGL_NOTICE,
	},
	[DMIB] = {
		.name = "DMIB",
		.description = "A-bis Input Driver for B-Channels (voice)",
		.enabled = 0, .loglevel = LOGL_NOTICE,
	},
	[DRSL] = {
		.name = "DRSL",
		.description = "A-bis Radio Siganlling Link (RSL)",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
	[DNM] = {
		.name = "DNM",
		.description = "A-bis Network Management / O&M (NM/OML)",
		.color = "\033[1;36m",
		.enabled = 1, .loglevel = LOGL_INFO,
	},
};

const struct log_info libosmo_abis_log_info = {
	.filter_fn = NULL,	/* the application should set this. */
	.cat = libosmo_abis_default_categories,
	.num_cat = ARRAY_SIZE(libosmo_abis_default_categories),
};

void libosmo_abis_init(void *ctx)
{
	log_init(&libosmo_abis_log_info);
	libosmo_abis_ctx = talloc_named_const(ctx, 0, "abis");
	e1inp_init();
}

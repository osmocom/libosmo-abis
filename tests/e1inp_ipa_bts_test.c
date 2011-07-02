#include <stdio.h>
#include <talloc.h>
#include <string.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/abis/ipaccess.h>

static void *tall_test;
static struct e1inp_sign_link *oml_sign_link, *rsl_sign_link;

#define DBTSTEST OSMO_LOG_SS_APPS

struct log_info_cat bts_test_cat[] = {
	[DBTSTEST] = {
		.name = "DBTSTEST",
		.description = "BTS-mode test",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

const struct log_info bts_test_log_info = {
	.filter_fn = NULL,
	.cat = bts_test_cat,
	.num_cat = ARRAY_SIZE(bts_test_cat),
};

static struct e1inp_sign_link *
sign_link_up(void *unit, struct e1inp_line *line, enum e1inp_sign_type type)
{
	struct e1inp_sign_link *sign_link = NULL;
	void *dst = NULL;

	switch(type) {
	case E1INP_SIGN_OML:
		LOGP(DBTSTEST, LOGL_NOTICE, "OML link up request received.\n");

		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML - 1], line);
		sign_link = oml_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_OML - 1],
						E1INP_SIGN_OML, NULL, 255, 0);
		if (!oml_sign_link) {
			LOGP(DBTSTEST, LOGL_ERROR,
				"cannot create OML sign link\n");
		}
		dst = oml_sign_link;
		break;
	case E1INP_SIGN_RSL:
		LOGP(DBTSTEST, LOGL_NOTICE, "RSL link up request received.\n");

		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_RSL - 1], line);

		sign_link = rsl_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_RSL - 1],
						E1INP_SIGN_RSL, NULL, 0, 0);
		if (!rsl_sign_link) {
			LOGP(DBTSTEST, LOGL_ERROR,
				"cannot create RSL sign link\n");
		}
		dst = rsl_sign_link;
		break;
	default:
		return NULL;
	}
	if (sign_link)
		LOGP(DBTSTEST, LOGL_NOTICE, "signal link has been set up.\n");

	return sign_link;
}

static void sign_link_down(struct e1inp_line *line)
{
	LOGP(DBTSTEST, LOGL_NOTICE, "signal link has been closed\n");
	e1inp_sign_link_destroy(oml_sign_link);
	e1inp_sign_link_destroy(rsl_sign_link);
}

static int sign_link(struct msgb *msg, struct e1inp_sign_link *link)
{
	LOGP(DBTSTEST, LOGL_NOTICE, "OML/RSL message received.\n");
	return 0;
}

int main(void)
{
	struct ipaccess_unit bts_dev_info = {
		.site_id	= 0,
		.bts_id		= 0,
		.trx_id		= 0,
		.unit_name	= "testBTS",
		.equipvers	= "0.1",
		.swversion	= "0.1",
		.mac_addr	= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
		.location1	= "testBTS",
		.location2	= "testBTS",
		.serno		= "",
	};
	struct e1inp_line *line;

	tall_test = talloc_named_const(NULL, 1, "e1inp_test");
	libosmo_abis_init(tall_test);

	osmo_init_logging(&bts_test_log_info);

	struct e1inp_line_ops ops = {
		.role		= E1INP_LINE_R_BTS,
		.addr		= "127.0.0.1",
		.data		= &bts_dev_info,
		.sign_link_up	= sign_link_up,
		.sign_link_down	= sign_link_down,
		.sign_link	= sign_link,
	};

#define LINENR 0

	line = e1inp_line_create(LINENR, "ipa");
	if (line == NULL) {
		LOGP(DBTSTEST, LOGL_ERROR, "problem enabling E1 line\n");
		exit(EXIT_FAILURE);
	}
	e1inp_line_bind_ops(line, &ops);

	if (e1inp_line_update(line) < 0) {
		LOGP(DBTSTEST, LOGL_ERROR, "problem enabling E1 line\n");
		exit(EXIT_FAILURE);
	}

	LOGP(DBTSTEST, LOGL_NOTICE, "entering main loop\n");

	while (1) {
		osmo_select_main(0);
	}
	return 0;
}

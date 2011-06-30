#include <stdio.h>
#include <talloc.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/core/application.h>
#include <osmocom/core/logging.h>

static void *tall_test;
static struct e1inp_sign_link *oml_sign_link, *rsl_sign_link;

static struct e1inp_sign_link *
sign_link_up(void *dev, struct e1inp_line *line, enum e1inp_sign_type type)
{
	struct e1inp_sign_link *sign_link = NULL;

	switch(type) {
	case E1INP_SIGN_OML:
		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML - 1], line);
		sign_link = oml_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_OML - 1],
					       E1INP_SIGN_OML, NULL, 255, 0);
		break;
	case E1INP_SIGN_RSL:
		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_RSL - 1], line);
		sign_link = rsl_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_RSL - 1],
					       E1INP_SIGN_OML, NULL, 0, 0);
		break;
	default:
		break;
	}
	return sign_link;
}

static void sign_link_down(struct e1inp_line *line)
{
	printf("link got down.\n");
	e1inp_sign_link_destroy(oml_sign_link);
	e1inp_sign_link_destroy(rsl_sign_link);
}

static int sign_link(struct msgb *msg, struct e1inp_sign_link *link)
{
	printf("OML/RSL data received\n");
	return 0;
}

#define DBSCTEST OSMO_LOG_SS_APPS

struct log_info_cat bsc_test_cat[] = {
	[DBSCTEST] = {
		.name = "DBSCTEST",
		.description = "BSC-mode test",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

const struct log_info bsc_test_log_info = {
        .filter_fn = NULL,
        .cat = bsc_test_cat,
        .num_cat = ARRAY_SIZE(bsc_test_cat),
};

int main(void)
{
	struct e1inp_line *line;

	tall_test = talloc_named_const(NULL, 1, "e1inp_test");
	libosmo_abis_init(tall_test);

	osmo_init_logging(&bsc_test_log_info);

	struct e1inp_line_ops ops = {
		.addr		= "0.0.0.0",
		.role		= E1INP_LINE_R_BSC,
		.sign_link_up	= sign_link_up,
		.sign_link_down	= sign_link_down,
		.sign_link	= sign_link,
	};

#define LINENR 0

	line = e1inp_line_create(LINENR, "ipa");
	if (line == NULL) {
		LOGP(DBSCTEST, LOGL_ERROR, "problem creating E1 line\n");
		exit(EXIT_FAILURE);
	}

	e1inp_line_bind_ops(line, &ops);

	/*
	 * Depending if this is a real or virtual E1 lines:
	 * - real (ISDN): create signal link for OML and RSL before line up.
	 * - vitual (INET): we create it in signal_link_up(...) callback.
	 *
	 * The signal link is created via e1inp_sign_link_create(...)
	 *
	 * See e1_reconfig_trx and e1_reconfig_bts in libbsc/e1_config.c,
	 * it explains how this is done with ISDN.
	 */

	if (e1inp_line_update(line) < 0) {
		LOGP(DBSCTEST, LOGL_ERROR, "problem creating E1 line\n");
		exit(EXIT_FAILURE);
	}

	LOGP(DBSCTEST, LOGL_NOTICE, "entering main loop\n");

	while (1) {
		osmo_select_main(0);
	}
	return 0;
}

#include <stdio.h>
#include <talloc.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>

static void *tall_test;

static int sign_link_up(struct msgb *msg, struct e1inp_line *line)
{
	printf("ID_RESP received, create sign link.\n");
	return 0;
}

static int sign_link(struct msgb *msg, struct e1inp_sign_link *link)
{
	printf("OML/RSL data received\n");
	return 0;
}

static int error(struct msgb *msg, int error)
{
	printf("error, malformed message\n");
	return 0;
}

int main(void)
{
	struct e1inp_line *line;

	tall_test = talloc_named_const(NULL, 1, "e1inp_test");
	libosmo_abis_init(tall_test);

	struct e1inp_line_ops ops = {
		.sign_link_up	= sign_link_up,
		.sign_link	= sign_link,
		.error		= error,
	};

#define LINENR 0

	line = e1inp_line_create(LINENR, "ipa", &ops);
	if (line == NULL) {
		fprintf(stderr, "problem creating E1 line\n");
		exit(EXIT_FAILURE);
	}

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

	if (e1inp_line_update(line, E1INP_LINE_R_BSC) < 0) {
		fprintf(stderr, "problem enabling E1 line\n");
		exit(EXIT_FAILURE);
	}

	while (1) {
		osmo_select_main(0);
	}
	return 0;
}

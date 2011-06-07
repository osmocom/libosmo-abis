#include <stdio.h>
#include <talloc.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>

static void *tall_test;

static int rx_cb(struct msgb *msg, struct e1inp_ts *ts)
{
	printf("received data\n");

	/* For ISDN: from the timeslot type, we can get if it's:
	 * - E1INP_TS_TYPE_SIGN: in this case, check if we have a signal
	 *   link in this timeslot, then, check if it's OML or RSL and pass
	 *   it to the upper layer.
	 * - E1INP_TS_TYPE_TRAU.
	 *
	 * For INET: we have to check if this is a control message, if it's
	 * a response to previous request id, we enable the signal links for
	 * OML and RSL. Otherwise, look up for the existing signal link and
	 * pass it to the upper layer. We don't receive TRAU frames, we use
	 * RTP.
	 */
	return 0;
}

static int rx_err_cb(int error)
{
	printf("error, malformed message\n");
	return 0;
}

int main(void)
{
	struct e1inp_line *line;
	struct e1inp_ts *sign_ts;

	tall_test = talloc_named_const(NULL, 1, "e1inp_test");
	libosmo_abis_init(tall_test);

#define LINENR 0

	line = e1inp_line_create(LINENR, "ipa", rx_cb, rx_err_cb);
	if (line == NULL) {
		fprintf(stderr, "problem creating E1 line\n");
		exit(EXIT_FAILURE);
	}
	sign_ts = &line->ts[0];
	if (e1inp_ts_config_sign(sign_ts, line) < 0) {
		fprintf(stderr, "problem setting timeslot in E1 line\n");
		exit(EXIT_FAILURE);
	}

	/*
	 * Depending if this is a real or virtual E1 lines:
	 * - real (ISDN): create signal link for OML and RSL before line up.
	 * - vitual (INET): we create it once we have seen response to REQ_ID.
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

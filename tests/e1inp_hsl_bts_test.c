#include <stdio.h>
#include <talloc.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <osmocom/abis/abis.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/gsm/protocol/gsm_12_21.h>

static void *tall_test;
static struct e1inp_sign_link *oml_sign_link, *rsl_sign_link;

#define DBTSTEST 0

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

enum bts_state_machine {
	BTS_TEST_OML_SIGN_LINK_DOWN	= 0,
	BTS_TEST_OML_SIGN_LINK_UP,
	BTS_TEST_OML_WAIT_SW_ACT_ACK,
};

static struct osmo_fd bts_eventfd;
static int eventfds[2];

static enum bts_state_machine bts_state = BTS_TEST_OML_SIGN_LINK_DOWN;

static struct e1inp_sign_link *
sign_link_up(void *unit, struct e1inp_line *line, enum e1inp_sign_type type)
{
	struct e1inp_sign_link *sign_link = NULL;

	LOGP(DBTSTEST, LOGL_NOTICE, "OML and RSL link up request received.\n");

	e1inp_ts_config_sign(&line->ts[0], line);
	sign_link = oml_sign_link =
		e1inp_sign_link_create(&line->ts[0],
					E1INP_SIGN_OML, NULL, 255, 0);
	if (!oml_sign_link) {
		LOGP(DBTSTEST, LOGL_ERROR, "cannot create OML sign link\n");
		return NULL;
	}
	sign_link = rsl_sign_link =
		e1inp_sign_link_create(&line->ts[0],
					E1INP_SIGN_RSL, NULL, 0, 0);
	if (!rsl_sign_link) {
		LOGP(DBTSTEST, LOGL_ERROR, "cannot create RSL sign link\n");
		return NULL;
	}

	if (sign_link) {
		LOGP(DBTSTEST, LOGL_NOTICE, "signal link has been set up.\n");
		/* Now we can send OML messages to the BSC. */
		bts_state = BTS_TEST_OML_SIGN_LINK_UP;
	}
	/* tell GSM 12.21 that we're ready to proceed via our event fd. */
	unsigned int event_type = 0;
	if (write(eventfds[1], &event_type, sizeof(unsigned int)) < 0)
		LOGP(DBTSTEST, LOGL_ERROR, "cannot write to event fd.\n");

	return sign_link;
}

static void sign_link_down(struct e1inp_line *line)
{
	LOGP(DBTSTEST, LOGL_NOTICE, "signal link has been closed\n");
	if (oml_sign_link)
		e1inp_sign_link_destroy(oml_sign_link);
	if (rsl_sign_link)
		e1inp_sign_link_destroy(rsl_sign_link);
}

static int abis_nm_rcvmsg_fom(struct msgb *msg)
{
	struct abis_om_fom_hdr *foh = msgb_l3(msg);
	uint8_t mt = foh->msg_type;
	int ret = 0;

	switch (mt) {
	case NM_MT_SW_ACT_REQ_ACK:  /* SW activate request ACK from BSC. */
		LOGP(DBTSTEST, LOGL_NOTICE, "Receive SW Act Req ACK\n");
		break;
	default:
		LOGP(DBTSTEST, LOGL_ERROR, "unknown OML message\n");
		break;
	}
	return ret;
}

static int abis_nm_rcvmsg(struct msgb *msg)
{
	int ret = 0;
	struct abis_om_hdr *oh = msgb_l2(msg);

	msg->l3h = (unsigned char *)oh + sizeof(*oh);
	switch (oh->mdisc) {
	case ABIS_OM_MDISC_FOM:
		ret = abis_nm_rcvmsg_fom(msg);
		break;
	default:
		LOGP(DBTSTEST, LOGL_ERROR, "unknown OML message\n");
	break;
	}
	return ret;
}

static int sign_link(struct msgb *msg)
{
	int ret = 0;
	struct e1inp_sign_link *link = msg->dst;

	switch(link->type) {
	case E1INP_SIGN_OML:
		LOGP(DBTSTEST, LOGL_NOTICE, "OML message received.\n");
		ret = abis_nm_rcvmsg(msg);
		break;
	case E1INP_SIGN_RSL:
		LOGP(DBTSTEST, LOGL_NOTICE, "RSL message received.\n");
		break;
	default:
		LOGP(DBTSTEST, LOGL_ERROR, "Unknown signalling message.\n");
		break;
	}
	return ret;
}

static void fill_om_hdr(struct abis_om_hdr *oh, uint8_t len)
{
	oh->mdisc = ABIS_OM_MDISC_FOM;
	oh->placement = ABIS_OM_PLACEMENT_ONLY;
	oh->sequence = 0;
	oh->length = len;
}

static void fill_om_fom_hdr(struct abis_om_hdr *oh, uint8_t len,
			    uint8_t msg_type, uint8_t obj_class,
			    uint8_t bts_nr, uint8_t trx_nr, uint8_t ts_nr)
{
	struct abis_om_fom_hdr *foh = (struct abis_om_fom_hdr *) oh->data;

	fill_om_hdr(oh, len+sizeof(*foh));
	foh->msg_type = msg_type;
	foh->obj_class = obj_class;
	foh->obj_inst.bts_nr = bts_nr;
	foh->obj_inst.trx_nr = trx_nr;
	foh->obj_inst.ts_nr = ts_nr;
}

#define OM_ALLOC_SIZE		1024
#define OM_HEADROOM_SIZE	128

static struct msgb *nm_msgb_alloc(void)
{
        return msgb_alloc_headroom(OM_ALLOC_SIZE, OM_HEADROOM_SIZE, "BTS/test");
}

static int abis_nm_sw_act_req(struct e1inp_sign_link *sign_link,
			      uint8_t obj_class,
			      uint8_t i1, uint8_t i2, uint8_t i3,
			      uint8_t *attr, int att_len)
{
	struct abis_om_hdr *oh;
	struct msgb *msg = nm_msgb_alloc();
	uint8_t msgtype = NM_MT_SW_ACT_REQ;

	oh = (struct abis_om_hdr *) msgb_put(msg, ABIS_OM_FOM_HDR_SIZE);
	fill_om_fom_hdr(oh, att_len, msgtype, obj_class, i1, i2, i3);

	if (attr) {
		uint8_t *ptr = msgb_put(msg, att_len);
		memcpy(ptr, attr, att_len);
	}
	msg->dst = sign_link;
	return abis_sendmsg(msg);
}

static int test_bts_gsm_12_21_cb(struct osmo_fd *ofd, unsigned int what)
{
	int ret;
	unsigned int event_type;

	LOGP(DBTSTEST, LOGL_NOTICE, "event fd callback running\n");

	if (read(ofd->fd, &event_type, sizeof(event_type)) == -1) {
		LOGP(DBTSTEST, LOGL_ERROR, "problems with event fd\n");
		return -EINVAL;
	}
	switch(bts_state) {
	case BTS_TEST_OML_SIGN_LINK_DOWN:
		/* Do nothing until OML link becomes ready. */
		break;
	case BTS_TEST_OML_SIGN_LINK_UP:
		/* OML link is up, send SW ACT REQ. */
		ret = abis_nm_sw_act_req(oml_sign_link, 0, 0, 0, 0, NULL, 0);
		bts_state = BTS_TEST_OML_WAIT_SW_ACT_ACK;
		break;
	case BTS_TEST_OML_WAIT_SW_ACT_ACK:
		/* ... things should continue after this. */
		break;
	}
	return 0;
}

int main(void)
{
	struct hsl_unit bts_dev_info = {
		.swversion	= 0xc0,
		.serno		= 0x0123456789ABCDEF,
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

	line = e1inp_line_create(LINENR, "hsl");
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

	if (pipe(eventfds) < 0) {
		LOGP(DBTSTEST, LOGL_ERROR, "cannot create pipe fds\n");
		exit(EXIT_FAILURE);
	}
	bts_eventfd.fd = eventfds[0];
	bts_eventfd.cb = test_bts_gsm_12_21_cb;
	bts_eventfd.when = BSC_FD_READ;
	if (osmo_fd_register(&bts_eventfd) < 0) {
		LOGP(DBTSTEST, LOGL_ERROR, "could not register event fd\n");
		exit(EXIT_FAILURE);
	}

	while (1) {
		osmo_select_main(0);
	}
	return 0;
}

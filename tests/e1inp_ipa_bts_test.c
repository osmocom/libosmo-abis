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

struct dummy_bts {
	uint16_t	bts_id;
	uint16_t	site_id;
	uint16_t	trx_id;
	uint8_t		mac_addr[6];
} dummy_bts = {
	.site_id	= 0,
	.bts_id		= 0,
	.trx_id		= 0,
	.mac_addr	= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};

char *software_version = "0.1";

static struct msgb *abis_msgb_alloc(int headroom)
{
	struct msgb *nmsg;

	headroom += sizeof(struct ipaccess_head);

	nmsg = msgb_alloc_headroom(1200 + headroom, headroom, "dummy BTS");
	if (!nmsg)
		return NULL;
	return nmsg;
}

static void abis_push_ipa(struct msgb *msg, uint8_t proto)
{
	struct ipaccess_head *nhh;

	msg->l2h = msg->data;
	nhh = (struct ipaccess_head *) msgb_push(msg, sizeof(*nhh));
	nhh->proto = proto;
	nhh->len = htons(msgb_l2len(msg));
}

/* XXX: we have to do this in input/ipaccess.c, moreover we have to put this
 * information in some data structure that we'll pass to sign_link_up. */
static struct msgb *ipa_bts_id_resp(uint8_t *data, int len)
{
	struct msgb *nmsg;
	char str[64];
	uint8_t *tag;

	nmsg = abis_msgb_alloc(0);
	if (!nmsg)
		return NULL;

	*msgb_put(nmsg, 1) = IPAC_MSGT_ID_RESP;
	while (len) {
		if (len < 2) {
			LOGP(DBTSTEST, LOGL_NOTICE,
				"Short read of ipaccess tag\n");
			msgb_free(nmsg);
			return NULL;
		}
		switch (data[1]) {
		case IPAC_IDTAG_UNIT:
			sprintf(str, "%u/%u/%u",
				dummy_bts.site_id,
				dummy_bts.bts_id,
				dummy_bts.trx_id);
			break;
		case IPAC_IDTAG_MACADDR:
			sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
				dummy_bts.mac_addr[0], dummy_bts.mac_addr[1],
				dummy_bts.mac_addr[2], dummy_bts.mac_addr[3],
				dummy_bts.mac_addr[4], dummy_bts.mac_addr[5]);
			break;
		case IPAC_IDTAG_LOCATION1:
			strcpy(str, "osmoBTS");
			break;
		case IPAC_IDTAG_LOCATION2:
			strcpy(str, "osmoBTS");
			break;
		case IPAC_IDTAG_EQUIPVERS:
		case IPAC_IDTAG_SWVERSION:
			strcpy(str, software_version);
			break;
		case IPAC_IDTAG_UNITNAME:
			sprintf(str, "osmoBTS-%02x-%02x-%02x-%02x-%02x-%02x",
				dummy_bts.mac_addr[0], dummy_bts.mac_addr[1],
				dummy_bts.mac_addr[2], dummy_bts.mac_addr[3],
				dummy_bts.mac_addr[4], dummy_bts.mac_addr[5]);
			break;
		case IPAC_IDTAG_SERNR:
			strcpy(str, "");
			break;
		default:
			LOGP(DBTSTEST, LOGL_NOTICE,
				"Unknown ipaccess tag 0x%02x\n", *data);
			msgb_free(nmsg);
			return NULL;
		}
		LOGP(DBTSTEST, LOGL_INFO, " tag %d: %s\n", data[1], str);
		tag = msgb_put(nmsg, 3 + strlen(str) + 1);
		tag[0] = 0x00;
		tag[1] = 1 + strlen(str) + 1;
		tag[2] = data[1];
		memcpy(tag + 3, str, strlen(str) + 1);
		data += 2;
		len -= 2;
	}
	abis_push_ipa(nmsg, IPAC_PROTO_IPACCESS);
	return nmsg;
}

static struct msgb *ipa_bts_id_ack(void)
{
	struct msgb *nmsg2;

	nmsg2 = abis_msgb_alloc(0);
	if (!nmsg2)
		return NULL;

	*msgb_put(nmsg2, 1) = IPAC_MSGT_ID_ACK;
	abis_push_ipa(nmsg2, IPAC_PROTO_IPACCESS);

	return nmsg2;
}

static struct e1inp_sign_link *
sign_link_up(void *unit, struct e1inp_line *line, enum e1inp_sign_type type)
{
	struct msgb *msg = unit;
	struct msgb *rmsg;
	uint8_t *data = msgb_l2(msg);
	int len = msgb_l2len(msg);
	struct e1inp_sign_link *sign_link = NULL;
	void *dst = NULL;

	switch(type) {
	case E1INP_SIGN_OML:
		printf("ID_RESP for OML received.\n");

		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML - 1], line);
		sign_link = oml_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_OML - 1],
						E1INP_SIGN_OML, NULL, 255, 0);
		if (!oml_sign_link)
			printf("cannot create OML sign link\n");

		dst = oml_sign_link;
		break;
	case E1INP_SIGN_RSL:
		printf("ID_RESP for RSL received.\n");

		e1inp_ts_config_sign(&line->ts[E1INP_SIGN_RSL - 1], line);

		sign_link = rsl_sign_link =
			e1inp_sign_link_create(&line->ts[E1INP_SIGN_RSL - 1],
						E1INP_SIGN_RSL, NULL, 0, 0);
		if (!rsl_sign_link)
			printf("cannot create RSL sign link\n");

		dst = rsl_sign_link;
		break;
	default:
		return NULL;
	}

	/* send ID_RESP. */
	rmsg = ipa_bts_id_resp(data + 1, len - 1);
	rmsg->dst = dst;
	abis_sendmsg(rmsg);

	/* send ID_ACK. */
	rmsg = ipa_bts_id_ack();
	rmsg->dst = dst;
	abis_sendmsg(rmsg);

	return sign_link;
}

static void sign_link_down(struct e1inp_line *line)
{
	printf("closing sign link.\n");
}

static int sign_link(struct msgb *msg, struct e1inp_sign_link *link)
{
	printf("OML/RSL data received\n");
	return 0;
}

int main(void)
{
	struct e1inp_line *line;

	tall_test = talloc_named_const(NULL, 1, "e1inp_test");
	libosmo_abis_init(tall_test);

	osmo_init_logging(&bts_test_log_info);

	struct e1inp_line_ops ops = {
		.role		= E1INP_LINE_R_BTS,
		.addr		= "127.0.0.1",
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

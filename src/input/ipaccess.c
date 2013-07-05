/* OpenBSC Abis input driver for ip.access */

/* (C) 2009 by Harald Welte <laforge@gnumonks.org>
 * (C) 2010 by Holger Hans Peter Freyther
 * (C) 2010 by On-Waves
 *
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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <osmocom/core/select.h>
#include <osmocom/gsm/tlv.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipaccess.h>
#include <osmocom/core/socket.h>
#include <osmocom/abis/ipa.h>

static void *tall_ipa_ctx;

#define TS1_ALLOC_SIZE	900

/*
 * Common propietary IPA messages:
 *      - PONG: in reply to PING.
 *      - ID_REQUEST: first messages once OML has been established.
 *      - ID_ACK: in reply to ID_ACK.
 */
const uint8_t ipa_pong_msg[] = {
	0, 1, IPAC_PROTO_IPACCESS, IPAC_MSGT_PONG
};

const uint8_t ipa_id_ack_msg[] = {
	0, 1, IPAC_PROTO_IPACCESS, IPAC_MSGT_ID_ACK
};

const uint8_t ipa_id_req_msg[] = {
	0, 17, IPAC_PROTO_IPACCESS, IPAC_MSGT_ID_GET,
	0x01, IPAC_IDTAG_UNIT,
	0x01, IPAC_IDTAG_MACADDR,
	0x01, IPAC_IDTAG_LOCATION1,
	0x01, IPAC_IDTAG_LOCATION2,
	0x01, IPAC_IDTAG_EQUIPVERS,
	0x01, IPAC_IDTAG_SWVERSION,
	0x01, IPAC_IDTAG_UNITNAME,
	0x01, IPAC_IDTAG_SERNR,
};

static const char *idtag_names[] = {
	[IPAC_IDTAG_SERNR]	= "Serial_Number",
	[IPAC_IDTAG_UNITNAME]	= "Unit_Name",
	[IPAC_IDTAG_LOCATION1]	= "Location_1",
	[IPAC_IDTAG_LOCATION2]	= "Location_2",
	[IPAC_IDTAG_EQUIPVERS]	= "Equipment_Version",
	[IPAC_IDTAG_SWVERSION]	= "Software_Version",
	[IPAC_IDTAG_IPADDR]	= "IP_Address",
	[IPAC_IDTAG_MACADDR]	= "MAC_Address",
	[IPAC_IDTAG_UNIT]	= "Unit_ID",
};

const char *ipaccess_idtag_name(uint8_t tag)
{
	if (tag >= ARRAY_SIZE(idtag_names))
		return "unknown";

	return idtag_names[tag];
}

int ipaccess_idtag_parse(struct tlv_parsed *dec, unsigned char *buf, int len)
{
	uint8_t t_len;
	uint8_t t_tag;
	uint8_t *cur = buf;

	memset(dec, 0, sizeof(*dec));

	while (len >= 2) {
		len -= 2;
		t_len = *cur++;
		t_tag = *cur++;

		if (t_len > len + 1) {
			LOGP(DLMI, LOGL_ERROR, "The tag does not fit: %d\n", t_len);
			return -EINVAL;
		}

		DEBUGPC(DLMI, "%s='%s' ", ipaccess_idtag_name(t_tag), cur);

		dec->lv[t_tag].len = t_len;
		dec->lv[t_tag].val = cur;

		cur += t_len;
		len -= t_len;
	}
	return 0;
}

int ipaccess_parse_unitid(const char *str, struct ipaccess_unit *unit_data)
{
	unsigned long ul;
	char *endptr;
	const char *nptr;

	nptr = str;
	ul = strtoul(nptr, &endptr, 10);
	if (endptr <= nptr)
		return -EINVAL;
	unit_data->site_id = ul & 0xffff;

	if (*endptr++ != '/')
		return -EINVAL;

	nptr = endptr;
	ul = strtoul(nptr, &endptr, 10);
	if (endptr <= nptr)
		return -EINVAL;
	unit_data->bts_id = ul & 0xffff;

	if (*endptr++ != '/')
		return -EINVAL;

	nptr = endptr;
	ul = strtoul(nptr, &endptr, 10);
	if (endptr <= nptr)
		return -EINVAL;
	unit_data->trx_id = ul & 0xffff;

	return 0;
}

static int ipaccess_send(int fd, const void *msg, size_t msglen)
{
	int ret;

	ret = write(fd, msg, msglen);
	if (ret < 0)
		return ret;
	if (ret < msglen) {
		LOGP(DLINP, LOGL_ERROR, "ipaccess_send: short write\n");
		return -EIO;
	}
	return ret;
}

int ipaccess_send_pong(int fd)
{
	return ipaccess_send(fd, ipa_pong_msg, sizeof(ipa_pong_msg));
}

int ipaccess_send_id_ack(int fd)
{
	return ipaccess_send(fd, ipa_id_ack_msg, sizeof(ipa_id_ack_msg));
}

int ipaccess_send_id_req(int fd)
{
	return ipaccess_send(fd, ipa_id_req_msg, sizeof(ipa_id_req_msg));
}

/* base handling of the ip.access protocol */
int ipaccess_rcvmsg_base(struct msgb *msg, struct osmo_fd *bfd)
{
	uint8_t msg_type = *(msg->l2h);
	int ret;

	switch (msg_type) {
	case IPAC_MSGT_PING:
		ret = ipaccess_send_pong(bfd->fd);
		if (ret < 0) {
			LOGP(DLINP, LOGL_ERROR, "Cannot send PING "
			     "message. Reason: %s\n", strerror(errno));
			break;
		}
		ret = 1;
		break;
	case IPAC_MSGT_PONG:
		DEBUGP(DLMI, "PONG!\n");
		ret = 1;
		break;
	case IPAC_MSGT_ID_ACK:
		DEBUGP(DLMI, "ID_ACK? -> ACK!\n");
		ret = ipaccess_send_id_ack(bfd->fd);
		if (ret < 0) {
			LOGP(DLINP, LOGL_ERROR, "Cannot send ID_ACK "
			     "message. Reason: %s\n", strerror(errno));
			break;
		}
		ret = 1;
		break;
	default:
		/* This is not an IPA PING, PONG or ID_ACK message */
		ret = 0;
		break;
	}
	return ret;
}

/* base handling of the ip.access protocol */
int ipaccess_rcvmsg_bts_base(struct msgb *msg,
			     struct osmo_fd *bfd)
{
	uint8_t msg_type = *(msg->l2h);
	int ret = 0;

	switch (msg_type) {
	case IPAC_MSGT_PING:
		ret = ipaccess_send_pong(bfd->fd);
		if (ret < 0) {
			LOGP(DLINP, LOGL_ERROR, "Cannot send PONG "
			     "message. Reason: %s\n", strerror(errno));
		}
		break;
	case IPAC_MSGT_PONG:
		DEBUGP(DLMI, "PONG!\n");
		break;
	case IPAC_MSGT_ID_ACK:
		DEBUGP(DLMI, "ID_ACK\n");
		break;
	}
	return ret;
}

static int ipaccess_drop(struct osmo_fd *bfd)
{
	int ret = 1;
	struct e1inp_line *line = bfd->data;

	/* Error case: we did not see any ID_RESP yet for this socket. */
	if (bfd->fd != -1) {
		LOGP(DLINP, LOGL_ERROR, "Forcing socket shutdown with "
					"no signal link set\n");
		osmo_fd_unregister(bfd);
		close(bfd->fd);
		bfd->fd = -1;
		ret = -ENOENT;
	}

	/* e1inp_sign_link_destroy releases the socket descriptors for us. */
	line->ops->sign_link_down(line);

	return ret;
}

static int ipaccess_rcvmsg(struct e1inp_line *line, struct msgb *msg,
			   struct osmo_fd *bfd)
{
	struct tlv_parsed tlvp;
	uint8_t msg_type = *(msg->l2h);
	struct ipaccess_unit unit_data = {};
	struct e1inp_sign_link *sign_link;
	char *unitid;
	int len, ret;

	/* Handle IPA PING, PONG and ID_ACK messages. */
	ret = ipaccess_rcvmsg_base(msg, bfd);
	switch(ret) {
	case -1:
		/* error in IPA control message handling */
		goto err;
	case 1:
		/* this is an IPA control message, skip further processing */
		return 0;
	case 0:
		/* this is not an IPA control message, continue */
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "Unexpected return from "
					"ipaccess_rcvmsg_base "
					"(ret=%d)\n", ret);
		goto err;
	}

	switch (msg_type) {
	case IPAC_MSGT_ID_RESP:
		DEBUGP(DLMI, "ID_RESP\n");
		/* parse tags, search for Unit ID */
		ret = ipaccess_idtag_parse(&tlvp, (uint8_t *)msg->l2h + 2,
						msgb_l2len(msg)-2);
		DEBUGP(DLMI, "\n");
		if (ret < 0) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"with malformed TLVs\n");
			ret = -EINVAL;
			goto err;
		}
		if (!TLVP_PRESENT(&tlvp, IPAC_IDTAG_UNIT)) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"without unit ID\n");
			ret = -EINVAL;
			goto err;

		}
		len = TLVP_LEN(&tlvp, IPAC_IDTAG_UNIT);
		if (len < 1) {
			LOGP(DLINP, LOGL_ERROR, "IPA response message "
				"with too small unit ID\n");
			ret = -EINVAL;
			goto err;
		}
		unitid = (char *) TLVP_VAL(&tlvp, IPAC_IDTAG_UNIT);
		unitid[len - 1] = '\0';
		ipaccess_parse_unitid(unitid, &unit_data);

		if (!line->ops->sign_link_up) {
			LOGP(DLINP, LOGL_ERROR,
			     "Unable to set signal link, closing socket.\n");
			ret = -EINVAL;
			goto err;
		}
		/* the BSC creates the new sign links at this stage. */
		if (bfd->priv_nr == E1INP_SIGN_OML) {
			sign_link =
				line->ops->sign_link_up(&unit_data, line,
							E1INP_SIGN_OML);
			if (sign_link == NULL) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				ret = -EINVAL;
				goto err;
			}
		} else if (bfd->priv_nr == E1INP_SIGN_RSL) {
			struct e1inp_ts *ts;
                        struct osmo_fd *newbfd;
			struct e1inp_line *new_line;

			sign_link =
				line->ops->sign_link_up(&unit_data, line,
							E1INP_SIGN_RSL);
			if (sign_link == NULL) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				ret = -EINVAL;
				goto err;
			}
			/* this is a bugtrap, the BSC should be using the
			 * virtual E1 line used by OML for this RSL link. */
			if (sign_link->ts->line == line) {
				LOGP(DLINP, LOGL_ERROR,
					"Fix your BSC, you should use the "
					"E1 line used by the OML link for "
					"your RSL link.\n");
				return 0;
			}
			/* Finally, we know which OML link is associated with
			 * this RSL link, attach it to this socket. */
			bfd->data = new_line = sign_link->ts->line;
			e1inp_line_get(new_line);
			ts = &new_line->ts[E1INP_SIGN_RSL+unit_data.trx_id-1];
			newbfd = &ts->driver.ipaccess.fd;

			/* get rid of our old temporary bfd */
			memcpy(newbfd, bfd, sizeof(*newbfd));
			newbfd->priv_nr = E1INP_SIGN_RSL + unit_data.trx_id;
			osmo_fd_unregister(bfd);
			bfd->fd = -1;
			osmo_fd_register(newbfd);
			/* now we can release the dummy RSL line. */
			e1inp_line_put(line);
		}
		break;
	default:
		LOGP(DLINP, LOGL_ERROR, "Unknown IPA message type\n");
		ret = -EINVAL;
		goto err;
	}
	return 0;
err:
	osmo_fd_unregister(bfd);
	close(bfd->fd);
	bfd->fd = -1;
	e1inp_line_put(line);
	return ret;
}

static int handle_ts1_read(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *link;
	struct ipaccess_head *hh;
	struct msgb *msg;
	int ret;

	ret = ipa_msg_recv(bfd->fd, &msg);
	if (ret < 0) {
		LOGP(DLINP, LOGL_NOTICE, "Sign link problems, "
			"closing socket. Reason: %s\n", strerror(errno));
		goto err;
	} else if (ret == 0) {
		LOGP(DLINP, LOGL_NOTICE, "Sign link vanished, dead socket\n");
		goto err;
	}
	DEBUGP(DLMI, "RX %u: %s\n", ts_nr, osmo_hexdump(msgb_l2(msg), msgb_l2len(msg)));

	hh = (struct ipaccess_head *) msg->data;
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		ipaccess_rcvmsg(line, msg, bfd);
		msgb_free(msg);
		return 0;
	}
	/* BIG FAT WARNING: bfd might no longer exist here, since ipaccess_rcvmsg()
	 * might have free'd it !!! */

	link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (!link) {
		LOGP(DLINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		ret = -EINVAL;
		goto err_msg;
	}
	msg->dst = link;

	/* XXX better use e1inp_ts_rx? */
	if (!e1i_ts->line->ops->sign_link) {
		LOGP(DLINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		ret = -EINVAL;
		goto err_msg;
	}
	if (e1i_ts->line->ops->sign_link(msg) < 0) {
		/* Don't close the signalling link if the upper layers report
		 * an error, that's too strict. BTW, the signalling layer is
		 * resposible for releasing the message.
		 */
		LOGP(DLINP, LOGL_ERROR, "Bad signalling message,"
			"sign_link returned error\n");
	}

	return 0;
err_msg:
	msgb_free(msg);
err:
	ipaccess_drop(bfd);
	return ret;
}

void ipaccess_prepend_header_ext(struct msgb *msg, int proto)
{
	struct ipaccess_head_ext *hh_ext;

	/* prepend the osmo ip.access header extension */
	hh_ext = (struct ipaccess_head_ext *) msgb_push(msg, sizeof(*hh_ext));
	hh_ext->proto = proto;
}

void ipaccess_prepend_header(struct msgb *msg, int proto)
{
	struct ipaccess_head *hh;

	/* prepend the ip.access header */
	hh = (struct ipaccess_head *) msgb_push(msg, sizeof(*hh));
	hh->len = htons(msg->len - sizeof(*hh));
	hh->proto = proto;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	e1i_ts->driver.ipaccess.fd.when |= BSC_FD_WRITE;

	return 0;
}

static void ipaccess_close(struct e1inp_sign_link *sign_link)
{
	struct e1inp_ts *e1i_ts = sign_link->ts;
	struct osmo_fd *bfd = &e1i_ts->driver.ipaccess.fd;
	return e1inp_close_socket(e1i_ts, sign_link, bfd);
}

static void timeout_ts1_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	ts_want_write(e1i_ts);
}

static int __handle_ts1_write(struct osmo_fd *bfd, struct e1inp_line *line)
{
	unsigned int ts_nr = bfd->priv_nr;
	struct e1inp_ts *e1i_ts = &line->ts[ts_nr-1];
	struct e1inp_sign_link *sign_link;
	struct msgb *msg;
	int ret;

	bfd->when &= ~BSC_FD_WRITE;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		return 0;
	}

	switch (sign_link->type) {
	case E1INP_SIGN_OML:
		break;
	case E1INP_SIGN_RSL:
		break;
	default:
		bfd->when |= BSC_FD_WRITE; /* come back for more msg */
		ret = -EINVAL;
		goto out;
	}

	msg->l2h = msg->data;
	ipaccess_prepend_header(msg, sign_link->tei);

	DEBUGP(DLMI, "TX %u: %s\n", ts_nr, osmo_hexdump(msg->l2h, msgb_l2len(msg)));

	ret = send(bfd->fd, msg->data, msg->len, 0);
	if (ret != msg->len) {
		LOGP(DLINP, LOGL_ERROR, "failed to send A-bis IPA signalling "
			"message. Reason: %s\n", strerror(errno));
		goto err;
	}

	/* set tx delay timer for next event */
	e1i_ts->sign.tx_timer.cb = timeout_ts1_write;
	e1i_ts->sign.tx_timer.data = e1i_ts;

	/* Reducing this might break the nanoBTS 900 init. */
	osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, e1i_ts->sign.delay);

out:
	msgb_free(msg);
	return ret;
err:
	ipaccess_drop(bfd);
	msgb_free(msg);
	return ret;
}

static int handle_ts1_write(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;

	return __handle_ts1_write(bfd, line);
}

static int ipaccess_bts_write_cb(struct ipa_client_conn *link)
{
	struct e1inp_line *line = link->line;

	return __handle_ts1_write(link->ofd, line);
}

/* callback from select.c in case one of the fd's can be read/written */
int ipaccess_fd_cb(struct osmo_fd *bfd, unsigned int what)
{
	int rc = 0;

	if (what & BSC_FD_READ)
		rc = handle_ts1_read(bfd);
	if (what & BSC_FD_WRITE)
		rc = handle_ts1_write(bfd);

	return rc;
}

static int ipaccess_line_update(struct e1inp_line *line);

struct e1inp_driver ipaccess_driver = {
	.name = "ipa",
	.want_write = ts_want_write,
	.line_update = ipaccess_line_update,
	.close = ipaccess_close,
	.default_delay = 0,
};

/* callback of the OML listening filedescriptor */
static int ipaccess_bsc_oml_cb(struct ipa_server_link *link, int fd)
{
	int ret;
	int idx = 0;
	int i;
	struct e1inp_line *line;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;

	/* clone virtual E1 line for this new OML link. */
	line = e1inp_line_clone(tall_ipa_ctx, link->line);
	if (line == NULL) {
		LOGP(DLINP, LOGL_ERROR, "could not clone E1 line\n");
		return -ENOMEM;
	}

	/* create virrtual E1 timeslots for signalling */
	e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML-1], line);

	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	e1i_ts = &line->ts[idx];

	bfd = &e1i_ts->driver.ipaccess.fd;
	bfd->fd = fd;
	bfd->data = line;
	bfd->priv_nr = E1INP_SIGN_OML;
	bfd->cb = ipaccess_fd_cb;
	bfd->when = BSC_FD_READ;
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}

	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipaccess_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n",
			strerror(errno));
		goto err_socket;
	}
	return ret;

err_socket:
	osmo_fd_unregister(bfd);
err_line:
	close(bfd->fd);
	bfd->fd = -1;
	e1inp_line_put(line);
	return ret;
}

static int ipaccess_bsc_rsl_cb(struct ipa_server_link *link, int fd)
{
	struct e1inp_line *line;
	struct e1inp_ts *e1i_ts;
	struct osmo_fd *bfd;
	int i, ret;

        /* We don't know yet which OML link to associate it with. Thus, we
         * allocate a temporary E1 line until we have received ID. */
	line = e1inp_line_clone(tall_ipa_ctx, link->line);
	if (line == NULL) {
		LOGP(DLINP, LOGL_ERROR, "could not clone E1 line\n");
		return -ENOMEM;
	}
	/* initialize the fds */
	for (i = 0; i < ARRAY_SIZE(line->ts); ++i)
		line->ts[i].driver.ipaccess.fd.fd = -1;

	/* we need this to initialize this in case to avoid crashes in case
	 * that the socket is closed before we've seen an ID_RESP. */
	e1inp_ts_config_sign(&line->ts[E1INP_SIGN_OML-1], line);

	e1i_ts = &line->ts[E1INP_SIGN_RSL-1];

	bfd = &e1i_ts->driver.ipaccess.fd;
	bfd->fd = fd;
	bfd->data = line;
	bfd->priv_nr = E1INP_SIGN_RSL;
	bfd->cb = ipaccess_fd_cb;
	bfd->when = BSC_FD_READ;
	ret = osmo_fd_register(bfd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not register FD\n");
		goto err_line;
	}
	/* Request ID. FIXME: request LOCATION, HW/SW VErsion, Unit Name, Serno */
	ret = ipaccess_send_id_req(bfd->fd);
	if (ret < 0) {
		LOGP(DLINP, LOGL_ERROR, "could not send ID REQ. Reason: %s\n",
			strerror(errno));
		goto err_socket;
	}
	return ret;

err_socket:
	osmo_fd_unregister(bfd);
err_line:
	close(bfd->fd);
	bfd->fd = -1;
	e1inp_line_put(line);
	return ret;
}

static struct msgb *
ipa_bts_id_resp(struct ipaccess_unit *dev, uint8_t *data, int len)
{
	struct msgb *nmsg;
	char str[64];
	uint8_t *tag;

	nmsg = ipa_msg_alloc(0);
	if (!nmsg)
		return NULL;

	*msgb_put(nmsg, 1) = IPAC_MSGT_ID_RESP;
	while (len) {
		if (len < 2) {
			LOGP(DLINP, LOGL_NOTICE,
				"Short read of ipaccess tag\n");
			msgb_free(nmsg);
			return NULL;
		}
		switch (data[1]) {
		case IPAC_IDTAG_UNIT:
			sprintf(str, "%u/%u/%u",
				dev->site_id, dev->bts_id, dev->trx_id);
			break;
		case IPAC_IDTAG_MACADDR:
			sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
				dev->mac_addr[0], dev->mac_addr[1],
				dev->mac_addr[2], dev->mac_addr[3],
				dev->mac_addr[4], dev->mac_addr[5]);
			break;
		case IPAC_IDTAG_LOCATION1:
			strcpy(str, dev->location1);
			break;
		case IPAC_IDTAG_LOCATION2:
			strcpy(str, dev->location2);
			break;
		case IPAC_IDTAG_EQUIPVERS:
			strcpy(str, dev->equipvers);
			break;
		case IPAC_IDTAG_SWVERSION:
			strcpy(str, dev->swversion);
			break;
		case IPAC_IDTAG_UNITNAME:
			sprintf(str, "%s-%02x-%02x-%02x-%02x-%02x-%02x",
				dev->unit_name,
				dev->mac_addr[0], dev->mac_addr[1],
				dev->mac_addr[2], dev->mac_addr[3],
				dev->mac_addr[4], dev->mac_addr[5]);
			break;
		case IPAC_IDTAG_SERNR:
			strcpy(str, dev->serno);
			break;
		default:
			LOGP(DLINP, LOGL_NOTICE,
				"Unknown ipaccess tag 0x%02x\n", *data);
			msgb_free(nmsg);
			return NULL;
		}
		LOGP(DLINP, LOGL_INFO, " tag %d: %s\n", data[1], str);
		tag = msgb_put(nmsg, 3 + strlen(str) + 1);
		tag[0] = 0x00;
		tag[1] = 1 + strlen(str) + 1;
		tag[2] = data[1];
		memcpy(tag + 3, str, strlen(str) + 1);
		data += 2;
		len -= 2;
	}
	ipa_msg_push_header(nmsg, IPAC_PROTO_IPACCESS);
	return nmsg;
}

static struct msgb *ipa_bts_id_ack(void)
{
	struct msgb *nmsg2;

	nmsg2 = ipa_msg_alloc(0);
	if (!nmsg2)
		return NULL;

	*msgb_put(nmsg2, 1) = IPAC_MSGT_ID_ACK;
	ipa_msg_push_header(nmsg2, IPAC_PROTO_IPACCESS);

	return nmsg2;
}

static int ipaccess_bts_cb(struct ipa_client_conn *link, struct msgb *msg)
{
	struct ipaccess_head *hh = (struct ipaccess_head *) msg->data;
	struct e1inp_ts *e1i_ts = NULL;
	struct e1inp_sign_link *sign_link;
	struct msgb *rmsg;
	int ret = 0;

	/* special handling for IPA CCM. */
	if (hh->proto == IPAC_PROTO_IPACCESS) {
		uint8_t msg_type = *(msg->l2h);

		/* ping, pong and acknowledgment cases. */
		ret = ipaccess_rcvmsg_bts_base(msg, link->ofd);
		if (ret < 0)
			goto err;

		/* this is a request for identification from the BSC. */
		if (msg_type == IPAC_MSGT_ID_GET) {
			struct e1inp_sign_link *sign_link;
			uint8_t *data = msgb_l2(msg);
			int len = msgb_l2len(msg);

			LOGP(DLINP, LOGL_NOTICE, "received ID get\n");
			if (!link->line->ops->sign_link_up) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				ret = -EINVAL;
				goto err;
			}
			sign_link = link->line->ops->sign_link_up(msg,
					link->line,
					link->ofd->priv_nr);
			if (sign_link == NULL) {
				LOGP(DLINP, LOGL_ERROR,
					"Unable to set signal link, "
					"closing socket.\n");
				ret = -EINVAL;
				goto err;
			}
			rmsg = ipa_bts_id_resp(link->line->ops->cfg.ipa.dev,
						data + 1, len - 1);
			ret = ipaccess_send(link->ofd->fd, rmsg->data,
						rmsg->len);
			if (ret != rmsg->len) {
				LOGP(DLINP, LOGL_ERROR, "cannot send ID_RESP "
				     "message. Reason: %s\n", strerror(errno));
				goto err_rmsg;
			}
			msgb_free(rmsg);

			/* send ID_ACK. */
			rmsg = ipa_bts_id_ack();
			ret = ipaccess_send(link->ofd->fd, rmsg->data,
						rmsg->len);
			if (ret != rmsg->len) {
				LOGP(DLINP, LOGL_ERROR, "cannot send ID_ACK "
				     "message. Reason: %s\n", strerror(errno));
				goto err_rmsg;
			}
			msgb_free(rmsg);
		}
		msgb_free(msg);
		return ret;
	} else if (link->port == IPA_TCP_PORT_OML)
		e1i_ts = &link->line->ts[0];
	else if (link->port == IPA_TCP_PORT_RSL)
		e1i_ts = &link->line->ts[1];

	OSMO_ASSERT(e1i_ts != NULL);

	/* look up for some existing signaling link. */
	sign_link = e1inp_lookup_sign_link(e1i_ts, hh->proto, 0);
	if (sign_link == NULL) {
		LOGP(DLINP, LOGL_ERROR, "no matching signalling link for "
			"hh->proto=0x%02x\n", hh->proto);
		ret = -EIO;
		goto err;
	}
	msg->dst = sign_link;

	/* XXX better use e1inp_ts_rx? */
	if (!link->line->ops->sign_link) {
		LOGP(DLINP, LOGL_ERROR, "Fix your application, "
			"no action set for signalling messages.\n");
		ret = -ENOENT;
		goto err;
	}
	link->line->ops->sign_link(msg);
	return 0;

err_rmsg:
	msgb_free(rmsg);
err:
	osmo_fd_unregister(link->ofd);
	close(link->ofd->fd);
	link->ofd->fd = -1;
	msgb_free(msg);
	return ret;
}

struct ipaccess_line {
	int line_already_initialized;
};

static int ipaccess_line_update(struct e1inp_line *line)
{
	int ret = -ENOENT;
	struct ipaccess_line *il;

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct ipaccess_line);

	if (!line->driver_data) {
		LOGP(DLINP, LOGL_ERROR, "ipaccess: OOM in line update\n");
		return -ENOMEM;
	}
	il = line->driver_data;

	/* We only initialize this line once. */
	if (il->line_already_initialized)
		return 0;

	il->line_already_initialized = 1;

	switch(line->ops->cfg.ipa.role) {
	case E1INP_LINE_R_BSC: {
		struct ipa_server_link *oml_link, *rsl_link;

		LOGP(DLINP, LOGL_NOTICE, "enabling ipaccess BSC mode\n");

		oml_link = ipa_server_link_create(tall_ipa_ctx, line,
					          "0.0.0.0", IPA_TCP_PORT_OML,
						  ipaccess_bsc_oml_cb, NULL);
		if (oml_link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create OML "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_server_link_open(oml_link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open OML BSC link: %s\n",
				strerror(errno));
			ipa_server_link_destroy(oml_link);
			return -EIO;
		}
		rsl_link = ipa_server_link_create(tall_ipa_ctx, line,
						  "0.0.0.0", IPA_TCP_PORT_RSL,
						  ipaccess_bsc_rsl_cb, NULL);
		if (rsl_link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create RSL "
				"BSC link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_server_link_open(rsl_link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open RSL BSC link: %s\n",
				strerror(errno));
			ipa_server_link_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	case E1INP_LINE_R_BTS: {
		struct ipa_client_conn *link, *rsl_link;

		LOGP(DLINP, LOGL_NOTICE, "enabling ipaccess BTS mode\n");

		link = ipa_client_conn_create(tall_ipa_ctx,
					      &line->ts[E1INP_SIGN_OML-1],
					      E1INP_SIGN_OML,
					      line->ops->cfg.ipa.addr,
					      IPA_TCP_PORT_OML,
					      NULL,
					      ipaccess_bts_cb,
					      ipaccess_bts_write_cb,
					      NULL);
		if (link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create OML "
				"BTS link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_client_conn_open(link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open OML BTS link: %s\n",
				strerror(errno));
			ipa_client_conn_close(link);
			ipa_client_conn_destroy(link);
			return -EIO;
		}
		rsl_link = ipa_client_conn_create(tall_ipa_ctx,
						  &line->ts[E1INP_SIGN_RSL-1],
						  E1INP_SIGN_RSL,
						  line->ops->cfg.ipa.addr,
						  IPA_TCP_PORT_RSL,
						  NULL,
						  ipaccess_bts_cb,
						  ipaccess_bts_write_cb,
						  NULL);
		if (rsl_link == NULL) {
			LOGP(DLINP, LOGL_ERROR, "cannot create RSL "
				"BTS link: %s\n", strerror(errno));
			return -ENOMEM;
		}
		if (ipa_client_conn_open(rsl_link) < 0) {
			LOGP(DLINP, LOGL_ERROR, "cannot open RSL BTS link: %s\n",
				strerror(errno));
			ipa_client_conn_close(rsl_link);
			ipa_client_conn_destroy(rsl_link);
			return -EIO;
		}
		ret = 0;
		break;
	}
	default:
		break;
	}
	return ret;
}

void e1inp_ipaccess_init(void)
{
	tall_ipa_ctx = talloc_named_const(libosmo_abis_ctx, 1, "ipa");
	e1inp_driver_register(&ipaccess_driver);
}

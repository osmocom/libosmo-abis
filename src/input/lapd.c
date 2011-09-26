/* OpenBSC minimal LAPD implementation */

/* (C) 2009 by oystein@homelien.no
 * (C) 2009 by Holger Hans Peter Freyther <zecke@selfish.org>
 * (C) 2010 by Digium and Matthew Fredrickson <creslin@digium.com>
 * (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * (C) 2011 by Andreas Eversberg <jolly@eversberg.eu>
 *
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include "internal.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/timer.h>
#include <osmocom/abis/lapd.h>

#define LAPD_ADDR2(sapi, cr) ((((sapi) & 0x3f) << 2) | (((cr) & 0x1) << 1))
#define LAPD_ADDR3(tei) ((((tei) & 0x7f) << 1) | 0x1)

#define LAPD_ADDR_SAPI(addr) ((addr) >> 2)
#define LAPD_ADDR_CR(addr) (((addr) >> 1) & 0x1)
#define LAPD_ADDR_EA(addr) ((addr) & 0x1)
#define LAPD_ADDR_TEI(addr) ((addr) >> 1)

#define LAPD_CTRL_I4(ns) (((ns) & 0x7f) << 1)
#define LAPD_CTRL_I5(nr, p) ((((nr) & 0x7f) << 1) | ((p) & 0x1))
#define LAPD_CTRL_S4(s) ((((s) & 0x3) << 2) | 0x1)
#define LAPD_CTRL_S5(nr, p) ((((nr) & 0x7f) << 1) | ((p) & 0x1))
#define LAPD_CTRL_U4(u, p) ((((u) & 0x1c) << (5-2)) | (((p) & 0x1) << 4) | (((u) & 0x3) << 2) | 0x3)

#define LAPD_CTRL_is_I(ctrl)	(((ctrl) & 0x1) == 0)
#define LAPD_CTRL_is_S(ctrl)	(((ctrl) & 0x3) == 1)
#define LAPD_CTRL_is_U(ctrl)	(((ctrl) & 0x3) == 3)

#define LAPD_CTRL_U_BITS(ctrl)	((((ctrl) & 0xC) >> 2) | ((ctrl) & 0xE0) >> 3)
#define LAPD_CTRL_U_PF(ctrl)	(((ctrl) >> 4) & 0x1)

#define LAPD_CTRL_S_BITS(ctrl)	(((ctrl) & 0xC) >> 2)
#define LAPD_CTRL_S_PF(ctrl)	(ctrl & 0x1)

#define LAPD_CTRL_I_Ns(ctrl)	(((ctrl) & 0xFE) >> 1)
#define LAPD_CTRL_I_P(ctrl)	(ctrl & 0x1)
#define LAPD_CTRL_Nr(ctrl)	(((ctrl) & 0xFE) >> 1)

#define LAPD_LEN(len)	((len << 2) | 0x1)
#define LAPD_EL	0x1

typedef enum {
	LAPD_TEI_NONE = 0,
	LAPD_TEI_ASSIGNED,
	LAPD_TEI_ACTIVE,
} lapd_tei_state;

const char *lapd_tei_states[] = {
	"NONE",
	"ASSIGNED",
	"ACTIVE",
};

/* Structure representing an allocated TEI within a LAPD instance. */
struct lapd_tei {
	struct llist_head list;
	struct lapd_instance *li;
	uint8_t tei;
	lapd_tei_state state;

	struct llist_head sap_list;
};

/* Structure representing a SAP within a TEI. It includes exactly one datalink
 * instance. */
struct lapd_sap {
	struct llist_head list;
	struct lapd_tei *tei;
	uint8_t sapi;

	struct lapd_datalink dl;
};

/* Resolve TEI structure from given numeric TEI */
static struct lapd_tei *teip_from_tei(struct lapd_instance *li, uint8_t tei)
{
	struct lapd_tei *lt;

	llist_for_each_entry(lt, &li->tei_list, list) {
		if (lt->tei == tei)
			return lt;
	}
	return NULL;
};

/* Change state of TEI */
static void lapd_tei_set_state(struct lapd_tei *teip, int newstate)
{
	LOGP(DLLAPD, LOGL_INFO, "LAPD state change on TEI %d: %s -> %s\n",
		teip->tei, lapd_tei_states[teip->state],
		lapd_tei_states[newstate]);
	teip->state = newstate;
};

/* Allocate a new TEI */
struct lapd_tei *lapd_tei_alloc(struct lapd_instance *li, uint8_t tei)
{
	struct lapd_tei *teip;

	teip = talloc_zero(li, struct lapd_tei);
	if (!teip)
		return NULL;

	teip->li = li;
	teip->tei = tei;
	llist_add(&teip->list, &li->tei_list);
	INIT_LLIST_HEAD(&teip->sap_list);

	lapd_tei_set_state(teip, LAPD_TEI_ASSIGNED);

	return teip;
}

/* Find a SAP within a given TEI */
static struct lapd_sap *lapd_sap_find(struct lapd_tei *teip, uint8_t sapi)
{
	struct lapd_sap *sap;

	llist_for_each_entry(sap, &teip->sap_list, list) {
		if (sap->sapi == sapi)
			return sap;
	}

	return NULL;
}

static int send_ph_data_req(struct lapd_msg_ctx *lctx, struct msgb *msg);
static int send_dlsap(struct osmo_dlsap_prim *dp, struct lapd_msg_ctx *lctx);

/* Allocate a new SAP within a given TEI */
static struct lapd_sap *lapd_sap_alloc(struct lapd_tei *teip, uint8_t sapi)
{
	struct lapd_sap *sap;
	struct lapd_datalink *dl;
	struct lapd_instance *li;
	int profile;
	int n201, k;
	int t200_sec, t200_usec;

	sap = talloc_zero(teip, struct lapd_sap);
	if (!sap)
		return NULL;

	LOGP(DLLAPD, LOGL_INFO, "LAPD Allocating SAP for SAPI=%u / TEI=%u\n",
		sapi, teip->tei);

	sap->sapi = sapi;
	sap->tei = teip;
	dl = &sap->dl;
	li = teip->li;

	profile = li->profile;
	switch (profile) {
	case LAPD_PROFILE_ABIS:
		n201 = 260;
		t200_sec = 0; t200_usec = 240000;
		if (sapi == 0)
			k = 2;
		else
			k = 1;
		break;
	case LAPD_PROFILE_ASAT:
		n201 = 260;
		t200_sec = 2; t200_usec = 500000;
		k = 15;
		li->short_address = 1;
		break;
	case LAPD_PROFILE_ISDN:
	default:
		n201 = 260;
		t200_sec = 1; t200_usec = 0;
		k = 7;
		break;
	}

	lapd_dl_init(dl, k, 128, n201);
	dl->use_sabme = 1; /* use SABME instead of SABM (GSM) */
	dl->send_ph_data_req = send_ph_data_req;
	dl->send_dlsap = send_dlsap;
	dl->lctx.dl = &sap->dl;
	dl->lctx.sapi = sapi;
	dl->lctx.tei = teip->tei;
	dl->lctx.n201 = n201;
	dl->t200_sec = t200_sec;
	dl->t200_usec = t200_usec;

	lapd_set_mode(&sap->dl, (teip->li->network_side) ? LAPD_MODE_NETWORK
							: LAPD_MODE_USER);

	llist_add(&sap->list, &teip->sap_list);

	return sap;
}

/* Free SAP instance, including the datalink */
static void lapd_sap_free(struct lapd_sap *sap)
{
	/* free datalink structures and timers */
	lapd_dl_exit(&sap->dl);

	llist_del(&sap->list);
	talloc_free(sap);
}

/* Free TEI instance */
static void lapd_tei_free(struct lapd_tei *teip)
{
	struct lapd_sap *sap, *sap2;

	llist_for_each_entry_safe(sap, sap2, &teip->sap_list, list) {
		lapd_sap_free(sap);
	}

	llist_del(&teip->list);
	talloc_free(teip);
}

/* Input function into TEI manager */
static int lapd_tei_receive(struct lapd_instance *li, uint8_t *data, int len)
{
	uint8_t entity = data[0];
	uint8_t ref = data[1];
	uint8_t mt = data[3];
	uint8_t action = data[4] >> 1;
	uint8_t e = data[4] & 1;
	uint8_t resp[8];
	struct lapd_tei *teip;
	struct msgb *msg;

	DEBUGP(DLLAPD, "LAPD TEIMGR: entity %x, ref %x, mt %x, action %x, "
		"e %x\n", entity, ref, mt, action, e);

	switch (mt) {
	case 0x01:	/* IDENTITY REQUEST */
		DEBUGP(DLLAPD, "LAPD TEIMGR: identity request for TEI %u\n",
			action);

		teip = teip_from_tei(li, action);
		if (!teip) {
			LOGP(DLLAPD, LOGL_INFO, "TEI MGR: New TEI %u\n",
				action);
			teip = lapd_tei_alloc(li, action);
			if (!teip)
				return -ENOMEM;
		}

		/* Send ACCEPT */
		memmove(resp, "\xfe\xff\x03\x0f\x00\x00\x02\x00", 8);
		resp[7] = (action << 1) | 1;
		msg = msgb_alloc_headroom(56, 56, "DL EST");
		msg->l2h = msgb_push(msg, 8);
		memcpy(msg->l2h, resp, 8);
		LOGP(DLLAPD, LOGL_DEBUG, "TX: %s\n",
			osmo_hexdump(msg->data, msg->len));
		li->transmit_cb(msg, li->transmit_cbdata);

		if (teip->state == LAPD_TEI_NONE)
			lapd_tei_set_state(teip, LAPD_TEI_ASSIGNED);
		break;
	default:
		LOGP(DLLAPD, LOGL_NOTICE, "LAPD TEIMGR: unknown mt %x "
			"action %x\n", mt, action);
		break;
	};

	return 0;
}

/* General input function for any data received for this LAPD instance */
int lapd_receive(struct lapd_instance *li, struct msgb *msg, int *error)
{
	int i;
	struct lapd_msg_ctx lctx;
	int rc;
	struct lapd_sap *sap;
	struct lapd_tei *teip;

	LOGP(DLLAPD, LOGL_DEBUG, "RX: %s\n", osmo_hexdump(msg->data, msg->len));
	if (msg->len < 2) {
		LOGP(DLLAPD, LOGL_ERROR, "LAPD receive len %d < 2, ignoring\n",
			msg->len);
		*error = LAPD_ERR_BAD_LEN;
		return -EINVAL;
	};
	msg->l2h = msg->data;

	memset(&lctx, 0, sizeof(lctx));

	i = 0;
	/* adress field */
	lctx.sapi = LAPD_ADDR_SAPI(msg->l2h[i]);
	lctx.cr = LAPD_ADDR_CR(msg->l2h[i]);
	lctx.lpd = 0;
	if (!LAPD_ADDR_EA(msg->l2h[i])) {
		i++;
		lctx.tei = LAPD_ADDR_TEI(msg->l2h[i]);
	}
	i++;
	/* control field */
	if (LAPD_CTRL_is_I(msg->l2h[i])) {
		lctx.format = LAPD_FORM_I;
		lctx.n_send = LAPD_CTRL_I_Ns(msg->l2h[i]);
		i++;
		lctx.n_recv = LAPD_CTRL_Nr(msg->l2h[i]);
		lctx.p_f = LAPD_CTRL_I_P(msg->l2h[i]);
	} else if (LAPD_CTRL_is_S(msg->l2h[i])) {
		lctx.format = LAPD_FORM_S;
		lctx.s_u = LAPD_CTRL_S_BITS(msg->l2h[i]);
		i++;
		lctx.n_recv = LAPD_CTRL_Nr(msg->l2h[i]);
		lctx.p_f = LAPD_CTRL_S_PF(msg->l2h[i]);
	} else if (LAPD_CTRL_is_U(msg->l2h[i])) {
		lctx.format = LAPD_FORM_U;
		lctx.s_u = LAPD_CTRL_U_BITS(msg->l2h[i]);
		lctx.p_f = LAPD_CTRL_U_PF(msg->l2h[i]);
	} else
		lctx.format = LAPD_FORM_UKN;
	i++;
	/* length */
	msg->l3h = msg->l2h + i;
	msgb_pull(msg, i);
	lctx.length = msg->len;

	/* perform TEI assignment, if received */
	if (lctx.tei == 127) {
		rc = lapd_tei_receive(li, msg->data, msg->len);
		msgb_free(msg);
		return rc;
	}

	/* resolve TEI and SAPI */
	teip = teip_from_tei(li, lctx.tei);
	if (!teip) {
		LOGP(DLLAPD, LOGL_NOTICE, "LAPD Unknown TEI %u\n", lctx.tei);
		*error = LAPD_ERR_UNKNOWN_TEI;
		msgb_free(msg);
		return -EINVAL;
	}
	sap = lapd_sap_find(teip, lctx.sapi);
	if (!sap) {
		LOGP(DLLAPD, LOGL_INFO, "LAPD No SAP for TEI=%u / SAPI=%u, "
			"allocating\n", lctx.tei, lctx.sapi);
		sap = lapd_sap_alloc(teip, lctx.sapi);
		if (!sap) {
			*error = LAPD_ERR_NO_MEM;
			msgb_free(msg);
			return -ENOMEM;
		}
	}
	lctx.dl = &sap->dl;
	lctx.n201 = lctx.dl->maxf;

	if (msg->len > lctx.n201) {
		LOGP(DLLAPD, LOGL_ERROR, "message len %d > N201(%d) "
			"(discarding)\n", msg->len, lctx.n201);
		msgb_free(msg);
		*error = LAPD_ERR_BAD_LEN;
		return -EINVAL;
	}

	/* send to LAPD */
	return lapd_ph_data_ind(msg, &lctx);
}

/* Start a (user-side) SAP for the specified TEI/SAPI on the LAPD instance */
int lapd_sap_start(struct lapd_instance *li, uint8_t tei, uint8_t sapi)
{
	struct lapd_sap *sap;
	struct lapd_tei *teip;
	struct osmo_dlsap_prim dp;
	struct msgb *msg;

	teip = teip_from_tei(li, tei);
	if (!teip)
		teip = lapd_tei_alloc(li, tei);

	sap = lapd_sap_find(teip, sapi);
	if (sap)
		return -EEXIST;

	sap = lapd_sap_alloc(teip, sapi);
	if (!sap)
		return -ENOMEM;

	LOGP(DLLAPD, LOGL_NOTICE, "LAPD DL-ESTABLISH request TEI=%d SAPI=%d\n",
		tei, sapi);

	/* prepare prim */
	msg = msgb_alloc_headroom(56, 56, "DL EST");
	msg->l3h = msg->data;
        osmo_prim_init(&dp.oph, 0, PRIM_DL_EST, PRIM_OP_REQUEST, msg);

        /* send to L2 */
        return lapd_recv_dlsap(&dp, &sap->dl.lctx);
}

/* Stop a (user-side) SAP for the specified TEI/SAPI on the LAPD instance */
int lapd_sap_stop(struct lapd_instance *li, uint8_t tei, uint8_t sapi)
{
	struct lapd_tei *teip;
	struct lapd_sap *sap;
	struct osmo_dlsap_prim dp;
	struct msgb *msg;

	teip = teip_from_tei(li, tei);
	if (!teip)
		return -ENODEV;

	sap = lapd_sap_find(teip, sapi);
	if (!sap)
		return -ENODEV;

	LOGP(DLLAPD, LOGL_NOTICE, "LAPD DL-RELEASE request TEI=%d SAPI=%d\n",
		tei, sapi);

	/* prepare prim */
	msg = msgb_alloc_headroom(56, 56, "DL REL");
	msg->l3h = msg->data;
        osmo_prim_init(&dp.oph, 0, PRIM_DL_REL, PRIM_OP_REQUEST, msg);

        /* send to L2 */
        return lapd_recv_dlsap(&dp, &sap->dl.lctx);
}

/* Transmit Data (DL-DATA request) on the given LAPD Instance / TEI / SAPI */
void lapd_transmit(struct lapd_instance *li, uint8_t tei, uint8_t sapi,
		   struct msgb *msg)
{
	struct lapd_tei *teip = teip_from_tei(li, tei);
	struct lapd_sap *sap;
	struct osmo_dlsap_prim dp;

	if (!teip) {
		LOGP(DLLAPD, LOGL_ERROR, "LAPD Cannot transmit on "
		     "non-existing TEI %u\n", tei);
		msgb_free(msg);
		return;
	}

	sap = lapd_sap_find(teip, sapi);
	if (!sap) {
		LOGP(DLLAPD, LOGL_INFO, "LAPD Tx on unknown SAPI=%u "
		     "in TEI=%u\n", sapi, tei);
		msgb_free(msg);
		return;
	}

	/* prepare prim */
	msg->l3h = msg->data;
        osmo_prim_init(&dp.oph, 0, PRIM_DL_DATA, PRIM_OP_REQUEST, msg);

        /* send to L2 */
        lapd_recv_dlsap(&dp, &sap->dl.lctx);
};

static int send_ph_data_req(struct lapd_msg_ctx *lctx, struct msgb *msg)
{
	struct lapd_datalink *dl = lctx->dl;
	struct lapd_sap *sap =
		container_of(dl, struct lapd_sap, dl);
	struct lapd_instance *li = sap->tei->li;
	int format = lctx->format;
	int addr_len;

	/* control field */
	switch (format) {
	case LAPD_FORM_I:
		msg->l2h = msgb_push(msg, 2);
		msg->l2h[0] = LAPD_CTRL_I4(lctx->n_send);
		msg->l2h[1] = LAPD_CTRL_I5(lctx->n_recv, lctx->p_f);
		break;
	case LAPD_FORM_S:
		msg->l2h = msgb_push(msg, 2);
		msg->l2h[0] = LAPD_CTRL_S4(lctx->s_u);
		msg->l2h[1] = LAPD_CTRL_S5(lctx->n_recv, lctx->p_f);
		break;
	case LAPD_FORM_U:
		msg->l2h = msgb_push(msg, 1);
		msg->l2h[0] = LAPD_CTRL_U4(lctx->s_u, lctx->p_f);
		break;
	default:
		msgb_free(msg);
		return -EINVAL;
	}
	/* address field */
	if (li->short_address && lctx->tei == 0)
		addr_len = 1;
	else
		addr_len = 2;
	msg->l2h = msgb_push(msg, addr_len);
	msg->l2h[0] = LAPD_ADDR2(lctx->sapi, lctx->cr);
	if (addr_len == 1)
		msg->l2h[0] |= 0x1;
	else
		msg->l2h[1] = LAPD_ADDR3(lctx->tei);

	/* forward frame to L1 */
	LOGP(DLLAPD, LOGL_DEBUG, "TX: %s\n", osmo_hexdump(msg->data, msg->len));
	li->transmit_cb(msg, li->transmit_cbdata);

	return 0;
}

/* A DL-SAP message is received from datalink instance and forwarded to L3 */
static int send_dlsap(struct osmo_dlsap_prim *dp, struct lapd_msg_ctx *lctx)
{
	struct lapd_datalink *dl = lctx->dl;
	struct lapd_sap *sap =
		container_of(dl, struct lapd_sap, dl);
	struct lapd_instance *li;
	uint8_t tei, sapi;
	char *op = (dp->oph.operation == PRIM_OP_INDICATION) ? "indication" 
							: "confirm";

	li = sap->tei->li;
	tei = lctx->tei;
	sapi = lctx->sapi;

	switch (dp->oph.primitive) {
	case PRIM_DL_EST:
		LOGP(DLLAPD, LOGL_NOTICE, "LAPD DL-ESTABLISH %s TEI=%d "
			"SAPI=%d\n", op, lctx->tei, lctx->sapi);
		break;
	case PRIM_DL_REL:
		LOGP(DLLAPD, LOGL_NOTICE, "LAPD DL-RELEASE %s TEI=%d "
			"SAPI=%d\n", op, lctx->tei, lctx->sapi);
		lapd_sap_free(sap);
		/* note: sap and dl is now gone, don't use it anymore */
		break;
	default:
		;
	}

	li->receive_cb(dp, tei, sapi, li->receive_cbdata);

	return 0;
}

/* Allocate a new LAPD instance */
struct lapd_instance *lapd_instance_alloc(int network_side,
	void (*tx_cb)(struct msgb *msg, void *cbdata), void *tx_cbdata,
	void (*rx_cb)(struct osmo_dlsap_prim *odp, uint8_t tei, uint8_t sapi, 
			void *rx_cbdata), void *rx_cbdata,
	enum lapd_profile profile)
{
	struct lapd_instance *li;

	li = talloc_zero(NULL, struct lapd_instance);
	if (!li)
		return NULL;

	li->network_side = network_side;
	li->transmit_cb = tx_cb;
	li->transmit_cbdata = tx_cbdata;
	li->receive_cb = rx_cb;
	li->receive_cbdata = rx_cbdata;
	li->profile = profile;

	INIT_LLIST_HEAD(&li->tei_list);

	return li;
}

void lapd_instance_free(struct lapd_instance *li)
{
	struct lapd_tei *teip, *teip2;

	/* Free all TEI instances */
	llist_for_each_entry_safe(teip, teip2, &li->tei_list, list) {
		lapd_tei_free(teip);
	}

	talloc_free(li);
}

/* IPA keep-alive FSM; Periodically transmit IPA_PING and expect IPA_PONG in return.
 *
 * (C) 2019 by Harald Welte <laforge@gnumonks.org>
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+
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
 */

#include <osmocom/core/fsm.h>
#include <osmocom/core/timer.h>
#include <osmocom/core/msgb.h>
#include <osmocom/core/logging.h>

#include <osmocom/gsm/protocol/ipaccess.h>

#include <osmocom/abis/ipa.h>

#define S(x)	(1 << (x))


/* generate a msgb containing an IPA CCM PING message */
static struct msgb *gen_ipa_ping(void)
{
	struct msgb *msg = msgb_alloc_headroom(64, 32, "IPA PING");
	if (!msg)
		return NULL;

	msgb_put_u8(msg, IPAC_MSGT_PING);
	ipa_msg_push_header(msg, IPAC_PROTO_IPACCESS);

	return msg;
}

enum osmo_ipa_keepalive_state {
	OSMO_IPA_KA_S_INIT,
	OSMO_IPA_KA_S_IDLE,		/* waiting for next interval */
	OSMO_IPA_KA_S_WAIT_RESP,	/* waiting for response to keepalive */
};

enum osmo_ipa_keepalive_event {
	OSMO_IPA_KA_E_START,
	OSMO_IPA_KA_E_STOP,
	OSMO_IPA_KA_E_PONG,
};

static const struct value_string ipa_keepalive_event_names[] = {
	OSMO_VALUE_STRING(OSMO_IPA_KA_E_START),
	OSMO_VALUE_STRING(OSMO_IPA_KA_E_STOP),
	OSMO_VALUE_STRING(OSMO_IPA_KA_E_PONG),
	{ 0, NULL }
};

enum ipa_fsm_timer {
	T_SEND_NEXT_PING = 1,
	T_PONG_NOT_RECEIVED = 2,
};

struct ipa_fsm_priv {
	struct ipa_keepalive_params params;

	struct ipa_server_conn *srv_conn;
	struct ipa_client_conn *client_conn;
	void *generic;
	ipa_keepalive_timeout_cb_t *timeout_cb;
	ipa_keepalive_send_cb_t *send_fn;
};

static void ipa_ka_init(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct ipa_fsm_priv *ifp = fi->priv;

	switch (event) {
	case OSMO_IPA_KA_E_START:
		osmo_fsm_inst_state_chg(fi, OSMO_IPA_KA_S_WAIT_RESP,
					ifp->params.wait_for_resp, T_PONG_NOT_RECEIVED);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

static void ipa_ka_wait_resp_onenter(struct osmo_fsm_inst *fi, uint32_t prev_state)
{
	struct ipa_fsm_priv *ifp = fi->priv;
	struct msgb *msg;

	/* Send an IPA PING to the peer */
	msg = gen_ipa_ping();
	OSMO_ASSERT(msg);

	if (ifp->send_fn && ifp->generic) {
		ifp->send_fn(fi, ifp->generic, msg);
		return;
	}

	if (ifp->srv_conn) {
		if (ifp->send_fn)
			ifp->send_fn(fi, ifp->srv_conn, msg);
		else
			ipa_server_conn_send(ifp->srv_conn, msg);
	}
	else {
		OSMO_ASSERT(ifp->client_conn);
		if (ifp->send_fn)
			ifp->send_fn(fi, ifp->client_conn, msg);
		else
			ipa_client_conn_send(ifp->client_conn, msg);
	}
}

static void ipa_ka_wait_resp(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	struct ipa_fsm_priv *ifp = fi->priv;

	switch (event) {
	case OSMO_IPA_KA_E_PONG:
		osmo_fsm_inst_state_chg(fi, OSMO_IPA_KA_S_IDLE,
					ifp->params.interval, T_SEND_NEXT_PING);
		break;
	default:
		OSMO_ASSERT(0);
	}
}

static int ipa_ka_fsm_timer_cb(struct osmo_fsm_inst *fi)
{
	struct ipa_fsm_priv *ifp = fi->priv;
	void *conn;

	switch (fi->T) {
	case T_SEND_NEXT_PING:
		/* send another PING */
		osmo_fsm_inst_state_chg(fi, OSMO_IPA_KA_S_WAIT_RESP,
					ifp->params.wait_for_resp, T_PONG_NOT_RECEIVED);
		return 0;
	case T_PONG_NOT_RECEIVED:
		LOGPFSML(fi, LOGL_NOTICE, "IPA keep-alive FSM timed out: PONG not received\n");
		/* PONG not received within time */
		if (ifp->srv_conn)
			conn = ifp->srv_conn;
		else if (ifp->client_conn)
			conn = ifp->client_conn;
		else
			conn = ifp->generic;
		if (ifp->timeout_cb)
			return ifp->timeout_cb(fi, conn);
		/* ask fsm core to terminate us */
		return 1;
	default:
		OSMO_ASSERT(0);
	}
}

static void ipa_ka_allstate_action(struct osmo_fsm_inst *fi, uint32_t event, void *data)
{
	switch (event) {
	case OSMO_IPA_KA_E_STOP:
		osmo_fsm_inst_state_chg(fi, OSMO_IPA_KA_S_INIT, 0, 0);
		break;
	default:
		OSMO_ASSERT(0);
		break;
	}
}

static const struct osmo_fsm_state ipa_keepalive_states[] = {
	[OSMO_IPA_KA_S_INIT] = {
		.name = "INIT",
		.in_event_mask = S(OSMO_IPA_KA_E_START),
		.out_state_mask = S(OSMO_IPA_KA_S_WAIT_RESP) | S(OSMO_IPA_KA_S_INIT),
		.action = ipa_ka_init,
	},
	[OSMO_IPA_KA_S_IDLE] = {
		.name = "IDLE",
		.out_state_mask = S(OSMO_IPA_KA_S_WAIT_RESP) | S(OSMO_IPA_KA_S_INIT),
		/* no permitted events aside from E_START, which is handled in allstate_events */
	},
	[OSMO_IPA_KA_S_WAIT_RESP] = {
		.name = "WAIT_RESP",
		.in_event_mask = S(OSMO_IPA_KA_E_PONG),
		.out_state_mask = S(OSMO_IPA_KA_S_IDLE) | S(OSMO_IPA_KA_S_INIT),
		.action = ipa_ka_wait_resp,
		.onenter = ipa_ka_wait_resp_onenter,
	},
};

static struct osmo_fsm ipa_keepalive_fsm = {
	.name = "IPA-KEEPALIVE",
	.states = ipa_keepalive_states,
	.num_states = ARRAY_SIZE(ipa_keepalive_states),
	.log_subsys = DLINP,
	.allstate_event_mask = S(OSMO_IPA_KA_E_STOP),
	.allstate_action = ipa_ka_allstate_action,
	.event_names = ipa_keepalive_event_names,
	.timer_cb = ipa_ka_fsm_timer_cb,
};

static __attribute__((constructor)) void on_dso_load(void)
{
	OSMO_ASSERT(osmo_fsm_register(&ipa_keepalive_fsm) == 0);
}


static struct osmo_fsm_inst *
__ipa_conn_alloc_keepalive_fsm(void *ctx, const struct ipa_keepalive_params *params, const char *id)
{
	struct osmo_fsm_inst *fi;
	struct ipa_fsm_priv *ifp;

	fi = osmo_fsm_inst_alloc(&ipa_keepalive_fsm, ctx, NULL, LOGL_DEBUG, id);
	if (!fi)
		return NULL;
	ifp = talloc_zero(fi, struct ipa_fsm_priv);
	if (!ifp) {
		osmo_fsm_inst_term(fi, OSMO_FSM_TERM_ERROR, NULL);
		return NULL;
	}
	memcpy(&ifp->params, params, sizeof(ifp->params));
	fi->priv = ifp;

	return fi;
}

/*! Create a new instance of an IPA keepalive FSM: Periodically transmit PING and expect PONG.
 *  \param[in] client The client connection for which to crate the FSM. Used as talloc context.
 *  \param[in] params Parameters describing the keepalive FSM time-outs.
 *  \param[in] id String used as identifier for the FSM.
 *  \returns pointer to the newly-created FSM instance; NULL in case of error. */
struct osmo_fsm_inst *ipa_client_conn_alloc_keepalive_fsm(struct ipa_client_conn *client,
							  const struct ipa_keepalive_params *params,
							  const char *id)
{
	struct osmo_fsm_inst *fi;
	struct ipa_fsm_priv *ifp;

	fi = __ipa_conn_alloc_keepalive_fsm(client, params, id);
	if (!fi)
		return NULL;
	ifp = fi->priv;
	ifp->client_conn = client;
	return fi;
}

/*! Create a new instance of an IPA keepalive FSM: Periodically transmit PING and expect PONG.
 *  \param[in] server The server connection for which to crate the FSM. Used as talloc context.
 *  \param[in] params Parameters describing the keepalive FSM time-outs.
 *  \param[in] id String used as identifier for the FSM.
 *  \returns pointer to the newly-created FSM instance; NULL in case of error. */
struct osmo_fsm_inst *ipa_server_conn_alloc_keepalive_fsm(struct ipa_server_conn *server,
							  const struct ipa_keepalive_params *params,
							  const char *id)
{
	struct osmo_fsm_inst *fi;
	struct ipa_fsm_priv *ifp;

	fi = __ipa_conn_alloc_keepalive_fsm(server, params, id);
	if (!fi)
		return NULL;
	ifp = fi->priv;
	ifp->srv_conn = server;
	return fi;
}

/*! Create a new instance of an IPA keepalive FSM: Periodically transmit PING and expect PONG.
 *  \param[in] ctx Talloc context.
 *  \param[in] data Data to pass to write/timeout cb.
 *  \param[in] params Parameters describing the keepalive FSM time-outs.
 *  \param[in] id String used as identifier for the FSM.
 *  \returns pointer to the newly-created FSM instance; NULL in case of error. */
struct osmo_fsm_inst *ipa_generic_conn_alloc_keepalive_fsm(void *ctx, void* data,
							   const struct ipa_keepalive_params *params,
							   const char *id)
{
	struct osmo_fsm_inst *fi;
	struct ipa_fsm_priv *ifp;
	
	fi = __ipa_conn_alloc_keepalive_fsm(ctx, params, id);
	if (!fi)
		return NULL;
	ifp = fi->priv;
	ifp->generic = data;
	return fi;
}

/*! Set a timeout call-back which is to be called once the peer doesn't respond anymore */
void ipa_keepalive_fsm_set_timeout_cb(struct osmo_fsm_inst *fi, ipa_keepalive_timeout_cb_t *cb)
{
	struct ipa_fsm_priv *ifp = fi->priv;
	OSMO_ASSERT(fi->fsm == &ipa_keepalive_fsm);
	ifp->timeout_cb = cb;
}

/*! Set a custom send callback for sending pings */
void ipa_keepalive_fsm_set_send_cb(struct osmo_fsm_inst *fi, ipa_keepalive_send_cb_t *fn)
{
	struct ipa_fsm_priv *ifp = fi->priv;
	OSMO_ASSERT(fi->fsm == &ipa_keepalive_fsm);
	ifp->send_fn = fn;
}

/*! Inform IPA Keepalive FSM that a PONG has been received. */
void ipa_keepalive_fsm_pong_received(struct osmo_fsm_inst *fi)
{
	OSMO_ASSERT(fi->fsm == &ipa_keepalive_fsm);
	osmo_fsm_inst_dispatch(fi, OSMO_IPA_KA_E_PONG, NULL);
}

/*! Start the ping/pong procedure of the IPA Keepalive FSM. */
void ipa_keepalive_fsm_start(struct osmo_fsm_inst *fi)
{
	struct ipa_fsm_priv *ifp = fi->priv;
	OSMO_ASSERT(fi->fsm == &ipa_keepalive_fsm);
	LOGPFSML(fi, LOGL_INFO, "Starting IPA keep-alive FSM (interval=%us wait=%us)\n",
		 ifp->params.interval, ifp->params.wait_for_resp);
	osmo_fsm_inst_dispatch(fi, OSMO_IPA_KA_E_START, NULL);
}

/*! Stop the ping/pong procedure of the IPA Keepalive FSM. */
void ipa_keepalive_fsm_stop(struct osmo_fsm_inst *fi)
{
	OSMO_ASSERT(fi->fsm == &ipa_keepalive_fsm);
	LOGPFSML(fi, LOGL_INFO, "Stopping IPA keep-alive FSM\n");
	osmo_fsm_inst_dispatch(fi, OSMO_IPA_KA_E_STOP, NULL);
}

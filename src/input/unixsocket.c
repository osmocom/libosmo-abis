/* OpenBSC Abis receive lapd over a unix socket */

/* (C) 2016 by sysmocom s.f.m.c. GmbH
 *
 * Author: Alexander Couzens <lynxis@fe80.eu>
 * Based on other e1_input drivers.
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

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <limits.h>
#include <string.h>

#include <osmocom/core/talloc.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/socket.h>

#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/lapd.h>
#include <osmocom/abis/e1_input.h>

#include "internal.h"

void *tall_unixsocket_ctx;
#define UNIXSOCKET_ALLOC_SIZE 1600
#define UNIXSOCKET_SOCK_PATH_DEFAULT "/tmp/osmo_abis_line_"

struct unixsocket_line {
	struct osmo_fd fd;
};

static int ts_want_write(struct e1inp_ts *e1i_ts);

static int unixsocket_exception_cb(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	LOGP(DLINP, LOGL_ERROR, "unixsocket: closing socket. Exception cb called.\n");

	close(bfd->fd);

	return 0;
}

static int unixsocket_read_cb(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct msgb *msg = msgb_alloc(UNIXSOCKET_ALLOC_SIZE, "UNIXSOCKET TS");
	int ret;

	if (!msg)
		return -ENOMEM;

	ret = read(bfd->fd, msg->data, UNIXSOCKET_ALLOC_SIZE - 16);
	if (ret == 0) {
		unixsocket_exception_cb(bfd);
		return ret;
	} else if (ret < 0) {
		perror("read ");
		return ret;
	}
	msgb_put(msg, ret);

	return e1inp_rx_ts_lapd(&line->ts[0], msg);
}

static void timeout_ts1_write(void *data)
{
	struct e1inp_ts *e1i_ts = (struct e1inp_ts *)data;

	/* trigger write of ts1, due to tx delay timer */
	ts_want_write(e1i_ts);
}

static int unixsocket_write_cb(struct osmo_fd *bfd)
{
	struct e1inp_line *line = bfd->data;
	struct e1inp_ts *e1i_ts = &line->ts[0];
	struct msgb *msg;
	struct e1inp_sign_link *sign_link;

	bfd->when &= ~BSC_FD_WRITE;

	/* get the next msg for this timeslot */
	msg = e1inp_tx_ts(e1i_ts, &sign_link);
	if (!msg) {
		/* no message after tx delay timer */
		LOGP(DLINP, LOGL_INFO, "unixsocket: no message available");
		return 0;
	}

	/* set tx delay timer for next event */
	e1i_ts->sign.tx_timer.cb = timeout_ts1_write;
	e1i_ts->sign.tx_timer.data = e1i_ts;

	osmo_timer_schedule(&e1i_ts->sign.tx_timer, 0, e1i_ts->sign.delay);

	LOGP(DLINP, LOGL_INFO, "unixsocket: sending: %s", msgb_hexdump(msg));
	lapd_transmit(e1i_ts->lapd, sign_link->tei,
			sign_link->sapi, msg);

	return 0;
}

static int unixsocket_cb(struct osmo_fd *bfd, unsigned int what)
{
	int ret = 0;

	if (what & BSC_FD_READ)
		ret = unixsocket_read_cb(bfd);
	if (what & BSC_FD_WRITE)
		ret = unixsocket_write_cb(bfd);

	return ret;
}

static int ts_want_write(struct e1inp_ts *e1i_ts)
{
	struct unixsocket_line *line = e1i_ts->line->driver_data;

	line->fd.when |= BSC_FD_WRITE;

	return 0;
}

/*!
 * \brief unixsocket_write_msg lapd callback for data to unixsocket
 * \param msg
 * \param cbdata
 */
static void unixsocket_write_msg(struct msgb *msg, void *cbdata)
{
	struct osmo_fd *bfd = cbdata;
	int ret;

	ret = write(bfd->fd, msg->data, msg->len);
	msgb_free(msg);
	if (ret == -1)
		unixsocket_exception_cb(bfd);
	else if (ret < 0)
		LOGP(DLMI, LOGL_NOTICE, "%s write failed %d\n", __func__, ret);
}

static int unixsocket_line_update(struct e1inp_line *line)
{
	struct unixsocket_line *config;
	char sock_path[PATH_MAX];
	int ret = 0;
	int i;

	if (line->sock_path)
		strcpy(sock_path, line->sock_path);
	else
		sprintf(sock_path, "%s%d", UNIXSOCKET_SOCK_PATH_DEFAULT,
			line->num);

	LOGP(DLINP, LOGL_NOTICE, "line update (line=%p)\n", line);

	if (!line->driver_data)
		line->driver_data = talloc_zero(line, struct unixsocket_line);

	if (!line->driver_data) {
		LOGP(DLINP, LOGL_ERROR, "unixsocket: OOM in line update\n");
		return -ENOMEM;
	}

	config = line->driver_data;

	config->fd.data = line;
	config->fd.when = BSC_FD_READ;
	config->fd.cb = unixsocket_cb;
	ret = osmo_sock_unix_init(SOCK_SEQPACKET, 0, sock_path, OSMO_SOCK_F_CONNECT);

	if (ret < 0) {
		talloc_free(config);
		return ret;
	}

	config->fd.fd = ret;
	if (osmo_fd_register(&config->fd) < 0) {
		close(config->fd.fd);
		return -EIO;
	}

	for (i = 0; i < ARRAY_SIZE(line->ts); i++) {
		struct e1inp_ts *e1i_ts = &line->ts[i];

		if (!e1i_ts->lapd)
			e1i_ts->lapd = lapd_instance_alloc(1,
				unixsocket_write_msg, &config->fd, e1inp_dlsap_up,
				e1i_ts, &lapd_profile_abis);
	}

	/* Ensure Superchannel is turned of when a new connection is made */
	e1inp_ericsson_set_altc(line, 0);

	return ret;
}

struct e1inp_driver unixsocket_driver = {
	.name = "unixsocket",
	.want_write = ts_want_write,
	.line_update = unixsocket_line_update,
	.default_delay = 0,
};

void e1inp_unixsocket_init(void)
{
	tall_unixsocket_ctx = talloc_named_const(libosmo_abis_ctx, 1, "unixsocket");
	e1inp_driver_register(&unixsocket_driver);
}

void e1inp_ericsson_set_altc(struct e1inp_line *unixline, int superchannel)
{
	struct unixsocket_line *config;
	struct msgb *msg;

	if (!unixline)
		return;

	if (unixline->driver != &unixsocket_driver) {
		LOGP(DLMI, LOGL_NOTICE, "altc is only supported by unixsocket\n");
		return;
	}

	config = unixline->driver_data;
	if (!config) {
		LOGP(DLMI, LOGL_NOTICE, "e1inp driver not yet initialized.\n");
		return;
	}


	msg = msgb_alloc_headroom(200, 100, "ALTC");

	/* magic */
	msgb_put_u32(msg, 0x23004200);
	msgb_put_u8(msg, superchannel ? 1 : 0);

	unixsocket_write_msg(msg, &config->fd);
}

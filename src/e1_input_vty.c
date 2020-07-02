/* E1 vty interface */
/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * SPDX-License-Identifier: AGPL-3.0+
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

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>

#include <osmocom/core/linuxlist.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/utils.h>
#include <osmocom/gsm/gsm_utils.h>

#include <osmocom/vty/command.h>
#include <osmocom/vty/buffer.h>
#include <osmocom/vty/vty.h>
#include <osmocom/vty/logging.h>
#include <osmocom/vty/misc.h>
#include <osmocom/vty/telnet_interface.h>

#include <osmocom/abis/e1_input.h>
#include <osmocom/abis/ipa.h>

/* CONFIG */

#define E1_DRIVER_NAMES		"(misdn|misdn_lapd|dahdi|e1d|ipa|unixsocket)"
#define E1_DRIVER_HELP		"mISDN supported E1 Card (kernel LAPD)\n" \
				"mISDN supported E1 Card (userspace LAPD)\n" \
				"DAHDI supported E1/T1/J1 Card\n" \
				"osmo-e1d supported E1 interface\n" \
				"IPA TCP/IP input\n" \
				"Unix socket input\n"

#define E1_LINE_HELP		"Configure E1/T1/J1 Line\n" "Line Number\n"

DEFUN(cfg_e1line_driver, cfg_e1_line_driver_cmd,
	"e1_line <0-255> driver " E1_DRIVER_NAMES,
	E1_LINE_HELP "Set driver for this line\n"
	E1_DRIVER_HELP)
{
	struct e1inp_line *line;
	int e1_nr = atoi(argv[0]);

	line = e1inp_line_find(e1_nr);
	if (line) {
		vty_out(vty, "%% Line %d already exists%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}
	line = e1inp_line_create(e1_nr, argv[1]);
	if (!line) {
		vty_out(vty, "%% Error creating line %d%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_e1line_port, cfg_e1_line_port_cmd,
	"e1_line <0-255> port <0-255>",
	E1_LINE_HELP "Set physical port/span/card number\n"
	"E1/T1 Port/Span/Card number\n")
{
	struct e1inp_line *line;
	int e1_nr = atoi(argv[0]);

	line = e1inp_line_find(e1_nr);
	if (!line) {
		vty_out(vty, "%% Line %d doesn't exist%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	line->port_nr = atoi(argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_e1line_socket, cfg_e1_line_socket_cmd,
	"e1_line <0-255> socket .SOCKET",
	E1_LINE_HELP "Set socket path for unixsocket\n"
	"socket path\n")
{
	struct e1inp_line *line;
	int e1_nr = atoi(argv[0]);
	struct sockaddr_un sun;

	/* Don't exceed the maximum unix socket path length, including a NUL byte. See the unix(7) man page.*/
	if (strlen(argv[1]) > sizeof(sun.sun_path) - 1) {
		vty_out(vty, "%% Socket path length exceeds %zd bytes: '%s'%s",
			sizeof(sun.sun_path) - 1, argv[1], VTY_NEWLINE);
		return CMD_WARNING;
	}

	line = e1inp_line_find(e1_nr);
	if (!line) {
		vty_out(vty, "%% Line %d doesn't exist%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	line->sock_path = talloc_strdup(line, argv[1]);

	return CMD_SUCCESS;
}

#define KEEPALIVE_HELP "Enable keep-alive probing\n"
static int set_keepalive_params(struct vty *vty, int e1_nr,
				int idle, int num_probes, int probe_interval)
{
	struct e1inp_line *line = e1inp_line_find(e1_nr);

	if (!line) {
		vty_out(vty, "%% Line %d doesn't exist%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}
	if (!line->driver->has_keepalive && num_probes != 0) {
		vty_out(vty, "%% Driver '%s' does not support keep alive%s",
			line->driver->name, VTY_NEWLINE);
		return CMD_WARNING;
	}

	line->keepalive_idle_timeout = idle;
	line->keepalive_num_probes = num_probes;
	line->keepalive_probe_interval = probe_interval;

	return CMD_SUCCESS;
}

DEFUN(cfg_e1line_keepalive, cfg_e1_line_keepalive_cmd,
	"e1_line <0-255> keepalive",
	E1_LINE_HELP KEEPALIVE_HELP)
{
	return set_keepalive_params(vty, atoi(argv[0]),
				    E1INP_USE_DEFAULT, E1INP_USE_DEFAULT,
				    E1INP_USE_DEFAULT);
}

DEFUN(cfg_e1line_keepalive_params, cfg_e1_line_keepalive_params_cmd,
	"e1_line <0-255> keepalive <1-300> <1-20> <1-300>",
	E1_LINE_HELP KEEPALIVE_HELP
	"Idle interval in seconds before probes are sent\n"
	"Number of probes to sent\n"
	"Delay between probe packets in seconds\n")
{
	return set_keepalive_params(vty, atoi(argv[0]),
				    atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
}

DEFUN(cfg_e1line_no_keepalive, cfg_e1_line_no_keepalive_cmd,
	"no e1_line <0-255> keepalive",
	NO_STR E1_LINE_HELP KEEPALIVE_HELP)
{
	return set_keepalive_params(vty, atoi(argv[0]), 0, 0, 0);
}

#define IPA_KEEPALIVE_HELP "Enable IPA PING/PONG keep-alive\n"
static int set_ipa_keepalive_params(struct vty *vty, int e1_nr, int interval, int wait_for_resp)
{
	struct e1inp_line *line = e1inp_line_find(e1_nr);

	if (!line) {
		vty_out(vty, "%% Line %d doesn't exist%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	if (strcmp(line->driver->name, "ipa") != 0) {
		vty_out(vty, "%% Line %d doesn't use the ipa driver%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}

	TALLOC_FREE(line->ipa_kap);
	if (interval) {
		line->ipa_kap = talloc_zero(line, struct ipa_keepalive_params);
		line->ipa_kap->wait_for_resp = wait_for_resp;
		line->ipa_kap->interval = interval;
	}

	return CMD_SUCCESS;
}

DEFUN(cfg_e1line_ipa_keepalive, cfg_e1_line_ipa_keepalive_cmd,
	"e1_line <0-255> ipa-keepalive <1-300> <1-300>",
	E1_LINE_HELP IPA_KEEPALIVE_HELP
	"Idle interval in seconds before probes are sent\n"
	"Time to wait for PONG response\n")
{
	return set_ipa_keepalive_params(vty, atoi(argv[0]), atoi(argv[1]), atoi(argv[2]));
}

DEFUN(cfg_e1line_no_ipa_keepalive, cfg_e1_line_no_ipa_keepalive_cmd,
	"no e1_line <0-255> ipa-keepalive",
	NO_STR E1_LINE_HELP IPA_KEEPALIVE_HELP)
{
	return set_ipa_keepalive_params(vty, atoi(argv[0]), 0, 0);
}

DEFUN(cfg_e1line_name, cfg_e1_line_name_cmd,
	"e1_line <0-255> name .LINE",
	E1_LINE_HELP "Set name for this line\n" "Human readable name\n")
{
	struct e1inp_line *line;
	int e1_nr = atoi(argv[0]);

	line = e1inp_line_find(e1_nr);
	if (!line) {
		vty_out(vty, "%% Line %d doesn't exist%s", e1_nr, VTY_NEWLINE);
		return CMD_WARNING;
	}
	if (line->name) {
		talloc_free((void *)line->name);
		line->name = NULL;
	}
	line->name = talloc_strdup(line, argv[1]);

	return CMD_SUCCESS;
}

DEFUN(cfg_e1_pcap, cfg_e1_pcap_cmd,
	"pcap .FILE",
	"Setup a pcap recording of all E1 traffic\n"
	"Filename to save the packets to\n")
{
	int fd;

	fd = open(argv[0], O_WRONLY | O_CREAT | O_TRUNC, 0660);
	if (fd < 0) {
		vty_out(vty, "Failed to setup E1 pcap recording to %s.%s", argv[0], VTY_NEWLINE);
		return CMD_WARNING;
	}

	e1_set_pcap_fd(fd);

	return CMD_SUCCESS;
}

DEFUN(cfg_e1_no_pcap, cfg_e1_no_pcap_cmd,
	"no pcap",
	NO_STR "Disable pcap recording of all E1 traffic\n")
{
	e1_set_pcap_fd(-1);
	return CMD_SUCCESS;
}

DEFUN(cfg_e1inp, cfg_e1inp_cmd,
	"e1_input",
	"Configure E1/T1/J1 TDM input\n")
{
	vty->node = L_E1INP_NODE;

	return CMD_SUCCESS;
}

DEFUN(cfg_ipa_bind,
      cfg_ipa_bind_cmd,
      "ipa bind A.B.C.D",
      "ipa driver config\n"
      "Set ipa local bind address\n"
      "Listen on this IP address (default 0.0.0.0)\n")
{
	e1inp_ipa_set_bind_addr(argv[0]);
	return CMD_SUCCESS;
}

static int e1inp_config_write(struct vty *vty)
{
	struct e1inp_line *line;

	if (llist_empty(&e1inp_line_list))
		return CMD_SUCCESS;

	vty_out(vty, "e1_input%s", VTY_NEWLINE);

	llist_for_each_entry(line, &e1inp_line_list, list) {
		vty_out(vty, " e1_line %u driver %s%s", line->num,
			line->driver->name, VTY_NEWLINE);
		vty_out(vty, " e1_line %u port %u%s", line->num,
			line->port_nr, VTY_NEWLINE);
		if (line->name)
			vty_out(vty, " e1_line %u name %s%s", line->num,
				line->name, VTY_NEWLINE);
		if (!line->keepalive_num_probes)
			vty_out(vty, " no e1_line %u keepalive%s", line->num,
				VTY_NEWLINE);
		else if (line->keepalive_idle_timeout == E1INP_USE_DEFAULT &&
			 line->keepalive_num_probes == E1INP_USE_DEFAULT &&
			 line->keepalive_probe_interval == E1INP_USE_DEFAULT)
			vty_out(vty, " e1_line %u keepalive%s", line->num,
				VTY_NEWLINE);
		else
			vty_out(vty, " e1_line %u keepalive %d %d %d%s",
				line->num,
				line->keepalive_idle_timeout,
				line->keepalive_num_probes,
				line->keepalive_probe_interval,
				VTY_NEWLINE);
		if (line->ipa_kap)
			vty_out(vty, " e1_line %u ipa-keepalive %d %d%s", line->num,
				line->ipa_kap->interval, line->ipa_kap->wait_for_resp,
				VTY_NEWLINE);
	}

	const char *ipa_bind = e1inp_ipa_get_bind_addr();
	if (ipa_bind && (strcmp(ipa_bind, "0.0.0.0") != 0))
		vty_out(vty, " ipa bind %s%s",
			ipa_bind, VTY_NEWLINE);

	return CMD_SUCCESS;
}

/* SHOW */

static void e1drv_dump_vty(struct vty *vty, struct e1inp_driver *drv)
{
	vty_out(vty, "E1 Input Driver %s%s", drv->name, VTY_NEWLINE);
}

DEFUN(show_e1drv,
      show_e1drv_cmd,
      "show e1_driver",
	SHOW_STR "Display information about available E1 drivers\n")
{
	struct e1inp_driver *drv;

	llist_for_each_entry(drv, &e1inp_driver_list, list)
		e1drv_dump_vty(vty, drv);

	return CMD_SUCCESS;
}

static void e1line_dump_vty(struct vty *vty, struct e1inp_line *line,
			    int stats)
{
	vty_out(vty, "E1 Line Number %u, Name %s, Driver %s%s",
		line->num, line->name ? line->name : "",
		line->driver->name, VTY_NEWLINE);
	if (line->driver->vty_show)
		line->driver->vty_show(vty, line);
	if (stats)
		vty_out_rate_ctr_group(vty, " ", line->rate_ctr);
}

DEFUN(show_e1line,
      show_e1line_cmd,
      "show e1_line [line_nr] [stats]",
	SHOW_STR "Display information about a E1 line\n"
	"E1 Line Number\n" "Include statistics\n")
{
	struct e1inp_line *line;
	int stats = 0;

	if (argc >= 1 && strcmp(argv[0], "stats")) {
		int num = atoi(argv[0]);
		if (argc >= 2)
			stats = 1;
		llist_for_each_entry(line, &e1inp_line_list, list) {
			if (line->num == num) {
				e1line_dump_vty(vty, line, stats);
				return CMD_SUCCESS;
			}
		}
		return CMD_WARNING;
	}

	if (argc >= 1 && !strcmp(argv[0], "stats"))
		stats = 1;

	llist_for_each_entry(line, &e1inp_line_list, list)
		e1line_dump_vty(vty, line, stats);

	return CMD_SUCCESS;
}

static void e1ts_dump_vty(struct vty *vty, struct e1inp_ts *ts)
{
	if (ts->type == E1INP_TS_TYPE_NONE)
		return;
	vty_out(vty, "E1 Timeslot %2u of Line %u is Type %s%s",
		ts->num, ts->line->num, e1inp_tstype_name(ts->type),
		VTY_NEWLINE);
}

DEFUN(show_e1ts,
      show_e1ts_cmd,
      "show e1_timeslot [line_nr] [ts_nr]",
	SHOW_STR "Display information about a E1 timeslot\n"
	"E1 Line Number\n" "E1 Timeslot Number\n")
{
	struct e1inp_line *line = NULL;
	struct e1inp_ts *ts;
	int ts_nr;

	if (argc <= 0) {
		llist_for_each_entry(line, &e1inp_line_list, list) {
			for (ts_nr = 0; ts_nr < line->num_ts; ts_nr++) {
				ts = &line->ts[ts_nr];
				e1ts_dump_vty(vty, ts);
			}
		}
		return CMD_SUCCESS;
	}
	if (argc >= 1) {
		int num = atoi(argv[0]);
		struct e1inp_line *l;
		llist_for_each_entry(l, &e1inp_line_list, list) {
			if (l->num == num) {
				line = l;
				break;
			}
		}
		if (!line) {
			vty_out(vty, "E1 line %s is invalid%s",
				argv[0], VTY_NEWLINE);
			return CMD_WARNING;
		}
	}
	if (argc >= 2) {
		ts_nr = atoi(argv[1]);
		if (ts_nr >= line->num_ts) {
			vty_out(vty, "E1 timeslot %s is invalid%s",
				argv[1], VTY_NEWLINE);
			return CMD_WARNING;
		}
		ts = &line->ts[ts_nr];
		e1ts_dump_vty(vty, ts);
		return CMD_SUCCESS;
	} else {
		for (ts_nr = 0; ts_nr < line->num_ts; ts_nr++) {
			ts = &line->ts[ts_nr];
			e1ts_dump_vty(vty, ts);
		}
		return CMD_SUCCESS;
	}
	return CMD_SUCCESS;
}


struct cmd_node e1inp_node = {
	L_E1INP_NODE,
	"%s(config-e1_input)# ",
	1,
};

int e1inp_vty_init(void)
{
	install_element(CONFIG_NODE, &cfg_e1inp_cmd);
	install_node(&e1inp_node, e1inp_config_write);

	install_element(L_E1INP_NODE, &cfg_e1_pcap_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_no_pcap_cmd);

	install_element(L_E1INP_NODE, &cfg_e1_line_driver_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_port_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_socket_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_name_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_keepalive_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_keepalive_params_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_no_keepalive_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_ipa_keepalive_cmd);
	install_element(L_E1INP_NODE, &cfg_e1_line_no_ipa_keepalive_cmd);

	install_element(L_E1INP_NODE, &cfg_ipa_bind_cmd);

	install_element_ve(&show_e1drv_cmd);
	install_element_ve(&show_e1line_cmd);
	install_element_ve(&show_e1ts_cmd);

	return 0;
}

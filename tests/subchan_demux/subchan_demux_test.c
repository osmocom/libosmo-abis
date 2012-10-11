/* Subchan Demux syncing test */

/* (C) 2012 by Holger Hans Peter Freyther
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

#include <osmocom/abis/subchan_demux.h>
#include <osmocom/core/utils.h>

#include <stdio.h>
#include <string.h>

static int data_cb(struct subch_demux *demux, int ch, uint8_t *data, int len, void *p)
{
	printf("DATA_CB Channel(%d): %s\n",
		ch, osmo_hexdump(data, len));
	return 0;
}

static void test_csd(void)
{
	struct subch_demux demux;
	memset(&demux, 0, sizeof(demux));
	subch_demux_init(&demux);

	demux.out_cb = data_cb;

	/* Push data into the demuxer and see what happens. */
	printf("Testing the csd sync.\n");

}

int main(int argc, char **argv)
{
	printf("Testing the subchannel demux.\n");

	/* run the tests */
	test_csd();

	printf("No crashes.\n");
	return 0;
}

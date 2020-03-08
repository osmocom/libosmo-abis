#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <osmocom/core/utils.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/logging.h>
#include <osmocom/core/application.h>
#include <osmocom/trau/osmo_ortp.h>

static void *tall_test;

static struct osmo_rtp_socket *create_bind_any(void)
{
	struct osmo_rtp_socket *rs;
	int i;

	rs = osmo_rtp_socket_create(tall_test, 0);
	for (i = 16384; i < 65535; i+=2) {
		if (osmo_rtp_socket_bind(rs, "0.0.0.0", i) == 0)
			return rs;
	}
	osmo_rtp_socket_free(rs);
	return NULL;
}

/* test if binding two sockets to the same local port fails (See OS#4444) */
static void test_bind_fail(void)
{
	struct osmo_rtp_socket *rs[2];
	uint32_t ip;
	int rc, port[2];

	printf("First bind to any random local port...\n");
	rs[0] = create_bind_any();
	OSMO_ASSERT(rs[0]);

	/* then obtain the local port */
	rc = osmo_rtp_get_bound_ip_port(rs[0], &ip, &port[0]);
	OSMO_ASSERT(rc == 0);

	printf("Now try to bind another socket to the same port: ");
	rs[1] = osmo_rtp_socket_create(tall_test, 0);
	OSMO_ASSERT(rs[1]);
	rc = osmo_rtp_socket_bind(rs[1], "0.0.0.0", port[0]);
	if (rc == -1 && errno == EADDRINUSE)
		printf("OK (EADDRINUSE)\n");
	else {
		printf("FAILED - second bind to port %u worked\n", port[0]);
		fflush(stdout);
		OSMO_ASSERT(0);
	}

	osmo_rtp_socket_free(rs[0]);
	osmo_rtp_socket_free(rs[1]);
}

#define DTEST 0

static const struct log_info_cat bts_test_cat[] = {
	[DTEST] = {
		.name = "DTEST",
		.description = "test",
		.color = "\033[1;35m",
		.enabled = 1, .loglevel = LOGL_NOTICE,
	},
};

static const struct log_info bts_test_log_info = {
	.filter_fn = NULL,
	.cat = bts_test_cat,
	.num_cat = ARRAY_SIZE(bts_test_cat),
};


int main(int argc, char **argv)
{
	tall_test = talloc_named_const(NULL, 1, "rtp_test");
	osmo_init_logging2(tall_test, &bts_test_log_info);
	osmo_rtp_init(tall_test);

	test_bind_fail();

	exit(0);
}


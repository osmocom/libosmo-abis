AM_CPPFLAGS = -I$(top_srcdir)/include
AM_CFLAGS=-Wall -g $(LIBOSMONETIF_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCORE_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(COVERAGE_LDFLAGS)

noinst_PROGRAMS = \
	lapd-over-datagram-user \
	lapd-over-datagram-network \
	$(NULL)

lapd_over_datagram_user_SOURCES = lapd-over-datagram/lapd-over-datagram-user.c
lapd_over_datagram_user_LDADD = \
	$(top_builddir)/src/libosmoabis.la \
	$(LIBOSMONETIF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)

lapd_over_datagram_network_SOURCES = lapd-over-datagram/lapd-over-datagram-network.c
lapd_over_datagram_network_LDADD = \
	$(top_builddir)/src/libosmoabis.la \
	$(LIBOSMONETIF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(NULL)

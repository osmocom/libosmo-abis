# This is _NOT_ the library release version, it's an API version.
# Please read chapter "Library interface versions" of the libtool documentation
# before making any modifications: https://www.gnu.org/software/libtool/manual/html_node/Versioning.html
ABIS_LIBVERSION=16:0:1
TRAU_LIBVERSION=12:0:1

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(top_builddir)
AM_CFLAGS= -Wall \
	$(LIBOSMONETIF_CFLAGS) \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOE1D_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(COVERAGE_CFLAGS) \
	$(NULL)
AM_LDFLAGS = $(COVERAGE_LDFLAGS)
COMMONLIBS = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOE1D_LIBS) # libosmocodec not [yet] needed here

lib_LTLIBRARIES = libosmoabis.la libosmotrau.la

libosmoabis_la_LDFLAGS = $(AM_LDFLAGS) \
			 -version-info $(ABIS_LIBVERSION) \
			 -no-undefined \
			 $(NULL)
libosmoabis_la_LIBADD = $(LIBOSMONETIF_LIBS) $(COMMONLIBS)
libosmoabis_la_SOURCES = init.c \
			 e1_input.c \
			 e1_input_vty.c \
			 subchan_demux.c \
			 trau_frame.c \
			 input/e1d.c \
			 input/ipa.c \
			 input/ipa_keepalive.c \
			 input/ipaccess.c \
			 input/lapd.c \
			 input/lapd_pcap.c \
			 input/misdn.c \
			 input/rs232.c \
			 input/unixsocket.c
if ENABLE_DAHDI
libosmoabis_la_SOURCES += input/dahdi.c
endif

libosmotrau_la_CFLAGS = $(AM_CFLAGS) $(ORTP_CFLAGS)
libosmotrau_la_LDFLAGS = $(AM_LDFLAGS) \
			 -version-info $(TRAU_LIBVERSION) \
			 -no-undefined \
			 $(NULL)
libosmotrau_la_LIBADD = $(COMMONLIBS) $(LIBOSMOCODEC_LIBS) $(ORTP_LIBS)
libosmotrau_la_SOURCES = trau/amr_rtp.c \
			 trau/amr_trau.c \
			 trau/csd_ra2.c \
			 trau/csd_v110.c \
			 trau/raa_prime_decode.c \
			 trau/raa_prime_encode.c \
			 trau/tfo_frame.c \
			 trau/trau_frame.c \
			 trau/trau_pcu_ericsson.c \
			 trau/trau_sync.c \
			 trau/trau_rtp_conv.c \
			 trau/twts007.c

if ENABLE_ORTP
libosmotrau_la_SOURCES += trau/osmo_ortp.c
endif

noinst_HEADERS = trau/ubit_buf.h

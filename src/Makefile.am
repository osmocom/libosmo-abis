# This is _NOT_ the library release version, it's an API version.
# Please read chapter "Library interface versions" of the libtool documentation
# before making any modifications: https://www.gnu.org/software/libtool/manual/html_node/Versioning.html
ABIS_LIBVERSION=11:1:1
TRAU_LIBVERSION=7:0:5

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(top_builddir)
AM_CFLAGS= -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOE1D_CFLAGS) $(LIBOSMOCODEC_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(COVERAGE_LDFLAGS)
COMMONLIBS = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOE1D_LIBS) # libosmocodec not [yet] needed here

lib_LTLIBRARIES = libosmoabis.la libosmotrau.la

libosmoabis_la_LDFLAGS = $(AM_LDFLAGS) -version-info $(ABIS_LIBVERSION)
libosmoabis_la_LIBADD = $(COMMONLIBS)
libosmoabis_la_SOURCES = init.c \
			 e1_input.c \
			 e1_input_vty.c \
			 ipa_proxy.c \
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
libosmotrau_la_LDFLAGS = $(AM_LDFLAGS) -version-info $(TRAU_LIBVERSION)
libosmotrau_la_LIBADD = $(COMMONLIBS) $(ORTP_LIBS)
libosmotrau_la_SOURCES = trau/osmo_ortp.c \
			 trau/trau_frame.c \
			 trau/trau_sync.c \
			 trau/trau_rtp_conv.c

noinst_HEADERS = trau/ubit_buf.h

SUBDIRS=input

# This is _NOT_ the library release version, it's an API version.
# Please read Chapter 6 "Library interface versions" of the libtool documentation before making any modification
LIBVERSION=0:0:0

INCLUDES = $(all_includes) -I$(top_srcdir)/include -I$(top_builddir)
AM_CFLAGS= -fPIC -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(COVERAGE_LDFLAGS)

lib_LTLIBRARIES = libosmoabis.la

libosmoabis_la_LIBADD = input/libosmoabis-input.la
libosmoabis_la_SOURCES = init.c \
			 e1_input.c \
			 e1_input_vty.c \
			 ipa_proxy.c \
			 subchan_demux.c \
			 trau_frame.c

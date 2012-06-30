noinst_LTLIBRARIES = libosmoabis-input.la

INCLUDES = $(all_includes) -I$(top_srcdir)/include -I$(top_builddir)
AM_CFLAGS= -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(COVERAGE_LDFLAGS)

libosmoabis_input_la_SOURCES  = dahdi.c		\
				hsl.c		\
				ipa.c		\
				ipaccess.c	\
				lapd.c		\
				misdn.c		\
				rs232.c

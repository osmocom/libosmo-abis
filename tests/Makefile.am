INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS=-Wall -g $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(COVERAGE_LDFLAGS)

noinst_PROGRAMS = e1inp_ipa_bsc_test

e1inp_ipa_bsc_test_SOURCES = e1inp_ipa_bsc_test.c

e1inp_ipa_bsc_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)

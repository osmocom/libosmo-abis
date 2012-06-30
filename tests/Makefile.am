INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS=-Wall -g $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(COVERAGE_LDFLAGS)

check_PROGRAMS = e1inp_ipa_bsc_test	\
		  e1inp_ipa_bts_test	\
		  e1inp_hsl_bsc_test	\
		  e1inp_hsl_bts_test	\
		  ipa_proxy_test

e1inp_ipa_bsc_test_SOURCES = e1inp_ipa_bsc_test.c
e1inp_ipa_bsc_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)

e1inp_ipa_bts_test_SOURCES = e1inp_ipa_bts_test.c
e1inp_ipa_bts_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)

e1inp_hsl_bsc_test_SOURCES = e1inp_hsl_bsc_test.c
e1inp_hsl_bsc_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)

e1inp_hsl_bts_test_SOURCES = e1inp_hsl_bts_test.c
e1inp_hsl_bts_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)

ipa_proxy_test_SOURCES = ipa_proxy_test.c
ipa_proxy_test_LDADD = $(top_builddir)/src/libosmoabis.la \
			$(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) \
			$(LIBOSMOVTY_LIBS)

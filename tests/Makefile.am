AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS=-Wall -g $(LIBOSMONETIF_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCORE_CFLAGS) $(COVERAGE_CFLAGS)
AM_LDFLAGS = $(COVERAGE_LDFLAGS) -no-install

check_PROGRAMS = e1inp_ipa_bsc_test	\
		  e1inp_ipa_bts_test	\
		  subchan_demux/subchan_demux_test \
		  ipa_recv/ipa_recv_test \
		  raa_prime/test_dec \
		  raa_prime/test_enc \
		  tfo/rtp2tfo \
		  tfo/tfo_extr_test \
		  trau_conv/rtp2trau_gen \
		  trau_conv/trau16_to_rtp \
		  trau_conv/trau2rtp_gen \
		  trau_sync/trau_sync_test \
		  trau_pcu_ericsson/trau_pcu_ericsson_test

if ENABLE_ORTP
check_PROGRAMS += rtp_test/rtp_test
endif

ABIS_LA_LIBS = \
	$(top_builddir)/src/libosmoabis.la \
	$(LIBOSMONETIF_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(NULL)

TRAU_LA_LIBS = \
	$(top_builddir)/src/libosmotrau.la \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(NULL)

e1inp_ipa_bsc_test_SOURCES = e1inp_ipa_bsc_test.c
e1inp_ipa_bsc_test_LDADD = $(ABIS_LA_LIBS)

e1inp_ipa_bts_test_SOURCES = e1inp_ipa_bts_test.c
e1inp_ipa_bts_test_LDADD = $(ABIS_LA_LIBS)

subchan_demux_subchan_demux_test_SOURCES = subchan_demux/subchan_demux_test.c
subchan_demux_subchan_demux_test_LDADD = $(ABIS_LA_LIBS)

ipa_recv_ipa_recv_test_SOURCES = ipa_recv/ipa_recv_test.c
ipa_recv_ipa_recv_test_LDADD = $(ABIS_LA_LIBS)

raa_prime_test_dec_SOURCES = raa_prime/test_dec.c
raa_prime_test_dec_LDADD = $(TRAU_LA_LIBS)

raa_prime_test_enc_SOURCES = raa_prime/test_enc.c
raa_prime_test_enc_LDADD = $(TRAU_LA_LIBS)

rtp_test_rtp_test_SOURCES = rtp_test/rtp_test.c
rtp_test_rtp_test_LDADD = $(TRAU_LA_LIBS)

tfo_rtp2tfo_SOURCES = tfo/rtp2tfo.c trau_conv/tw5reader.c
tfo_rtp2tfo_LDADD = $(TRAU_LA_LIBS)

tfo_tfo_extr_test_SOURCES = tfo/tfo_extr_test.c
tfo_tfo_extr_test_LDADD = $(TRAU_LA_LIBS)

trau_conv_rtp2trau_gen_SOURCES = trau_conv/rtp2trau_gen.c trau_conv/tw5reader.c
trau_conv_rtp2trau_gen_LDADD = $(TRAU_LA_LIBS)

trau_conv_trau16_to_rtp_SOURCES = trau_conv/trau16_to_rtp.c
trau_conv_trau16_to_rtp_LDADD = $(TRAU_LA_LIBS)

trau_conv_trau2rtp_gen_SOURCES = trau_conv/trau2rtp_gen.c
trau_conv_trau2rtp_gen_LDADD = $(TRAU_LA_LIBS)

trau_sync_trau_sync_test_SOURCES = trau_sync/trau_sync_test.c
trau_sync_trau_sync_test_LDADD = $(TRAU_LA_LIBS)

trau_pcu_ericsson_trau_pcu_ericsson_test_SOURCES = trau_pcu_ericsson/trau_pcu_ericsson_test.c
trau_pcu_ericsson_trau_pcu_ericsson_test_LDADD = $(TRAU_LA_LIBS)

noinst_HEADERS = trau_conv/tw5reader.h

# boilerplate for the tests
# The `:;' works around a Bash 3.2 bug when the output is not writeable.
$(srcdir)/package.m4: $(top_srcdir)/configure.ac
	:;{ \
               echo '# Signature of the current package.' && \
               echo 'm4_define([AT_PACKAGE_NAME],' && \
               echo '  [$(PACKAGE_NAME)])' && \
               echo 'm4_define([AT_PACKAGE_TARNAME],' && \
               echo '  [$(PACKAGE_TARNAME)])' && \
               echo 'm4_define([AT_PACKAGE_VERSION],' && \
               echo '  [$(PACKAGE_VERSION)])' && \
               echo 'm4_define([AT_PACKAGE_STRING],' && \
               echo '  [$(PACKAGE_STRING)])' && \
               echo 'm4_define([AT_PACKAGE_BUGREPORT],' && \
               echo '  [$(PACKAGE_BUGREPORT)])'; \
               echo 'm4_define([AT_PACKAGE_URL],' && \
               echo '  [$(PACKAGE_URL)])'; \
             } >'$(srcdir)/package.m4'

EXTRA_DIST = testsuite.at $(srcdir)/package.m4 $(TESTSUITE) \
	subchan_demux/subchan_demux_test.ok \
	ipa_recv/ipa_recv_test.ok \
	raa_prime/d144-ul-input.asc raa_prime/nokia-tcsm2-atrau.hex \
	rtp_test/rtp_test.ok \
	tfo/enc_test_efr.in tfo/enc_test_fr.in tfo/enc_test_hr.in \
	tfo/extract_test_efr.ok tfo/extract_test_fr.ok tfo/extract_test_hr.ok \
	tfo/nokia_tcsm2/tfo-efr.hex tfo/nokia_tcsm2/tfo-fr.hex \
	tfo/nokia_tcsm2/tfo-hr.hex \
	trau_conv/efr_speech_basic.hex trau_conv/efr_speech_twts001_good.hex \
	trau_conv/efr_speech_twts001_mix.hex \
	trau_conv/fr_speech_basic.hex trau_conv/fr_speech_twts001_good.hex \
	trau_conv/fr_speech_twts001_mix.hex \
	trau_conv/hr_speech_rfc5993.hex trau_conv/hr_speech_ts101318.hex \
	trau_conv/hr_speech_twts002.hex trau_conv/hr_speech_invsid_bits.hex \
	trau_conv/rtp2trau_efr_dl1.ok trau_conv/rtp2trau_efr_dl2.ok \
	trau_conv/rtp2trau_efr_ul1.ok trau_conv/rtp2trau_efr_ul2.ok \
	trau_conv/rtp2trau_efr_ul3.ok \
	trau_conv/rtp2trau_fr_dl1.ok trau_conv/rtp2trau_fr_dl2.ok \
	trau_conv/rtp2trau_fr_ul1.ok trau_conv/rtp2trau_fr_ul2.ok \
	trau_conv/rtp2trau_fr_ul3.ok \
	trau_conv/rtp2trau_hr_dl.ok trau_conv/rtp2trau_hr_ul1.ok \
	trau_conv/rtp2trau_hr_ul2.ok trau_conv/rtp2trau_hr_ul3.ok \
	trau_conv/trau16_efr.in trau_conv/trau16_efr_std.ok \
	trau_conv/trau16_efr_twts001.ok \
	trau_conv/trau16_fr.in trau_conv/trau16_fr_std.ok \
	trau_conv/trau16_fr_twts001.ok \
	trau_conv/trau2rtp_hr.in trau_conv/trau2rtp_hr_std.ok \
	trau_conv/trau2rtp_hr_twts002.ok \
	trau_sync/trau_sync_test.ok trau_sync/trau_sync_test.err \
	trau_pcu_ericsson/trau_pcu_ericsson_test.ok

TESTSUITE = $(srcdir)/testsuite

DISTCLEANFILES = atconfig

check-local: atconfig $(TESTSUITE)
	$(SHELL) '$(TESTSUITE)' $(TESTSUITEFLAGS)

installcheck-local: atconfig $(TESTSUITE)
	$(SHELL) '$(TESTSUITE)' AUTOTEST_PATH='$(bindir)' \
		$(TESTSUITEFLAGS)

clean-local:
	test ! -f '$(TESTSUITE)' || \
		$(SHELL) '$(TESTSUITE)' --clean

AUTOM4TE = $(SHELL) $(top_srcdir)/missing --run autom4te
AUTOTEST = $(AUTOM4TE) --language=autotest
$(TESTSUITE): $(srcdir)/testsuite.at $(srcdir)/package.m4
	$(AUTOTEST) -I '$(srcdir)' -o $@.tmp $@.at
	mv $@.tmp $@

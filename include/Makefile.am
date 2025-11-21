noinst_HEADERS=mISDNif.h internal.h

osmocom/abis/version.h: osmocom/abis/version.h.tpl
	$(AM_V_GEN)$(MKDIR_P) $(dir $@)
	$(AM_V_GEN)sed \
		-e "s/{{VERSION}}/$$(echo '@VERSION@' | cut -d. -f1-3)/g" \
		-e "s/{{VERSION_MAJOR}}/$$(echo '@VERSION@' | cut -d. -f1)/g" \
		-e "s/{{VERSION_MINOR}}/$$(echo '@VERSION@' | cut -d. -f2)/g" \
		-e "s/{{VERSION_PATCH}}/$$(echo '@VERSION@' | cut -d. -f3)/g" \
		$< > $@

EXTRA_DIST = \
	osmocom/abis/version.h.tpl \
	$(NULL)

nobase_include_HEADERS = \
	osmocom/abis/abis.h \
	osmocom/abis/e1_input.h \
	osmocom/abis/ipa.h \
	osmocom/abis/ipaccess.h \
	osmocom/abis/lapd.h \
	osmocom/abis/lapd_pcap.h \
	osmocom/abis/subchan_demux.h \
	osmocom/abis/trau_frame.h \
	osmocom/abis/unixsocket_proto.h \
	osmocom/abis/version.h \
	osmocom/trau/amr_trau.h \
	osmocom/trau/clearmode.h \
	osmocom/trau/csd_ra2.h \
	osmocom/trau/csd_raa_prime.h \
	osmocom/trau/csd_v110.h \
	osmocom/trau/tfo_frame.h \
	osmocom/trau/trau_frame.h \
	osmocom/trau/trau_pcu_ericsson.h \
	osmocom/trau/trau_rtp.h \
	osmocom/trau/trau_sync.h \
	osmocom/trau/twts007.h \
	$(NULL)

if ENABLE_ORTP
nobase_include_HEADERS += osmocom/trau/osmo_ortp.h
endif

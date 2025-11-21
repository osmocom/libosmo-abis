/*
 * Interworking of AMR codec frames between TS 48.060 & 48.061 TRAU frame
 * formats, RTP in RFC 4867 and TW-TS-006 formats, and speech encoder/decoder
 * engines.
 *
 * This code was contributed to Osmocom Cellular Network Infrastructure
 * project by Mother Mychaela N. Falconia of Themyscira Wireless.
 * Mother Mychaela's contributions are NOT subject to copyright:
 * no rights reserved, all rights relinquished.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <osmocom/core/bits.h>
#include <osmocom/codec/codec.h>
#include <osmocom/trau/trau_frame.h>

/*! \defgroup amr_trau AMR TRAU frame interworking facility
 *  @{
 *
 * The present facility provides interworking of AMR codec frames between
 * the following 3 possible external interfaces:
 *
 * 1) AMR TRAU frames as they appear on T1/E1 Abis per TS 48.060 & 48.061,
 *    or AMR TFO frames as they appear inside G.711 PCM sample stream
 *    per TS 28.062.
 *
 * 2) Representation of AMR codec in RTP, in either of the two formats
 *    (BWE or OA) defined in RFC 4867 or in the OAX (OA extended) format
 *    of TW-TS-006.
 *
 * 3) AMR speech encoder and decoder engines that operate on arrays of
 *    codec parameters in uint16_t or int16_t format, such as Themyscira
 *    libtwamr.
 *
 * Encoding and decoding functions for interfaces 1 and 2 are provided
 * as part of the present library component.  Implementation of
 * interface 3 requires applying lower-level libosmocodec functions
 * osmo_amr_param_to_sbits() and osmo_amr_sbits_to_param() directly
 * to s_bits[] member in struct osmo_amrt_if; higher-level functions
 * that interface struct osmo_amrt_if to struct amr_param_frame (libtwamr)
 * can also be implemented, but they cannot be included in libosmo*
 * because compile-time dependency on libtwamr is not allowed.
 *
 * All API functions, struct and enum definitions that are part of this
 * facility are prefixed with osmo_amrt_ for namespace reasons.
 */

/*! Every AMR frame described in a struct osmo_amrt_if has a classification
 *  as in speech, SID etc.  This classification is captured in the following
 *  enum, and is based on Frame_Classification and No_Speech_Classification
 *  definitions in TS 48.060.  These TS 48.060 definitions have been chosen
 *  as the basis for the common intermediate representation, rather than
 *  the frame type system of RFC 4867, because the latter is crippled,
 *  removing Speech_Degraded and Onset classifications, and furthermore
 *  the distinctions between good and bad speech, good and bad SID and
 *  Sid_First vs Sid_Update are coded in a very indirect way in RTP.
 *  The present TRAU-based frame classification system maps directly to
 *  speech encoder and decoder frame types in libtwamr (based on 3GPP
 *  reference implementation), hence the entire burden of converting
 *  between the two classification systems is moved into RTP interface
 *  functions.
 *
 *  For our enum osmo_amrt_fc we have chosen 5-bit code points that map
 *  directly to TS 48.060: bits [4:3] are Frame_Classification per section
 *  5.5.1.2.1, bits [2:0] are No_Speech_Classification per section 5.5.1.2.2.
 */
enum osmo_amrt_fc {
	OSMO_AMRT_FC_NO_DATA		= 0x00,
	OSMO_AMRT_FC_SID_BAD		= 0x04,
	OSMO_AMRT_FC_SID_UPDATE		= 0x05,
	OSMO_AMRT_FC_ONSET		= 0x06,
	OSMO_AMRT_FC_SID_FIRST		= 0x07,
	OSMO_AMRT_FC_SPEECH_BAD		= 0x08,
	OSMO_AMRT_FC_SPEECH_DEGRADED	= 0x10,
	OSMO_AMRT_FC_SPEECH_GOOD	= 0x18,
};

static inline bool osmo_amrt_fc_is_speech(enum osmo_amrt_fc fc)
{
	return (fc & 0x18) != 0;
}

/*! This enum captures the 3-bit Config_Prot field of TS 28.062 section
 *  C.6.1.2.  This field is used to embed TFO configuration messages
 *  in AMR TRAU or TFO frames when 3GPP Rel4 method is in use, and it
 *  needs to be ferried between TRAU/TFO frames and TW-TS-006.
 */
enum osmo_amrt_tfo_config_prot {
	OSMO_AMRT_CFG_PROT_NO_CON	= 0x00,
	OSMO_AMRT_CFG_PROT_CON_REQ	= 0x01,
	OSMO_AMRT_CFG_PROT_DIS_REQ	= 0x02,
	OSMO_AMRT_CFG_PROT_CON_ACK	= 0x03,
	OSMO_AMRT_CFG_PROT_UL_ACK	= 0x05,
	OSMO_AMRT_CFG_PROT_DL_ACK	= 0x06,
};

/*! TW-TS-006 section 5.7 adds a new Param_Info field, used to describe
 *  presence or absence of TFO configuration parameters beyond Config_Prot
 *  and Message_No fields.  This enum captures Param_Info definitions
 *  from TW-TS-006.  Unshifted values are used, exactly as they appear
 *  in the Config_Prot extension octet.
 */
enum osmo_amrt_tfo_param_info {
	OSMO_AMRT_TFOP_ABSENT_NATIVE	= 0x00,
	OSMO_AMRT_TFOP_ABSENT_NO_ROOM	= 0x02,
	OSMO_AMRT_TFOP_ABSENT_BAD_CRC	= 0x04,
	OSMO_AMRT_TFOP_PRESENT_VALID	= 0x06,
};

/*! Every AMR frame that passes through the present interworking facility
 *  passes through a common intermediate format on its way between the
 *  externally received input format and the ultimate destination format.
 *  The following struct carries this intermediate format.
 *
 *  Note regarding validity: because this intermediate structure is
 *  strictly internal to applications performing interworking conversions,
 *  library functions that take this struct as input are allowed to
 *  expect all stated validity rules to be observed, and are allowed to
 *  OSMO_ASSERT on invalid inputs.
 */
struct osmo_amrt_if {
	/* Fields in this structure are grouped in logical order, but also
	 * with an eye toward structure packing optimization: in each logical
	 * section enum fields appear first, followed by fields of type
	 * uint8_t, ubit_t or bool.
	 */

	/*! Fundamental class of this frame */
	enum osmo_amrt_fc frame_class;
	/*! Codec Mode Indication: if this field is valid as marked by
	 *  cmi_valid being true, only modes 0 through 7 are valid here,
	 *  _not_ AMR_SID or AMR_NO_DATA! */
	enum osmo_amr_type cmi;
	/*! Codec Mode Request: modes [0,7] and AMR_NO_DATA are potentially
	 *  valid here; the latter can occur if the input was RTP. */
	enum osmo_amr_type cmr;
	/*! Is CMI field valid?  If frame_class is OSMO_AMRT_FC_NO_DATA,
	 *  then CMI may or may not be valid; all other frame classes
	 *  require valid CMI. */
	bool cmi_valid;
	/*! Is CMR field valid?  CMR validity is independent of frame class
	 *  and other factors. */
	bool cmr_valid;

	/*! Frame payload bits in s-bit order as used in TRAU and TFO frames,
	 *  also most convenient for conversion from/to speech codec parameter
	 *  arrays, _not_ the d-bit order used on Um interface and in RTP.
	 *  In the case of SID frames, only 35 bits are stored in this array
	 *  as in TRAU frames, not 39 bits as in RTP.  In the case of Sid_First,
	 *  we extract 35 s-bits from whichever source format we are receiving
	 *  (TRAU or RTP), but we don't use this array when we emit Sid_First
	 *  in output encoding direction: instead we emit all-1s in TRAU frames
	 *  as required by TS 48.060 & 48.061 and all-0s in RTP as required by
	 *  TS 26.101, which forms the basis for RFC 4867.  Exception: when we
	 *  emit RTP output in TW-TS-006 format, we emit whatever Sid_First
	 *  bits we received from TRAU frame source, be they all-1s or not.
	 */
	ubit_t s_bits[244];

	/*! Request or Indication Flag as defined in TS 48.060 section
	 *  5.5.1.2.1, present in TRAU frames and in TW-TS-006 RTP format. */
	ubit_t rif;
	/*! Is RIF field valid? */
	bool rif_valid;

	/*! The 3 fields that follow after this validity flag (TA, DTXd
	 *  and TFOE) are valid only if this validity flag is true. */
	bool ta_dtxd_tfoe_valid;
	/*! Time Alignment field in TRAU frames, also used to convey TFO
	 *  and handover notifications. */
	uint8_t ta;
	/*! DTXd: bit C19 in TRAU-AMR-16k or D63 in TRAU-AMR-8k-No_Speech */
	ubit_t dtxd;
	/*! TFOE: bit C20 in TRAU-AMR-16k or D64 in TRAU-AMR-8k-No_Speech */
	ubit_t tfoe;

	/*! All following TFO-related fields are valid only if this Config_Prot
	 *  field is non-zero. */
	enum osmo_amrt_tfo_config_prot config_prot;
	/*! Param_Info field is defined in TW-TS-006 section 5.7,
	 *  see enum definition above. */
	enum osmo_amrt_tfo_param_info param_info;
	/*! Message_No field is defined in TS 28.062 section C.6.1.3 */
	uint8_t message_no;
	/*! Payload bits of TFO parameters as specified in TW-TS-006
	 *  section 5.8. */
	ubit_t tfo_param_bits[71];
};

/*! Extraction of information content from a received stream of AMR TRAU
 *  frames is a stateful procedure, requiring state to be maintained
 *  from one decoded TRAU frame to the next.  This structure captures
 *  the necessary state, as detailed in TW-TS-006 section 6.1.1.
 *
 *  On initialization, set cmi_valid and cmr_valid to false, or zero out
 *  the whole struct.
 */
struct osmo_amrt_decode_state {
	/*! Codec Mode Indication: if this field is valid as marked by
	 *  cmi_valid being true, only modes 0 through 7 are valid here,
	 *  _not_ AMR_SID or AMR_NO_DATA! */
	enum osmo_amr_type cmi;
	/*! Codec Mode Request: same range restriction as for CMI. */
	enum osmo_amr_type cmr;
	/*! Is CMI field valid? */
	bool cmi_valid;
	/*! Is CMR field valid? */
	bool cmr_valid;
};

/*! The operation of decoding a received AMR TRAU frame can have three
 *  outcomes, in addition to filled struct osmo_amrt_if output.
 *  This enum defines the 3 possible result codes.
 */
enum osmo_amrt_decode_result {
	/*! The frame that was received is found to be perfectly valid
	 *  and properly formed - the normal case.
	 */
	OSMO_AMRT_FRAME_DEC_GOOD,
	/*! The frame that was received is bad, not in the sense of Rx radio
	 *  errors or FACCH stealing etc, but in the sense of errors in
	 *  frame coding or CRC etc.  If this condition occurs when a TRAU
	 *  or TRAU-like MGW decodes a TRAU-UL frame, UFE needs to be
	 *  indicated back to the BTS/CCU.  Note that the frame may have been
	 *  decoded (struct osmo_amrt_if filled with something other than
	 *  OSMO_AMRT_FC_NO_DATA w/o CMI) if the bad CRC was CRC[2..4]
	 *  for speech, rather than CRC1 that covers control bits.
	 */
	OSMO_AMRT_FRAME_DEC_BAD,
	/*! We decoded the frame just fine, but the far end indicated UFE/DFE
	 *  via TRAU-AMR-16k bit C13 or TRAU-AMR-8k-No_Speech bit D7.  DFE
	 *  condition in TRAU-AMR-8k mode is important for TRAU or TRAU-like
	 *  MGW implementations.
	 */
	OSMO_AMRT_FRAME_DEC_REMOTE_BAD,
};

enum osmo_amrt_decode_result
osmo_amrt_decode_trau_frame(struct osmo_amrt_decode_state *state,
			    struct osmo_amrt_if *fr,
			    const struct osmo_trau_frame *tf,
			    bool allow_config_prot);

/*! Generation of AMR TRAU frames from a stream of struct osmo_amrt_if inputs
 *  (ultimately derived from RTP input in most cases) is likewise a stateful
 *  procedure, requiring state to be maintained from one encoded TRAU frame
 *  to the next.  This structure captures the necessary state, as detailed in
 *  TW-TS-006 section 6.3.2.
 *
 *  Initialization: there is no "one size fits all" answer, instead application
 *  implementors have to understand the problem and decide as appropriate for
 *  each use case.  See TW-TS-006 sections 6.3.3.1 and 6.3.6.
 */
struct osmo_amrt_encode_state {
	/*! Codec Mode Indication: valid range is [0,7] for TRAU-AMR-16k
	 *  or [0,4] for TRAU-AMR-8k.  In any case, no AMR_SID or AMR_NO_DATA
	 *  can appear in this field. */
	enum osmo_amr_type cmi;
	/*! Codec Mode Request: same range restriction as for CMI. */
	enum osmo_amr_type cmr;
	/*! Request or Indication Flag.  In between frames, this state
	 *  variable remembers RIF of the last emitted frame. */
	ubit_t rif;
	/*! What should we emit in DTXd bit? */
	ubit_t dtxd;
	/*! What should we emit in TFOE bit? */
	ubit_t tfoe;
};

/*! In TRAU frame encoding direction, we split off CRC generation into a
 *  separate post-processing function, to be called after main AMR TRAU frame
 *  encoding functions.  This split is needed so that applications can modify
 *  some control bits after encoding: set C5 for TFO, set C6..C11 for timing
 *  alignment, set UFE etc.  However, the exact manner of AMR CRC computation
 *  depends on speech mode or No_Speech, which cannot always be derived from
 *  encoded struct osmo_trau_frame alone - consider speech frames with RIF=1.
 *  Therefore, we pass an additional CRC type code from our main AMR TRAU
 *  frame encoding functions to the CRC-setter post-processing function.
 */
enum osmo_amrt_crc_type {
	OSMO_AMRT_CRC_NONE,
	OSMO_AMRT_CRC_16k_NO_SPEECH,
	OSMO_AMRT_CRC_16k_SPEECH_MR475,
	OSMO_AMRT_CRC_16k_SPEECH_MR515,
	OSMO_AMRT_CRC_16k_SPEECH_MR59,
	OSMO_AMRT_CRC_16k_SPEECH_MR67,
	OSMO_AMRT_CRC_16k_SPEECH_MR74,
	OSMO_AMRT_CRC_16k_SPEECH_MR795,
	OSMO_AMRT_CRC_16k_SPEECH_MR102,
	OSMO_AMRT_CRC_16k_SPEECH_MR122,
	OSMO_AMRT_CRC_8k_SID_UPDATE,
	OSMO_AMRT_CRC_8k_SPEECH_MR475,
	OSMO_AMRT_CRC_8k_SPEECH_MR515,
	OSMO_AMRT_CRC_8k_SPEECH_MR59,
	OSMO_AMRT_CRC_8k_SPEECH_MR67,
	OSMO_AMRT_CRC_8k_SPEECH_MR74,
};

enum osmo_amrt_crc_type
osmo_amrt_encode_trau_frame_16k(struct osmo_amrt_encode_state *state,
				struct osmo_trau_frame *tf,
				const struct osmo_amrt_if *fr, bool is_tfo);

/*! In TRAU-AMR-8k output direction, it may be necessary to restrict
 *  the type of frames that may be emitted: for example, a TRAU-like MGW
 *  may need to set UFE or obey DFE from the BTS/CCU.  The following
 *  enum controls these restrictions.
 */
enum osmo_amrt_8k_restrict {
	/*! No restriction, any TRAU-AMR-8k output is allowed */
	OSMO_AMRT_OUT8k_NO_RESTRICT,
	/*! Restrict to either No_Speech or lower modes */
	OSMO_AMRT_OUT8k_REQUIRE_LOW,
	/*! No_Speech output is required */
	OSMO_AMRT_OUT8k_REQUIRE_NO_SPEECH,
};

enum osmo_amrt_crc_type
osmo_amrt_encode_trau_frame_8k(struct osmo_amrt_encode_state *state,
				struct osmo_trau_frame *tf,
				const struct osmo_amrt_if *fr, bool is_tfo,
				enum osmo_amrt_8k_restrict restr);

int osmo_amrt_set_trau_crc(struct osmo_trau_frame *tf,
			   enum osmo_amrt_crc_type crc_type);

/*! On RTP side we support 3 different formats: RFC 4867 BWE, RFC 4867 OA
 *  and TW-TS-006 OAX (OA extended).  This enum captures the necessary
 *  format selector.
 */
enum osmo_amrt_rtp_format {
	OSMO_AMRT_RTP_FMT_BWE,
	OSMO_AMRT_RTP_FMT_OA,
	OSMO_AMRT_RTP_FMT_OAX,
};

int osmo_amrt_decode_rtp(struct osmo_amrt_if *fr, const uint8_t *rtp_pl,
			 unsigned rtp_pl_len, enum osmo_amrt_rtp_format fmt);
int osmo_amrt_encode_rtp(uint8_t *pl_buf, unsigned pl_buf_size,
			 const struct osmo_amrt_if *fr,
			 enum osmo_amrt_rtp_format fmt);

/*! @} */

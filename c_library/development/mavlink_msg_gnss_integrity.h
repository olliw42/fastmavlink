//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GNSS_INTEGRITY_H
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_H


//----------------------------------------
//-- Message GNSS_INTEGRITY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gnss_integrity_t {
    uint32_t system_errors;
    uint16_t raim_hfom;
    uint16_t raim_vfom;
    uint8_t id;
    uint8_t authentication_state;
    uint8_t jamming_state;
    uint8_t spoofing_state;
    uint8_t raim_state;
    uint8_t corrections_quality;
    uint8_t system_status_summary;
    uint8_t gnss_signal_quality;
    uint8_t post_processing_quality;
}) fmav_gnss_integrity_t;


#define FASTMAVLINK_MSG_ID_GNSS_INTEGRITY  441

#define FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_CRCEXTRA  169

#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FLAGS  0
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FRAME_LEN_MAX  42



#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_SYSTEM_ERRORS_OFS  0
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_RAIM_HFOM_OFS  4
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_RAIM_VFOM_OFS  6
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_ID_OFS  8
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_AUTHENTICATION_STATE_OFS  9
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_JAMMING_STATE_OFS  10
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_SPOOFING_STATE_OFS  11
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_RAIM_STATE_OFS  12
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_CORRECTIONS_QUALITY_OFS  13
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_SYSTEM_STATUS_SUMMARY_OFS  14
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_GNSS_SIGNAL_QUALITY_OFS  15
#define FASTMAVLINK_MSG_GNSS_INTEGRITY_FIELD_POST_PROCESSING_QUALITY_OFS  16


//----------------------------------------
//-- Message GNSS_INTEGRITY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality,
    fmav_status_t* _status)
{
    fmav_gnss_integrity_t* _payload = (fmav_gnss_integrity_t*)_msg->payload;

    _payload->system_errors = system_errors;
    _payload->raim_hfom = raim_hfom;
    _payload->raim_vfom = raim_vfom;
    _payload->id = id;
    _payload->authentication_state = authentication_state;
    _payload->jamming_state = jamming_state;
    _payload->spoofing_state = spoofing_state;
    _payload->raim_state = raim_state;
    _payload->corrections_quality = corrections_quality;
    _payload->system_status_summary = system_status_summary;
    _payload->gnss_signal_quality = gnss_signal_quality;
    _payload->post_processing_quality = post_processing_quality;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GNSS_INTEGRITY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_GNSS_INTEGRITY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gnss_integrity_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gnss_integrity_pack(
        _msg, sysid, compid,
        _payload->id, _payload->system_errors, _payload->authentication_state, _payload->jamming_state, _payload->spoofing_state, _payload->raim_state, _payload->raim_hfom, _payload->raim_vfom, _payload->corrections_quality, _payload->system_status_summary, _payload->gnss_signal_quality, _payload->post_processing_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality,
    fmav_status_t* _status)
{
    fmav_gnss_integrity_t* _payload = (fmav_gnss_integrity_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->system_errors = system_errors;
    _payload->raim_hfom = raim_hfom;
    _payload->raim_vfom = raim_vfom;
    _payload->id = id;
    _payload->authentication_state = authentication_state;
    _payload->jamming_state = jamming_state;
    _payload->spoofing_state = spoofing_state;
    _payload->raim_state = raim_state;
    _payload->corrections_quality = corrections_quality;
    _payload->system_status_summary = system_status_summary;
    _payload->gnss_signal_quality = gnss_signal_quality;
    _payload->post_processing_quality = post_processing_quality;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GNSS_INTEGRITY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GNSS_INTEGRITY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GNSS_INTEGRITY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gnss_integrity_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gnss_integrity_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->system_errors, _payload->authentication_state, _payload->jamming_state, _payload->spoofing_state, _payload->raim_state, _payload->raim_hfom, _payload->raim_vfom, _payload->corrections_quality, _payload->system_status_summary, _payload->gnss_signal_quality, _payload->post_processing_quality,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality,
    fmav_status_t* _status)
{
    fmav_gnss_integrity_t _payload;

    _payload.system_errors = system_errors;
    _payload.raim_hfom = raim_hfom;
    _payload.raim_vfom = raim_vfom;
    _payload.id = id;
    _payload.authentication_state = authentication_state;
    _payload.jamming_state = jamming_state;
    _payload.spoofing_state = spoofing_state;
    _payload.raim_state = raim_state;
    _payload.corrections_quality = corrections_quality;
    _payload.system_status_summary = system_status_summary;
    _payload.gnss_signal_quality = gnss_signal_quality;
    _payload.post_processing_quality = post_processing_quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GNSS_INTEGRITY,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gnss_integrity_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GNSS_INTEGRITY,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GNSS_INTEGRITY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GNSS_INTEGRITY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gnss_integrity_decode(fmav_gnss_integrity_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GNSS_INTEGRITY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gnss_integrity_get_field_system_errors(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_get_field_raim_hfom(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gnss_integrity_get_field_raim_vfom(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_authentication_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_jamming_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_spoofing_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_raim_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_corrections_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_system_status_summary(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_gnss_signal_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gnss_integrity_get_field_post_processing_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GNSS_INTEGRITY  441

#define mavlink_gnss_integrity_t  fmav_gnss_integrity_t

#define MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN  17
#define MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN  17
#define MAVLINK_MSG_ID_441_LEN  17
#define MAVLINK_MSG_ID_441_MIN_LEN  17

#define MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC  169
#define MAVLINK_MSG_ID_441_CRC  169




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gnss_integrity_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gnss_integrity_pack(
        _msg, sysid, compid,
        id, system_errors, authentication_state, jamming_state, spoofing_state, raim_state, raim_hfom, raim_vfom, corrections_quality, system_status_summary, gnss_signal_quality, post_processing_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gnss_integrity_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gnss_integrity_t* _payload)
{
    return mavlink_msg_gnss_integrity_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->system_errors, _payload->authentication_state, _payload->jamming_state, _payload->spoofing_state, _payload->raim_state, _payload->raim_hfom, _payload->raim_vfom, _payload->corrections_quality, _payload->system_status_summary, _payload->gnss_signal_quality, _payload->post_processing_quality);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gnss_integrity_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
    return fmav_msg_gnss_integrity_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, system_errors, authentication_state, jamming_state, spoofing_state, raim_state, raim_hfom, raim_vfom, corrections_quality, system_status_summary, gnss_signal_quality, post_processing_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gnss_integrity_decode(const mavlink_message_t* msg, mavlink_gnss_integrity_t* payload)
{
    fmav_msg_gnss_integrity_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GNSS_INTEGRITY_H

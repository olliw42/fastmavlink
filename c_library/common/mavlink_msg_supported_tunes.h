//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SUPPORTED_TUNES_H
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_H


//----------------------------------------
//-- Message SUPPORTED_TUNES
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_supported_tunes_t {
    uint32_t format;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_supported_tunes_t;


#define FASTMAVLINK_MSG_ID_SUPPORTED_TUNES  401


#define FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MIN  6
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN  6
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_CRCEXTRA  183

#define FASTMAVLINK_MSG_ID_401_LEN_MIN  6
#define FASTMAVLINK_MSG_ID_401_LEN_MAX  6
#define FASTMAVLINK_MSG_ID_401_LEN  6
#define FASTMAVLINK_MSG_ID_401_CRCEXTRA  183



#define FASTMAVLINK_MSG_SUPPORTED_TUNES_FLAGS  3
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_SUPPORTED_TUNES_TARGET_COMPONENT_OFS  5


//----------------------------------------
//-- Message SUPPORTED_TUNES packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_supported_tunes_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format,
    fmav_status_t* _status)
{
    fmav_supported_tunes_t* _payload = (fmav_supported_tunes_t*)msg->payload;

    _payload->format = format;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SUPPORTED_TUNES;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SUPPORTED_TUNES_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_supported_tunes_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_supported_tunes_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_supported_tunes_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->format,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_supported_tunes_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format,
    fmav_status_t* _status)
{
    fmav_supported_tunes_t* _payload = (fmav_supported_tunes_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->format = format;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SUPPORTED_TUNES;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SUPPORTED_TUNES >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SUPPORTED_TUNES >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SUPPORTED_TUNES_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_supported_tunes_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_supported_tunes_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_supported_tunes_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->format,
        _status);
}


//----------------------------------------
//-- Message SUPPORTED_TUNES unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_supported_tunes_decode(fmav_supported_tunes_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SUPPORTED_TUNES_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SUPPORTED_TUNES  401

#define mavlink_supported_tunes_t  fmav_supported_tunes_t

#define MAVLINK_MSG_ID_SUPPORTED_TUNES_LEN  6
#define MAVLINK_MSG_ID_SUPPORTED_TUNES_MIN_LEN  6
#define MAVLINK_MSG_ID_401_LEN  6
#define MAVLINK_MSG_ID_401_MIN_LEN  6

#define MAVLINK_MSG_ID_SUPPORTED_TUNES_CRC  183
#define MAVLINK_MSG_ID_401_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_supported_tunes_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t format)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_supported_tunes_pack(
        msg, sysid, compid,
        target_system, target_component, format,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_supported_tunes_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format)
{
    return fmav_msg_supported_tunes_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, format,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_supported_tunes_decode(const mavlink_message_t* msg, mavlink_supported_tunes_t* payload)
{
    fmav_msg_supported_tunes_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SUPPORTED_TUNES_H

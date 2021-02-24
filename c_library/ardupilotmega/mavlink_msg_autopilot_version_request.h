//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_H
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_H


//----------------------------------------
//-- Message AUTOPILOT_VERSION_REQUEST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_version_request_t {
    uint8_t target_system;
    uint8_t target_component;
}) fmav_autopilot_version_request_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST  183


#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_CRCEXTRA  85

#define FASTMAVLINK_MSG_ID_183_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_183_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_183_LEN  2
#define FASTMAVLINK_MSG_ID_183_CRCEXTRA  85



#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_FLAGS  3
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message AUTOPILOT_VERSION_REQUEST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_request_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_autopilot_version_request_t* _payload = (fmav_autopilot_version_request_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_request_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_request_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_request_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_autopilot_version_request_t* _payload = (fmav_autopilot_version_request_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_request_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_request_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


//----------------------------------------
//-- Message AUTOPILOT_VERSION_REQUEST unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_version_request_decode(fmav_autopilot_version_request_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST  183

#define mavlink_autopilot_version_request_t  fmav_autopilot_version_request_t

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_LEN  2
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN  2
#define MAVLINK_MSG_ID_183_LEN  2
#define MAVLINK_MSG_ID_183_MIN_LEN  2

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_CRC  85
#define MAVLINK_MSG_ID_183_CRC  85




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_version_request_pack(
        msg, sysid, compid,
        target_system, target_component,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_request_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_autopilot_version_request_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_version_request_decode(const mavlink_message_t* msg, mavlink_autopilot_version_request_t* payload)
{
    fmav_msg_autopilot_version_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_VERSION_REQUEST_H

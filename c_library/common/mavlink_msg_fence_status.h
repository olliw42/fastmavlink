//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FENCE_STATUS_H
#define FASTMAVLINK_MSG_FENCE_STATUS_H


//----------------------------------------
//-- Message FENCE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_fence_status_t {
    uint32_t breach_time;
    uint16_t breach_count;
    uint8_t breach_status;
    uint8_t breach_type;
    uint8_t breach_mitigation;
}) fmav_fence_status_t;


#define FASTMAVLINK_MSG_ID_FENCE_STATUS  162


#define FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MIN  8
#define FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN  9
#define FASTMAVLINK_MSG_FENCE_STATUS_CRCEXTRA  189

#define FASTMAVLINK_MSG_ID_162_LEN_MIN  8
#define FASTMAVLINK_MSG_ID_162_LEN_MAX  9
#define FASTMAVLINK_MSG_ID_162_LEN  9
#define FASTMAVLINK_MSG_ID_162_CRCEXTRA  189



#define FASTMAVLINK_MSG_FENCE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_FENCE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FENCE_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message FENCE_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation,
    fmav_status_t* _status)
{
    fmav_fence_status_t* _payload = (fmav_fence_status_t*)msg->payload;

    _payload->breach_time = breach_time;
    _payload->breach_count = breach_count;
    _payload->breach_status = breach_status;
    _payload->breach_type = breach_type;
    _payload->breach_mitigation = breach_mitigation;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_FENCE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_FENCE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fence_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fence_status_pack(
        msg, sysid, compid,
        _payload->breach_status, _payload->breach_count, _payload->breach_type, _payload->breach_time, _payload->breach_mitigation,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation,
    fmav_status_t* _status)
{
    fmav_fence_status_t* _payload = (fmav_fence_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->breach_time = breach_time;
    _payload->breach_count = breach_count;
    _payload->breach_status = breach_status;
    _payload->breach_type = breach_type;
    _payload->breach_mitigation = breach_mitigation;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FENCE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FENCE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FENCE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FENCE_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fence_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fence_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->breach_status, _payload->breach_count, _payload->breach_type, _payload->breach_time, _payload->breach_mitigation,
        _status);
}


//----------------------------------------
//-- Message FENCE_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_fence_status_decode(fmav_fence_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_FENCE_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FENCE_STATUS  162

#define mavlink_fence_status_t  fmav_fence_status_t

#define MAVLINK_MSG_ID_FENCE_STATUS_LEN  9
#define MAVLINK_MSG_ID_FENCE_STATUS_MIN_LEN  8
#define MAVLINK_MSG_ID_162_LEN  9
#define MAVLINK_MSG_ID_162_MIN_LEN  8

#define MAVLINK_MSG_ID_FENCE_STATUS_CRC  189
#define MAVLINK_MSG_ID_162_CRC  189




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fence_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_fence_status_pack(
        msg, sysid, compid,
        breach_status, breach_count, breach_type, breach_time, breach_mitigation,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fence_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t breach_status, uint16_t breach_count, uint8_t breach_type, uint32_t breach_time, uint8_t breach_mitigation)
{
    return fmav_msg_fence_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        breach_status, breach_count, breach_type, breach_time, breach_mitigation,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_fence_status_decode(const mavlink_message_t* msg, mavlink_fence_status_t* payload)
{
    fmav_msg_fence_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FENCE_STATUS_H
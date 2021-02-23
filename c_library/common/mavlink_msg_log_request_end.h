//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_REQUEST_END_H
#define FASTMAVLINK_MSG_LOG_REQUEST_END_H


//----------------------------------------
//-- Message LOG_REQUEST_END
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_request_end_t {
    uint8_t target_system;
    uint8_t target_component;
}) fmav_log_request_end_t;


#define FASTMAVLINK_MSG_ID_LOG_REQUEST_END  122


#define FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_LOG_REQUEST_END_CRCEXTRA  203

#define FASTMAVLINK_MSG_ID_122_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_122_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_122_LEN  2
#define FASTMAVLINK_MSG_ID_122_CRCEXTRA  203



#define FASTMAVLINK_MSG_LOG_REQUEST_END_FLAGS  3
#define FASTMAVLINK_MSG_LOG_REQUEST_END_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_REQUEST_END_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message LOG_REQUEST_END packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_end_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_log_request_end_t* _payload = (fmav_log_request_end_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOG_REQUEST_END;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_END_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_end_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_end_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_end_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_end_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_log_request_end_t* _payload = (fmav_log_request_end_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_END;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_END >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_END >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_REQUEST_END_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_end_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_end_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_end_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


//----------------------------------------
//-- Message LOG_REQUEST_END unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_request_end_decode(fmav_log_request_end_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOG_REQUEST_END_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_REQUEST_END  122

#define mavlink_log_request_end_t  fmav_log_request_end_t

#define MAVLINK_MSG_ID_LOG_REQUEST_END_LEN  2
#define MAVLINK_MSG_ID_LOG_REQUEST_END_MIN_LEN  2
#define MAVLINK_MSG_ID_122_LEN  2
#define MAVLINK_MSG_ID_122_MIN_LEN  2

#define MAVLINK_MSG_ID_LOG_REQUEST_END_CRC  203
#define MAVLINK_MSG_ID_122_CRC  203




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_end_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_request_end_pack(
        msg, sysid, compid,
        target_system, target_component,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_end_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_log_request_end_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_request_end_decode(const mavlink_message_t* msg, mavlink_log_request_end_t* payload)
{
    fmav_msg_log_request_end_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_REQUEST_END_H

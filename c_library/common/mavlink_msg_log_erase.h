//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_ERASE_H
#define FASTMAVLINK_MSG_LOG_ERASE_H


//----------------------------------------
//-- Message LOG_ERASE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_erase_t {
    uint8_t target_system;
    uint8_t target_component;
}) fmav_log_erase_t;


#define FASTMAVLINK_MSG_ID_LOG_ERASE  121


#define FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_LOG_ERASE_CRCEXTRA  237

#define FASTMAVLINK_MSG_ID_121_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_121_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_121_LEN  2
#define FASTMAVLINK_MSG_ID_121_CRCEXTRA  237



#define FASTMAVLINK_MSG_LOG_ERASE_FLAGS  3
#define FASTMAVLINK_MSG_LOG_ERASE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_ERASE_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message LOG_ERASE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_erase_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_log_erase_t* _payload = (fmav_log_erase_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOG_ERASE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_LOG_ERASE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_erase_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_erase_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_erase_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_erase_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_log_erase_t* _payload = (fmav_log_erase_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_ERASE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ERASE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ERASE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_ERASE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_erase_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_erase_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_erase_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


//----------------------------------------
//-- Message LOG_ERASE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_erase_decode(fmav_log_erase_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOG_ERASE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_ERASE  121

#define mavlink_log_erase_t  fmav_log_erase_t

#define MAVLINK_MSG_ID_LOG_ERASE_LEN  2
#define MAVLINK_MSG_ID_LOG_ERASE_MIN_LEN  2
#define MAVLINK_MSG_ID_121_LEN  2
#define MAVLINK_MSG_ID_121_MIN_LEN  2

#define MAVLINK_MSG_ID_LOG_ERASE_CRC  237
#define MAVLINK_MSG_ID_121_CRC  237




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_erase_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_erase_pack(
        msg, sysid, compid,
        target_system, target_component,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_erase_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_log_erase_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_erase_decode(const mavlink_message_t* msg, mavlink_log_erase_t* payload)
{
    fmav_msg_log_erase_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_ERASE_H

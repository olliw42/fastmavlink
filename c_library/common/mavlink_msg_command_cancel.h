//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMMAND_CANCEL_H
#define FASTMAVLINK_MSG_COMMAND_CANCEL_H


//----------------------------------------
//-- Message COMMAND_CANCEL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_command_cancel_t {
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_command_cancel_t;


#define FASTMAVLINK_MSG_ID_COMMAND_CANCEL  80


#define FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MIN  4
#define FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN  4
#define FASTMAVLINK_MSG_COMMAND_CANCEL_CRCEXTRA  14

#define FASTMAVLINK_MSG_ID_80_LEN_MIN  4
#define FASTMAVLINK_MSG_ID_80_LEN_MAX  4
#define FASTMAVLINK_MSG_ID_80_LEN  4
#define FASTMAVLINK_MSG_ID_80_CRCEXTRA  14



#define FASTMAVLINK_MSG_COMMAND_CANCEL_FLAGS  3
#define FASTMAVLINK_MSG_COMMAND_CANCEL_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_COMMAND_CANCEL_TARGET_COMPONENT_OFS  3


//----------------------------------------
//-- Message COMMAND_CANCEL packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_cancel_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command,
    fmav_status_t* _status)
{
    fmav_command_cancel_t* _payload = (fmav_command_cancel_t*)msg->payload;

    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COMMAND_CANCEL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_COMMAND_CANCEL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_cancel_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_cancel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_cancel_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->command,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_cancel_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command,
    fmav_status_t* _status)
{
    fmav_command_cancel_t* _payload = (fmav_command_cancel_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMMAND_CANCEL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_CANCEL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_CANCEL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_CANCEL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_cancel_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_cancel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_cancel_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->command,
        _status);
}


//----------------------------------------
//-- Message COMMAND_CANCEL unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_command_cancel_decode(fmav_command_cancel_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_COMMAND_CANCEL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMMAND_CANCEL  80

#define mavlink_command_cancel_t  fmav_command_cancel_t

#define MAVLINK_MSG_ID_COMMAND_CANCEL_LEN  4
#define MAVLINK_MSG_ID_COMMAND_CANCEL_MIN_LEN  4
#define MAVLINK_MSG_ID_80_LEN  4
#define MAVLINK_MSG_ID_80_MIN_LEN  4

#define MAVLINK_MSG_ID_COMMAND_CANCEL_CRC  14
#define MAVLINK_MSG_ID_80_CRC  14




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_cancel_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t command)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_command_cancel_pack(
        msg, sysid, compid,
        target_system, target_component, command,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_cancel_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command)
{
    return fmav_msg_command_cancel_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, command,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_command_cancel_decode(const mavlink_message_t* msg, mavlink_command_cancel_t* payload)
{
    fmav_msg_command_cancel_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMMAND_CANCEL_H

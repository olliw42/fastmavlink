//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMMAND_LONG_H
#define FASTMAVLINK_MSG_COMMAND_LONG_H


//----------------------------------------
//-- Message COMMAND_LONG
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_command_long_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
}) fmav_command_long_t;


#define FASTMAVLINK_MSG_ID_COMMAND_LONG  76


#define FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MIN  33
#define FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN  33
#define FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA  152

#define FASTMAVLINK_MSG_ID_76_LEN_MIN  33
#define FASTMAVLINK_MSG_ID_76_LEN_MAX  33
#define FASTMAVLINK_MSG_ID_76_LEN  33
#define FASTMAVLINK_MSG_ID_76_CRCEXTRA  152



#define FASTMAVLINK_MSG_COMMAND_LONG_FLAGS  3
#define FASTMAVLINK_MSG_COMMAND_LONG_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_COMMAND_LONG_TARGET_COMPONENT_OFS  31


//----------------------------------------
//-- Message COMMAND_LONG packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7,
    fmav_status_t* _status)
{
    fmav_command_long_t* _payload = (fmav_command_long_t*)msg->payload;

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->param5 = param5;
    _payload->param6 = param6;
    _payload->param7 = param7;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->confirmation = confirmation;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COMMAND_LONG;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_long_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_long_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->command, _payload->confirmation, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->param5, _payload->param6, _payload->param7,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7,
    fmav_status_t* _status)
{
    fmav_command_long_t* _payload = (fmav_command_long_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->param5 = param5;
    _payload->param6 = param6;
    _payload->param7 = param7;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->confirmation = confirmation;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMMAND_LONG;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_LONG >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_LONG >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_long_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_long_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->command, _payload->confirmation, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->param5, _payload->param6, _payload->param7,
        _status);
}


//----------------------------------------
//-- Message COMMAND_LONG unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_command_long_decode(fmav_command_long_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMMAND_LONG  76

#define mavlink_command_long_t  fmav_command_long_t

#define MAVLINK_MSG_ID_COMMAND_LONG_LEN  33
#define MAVLINK_MSG_ID_COMMAND_LONG_MIN_LEN  33
#define MAVLINK_MSG_ID_76_LEN  33
#define MAVLINK_MSG_ID_76_MIN_LEN  33

#define MAVLINK_MSG_ID_COMMAND_LONG_CRC  152
#define MAVLINK_MSG_ID_76_CRC  152




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_long_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_command_long_pack(
        msg, sysid, compid,
        target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_long_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    return fmav_msg_command_long_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_command_long_decode(const mavlink_message_t* msg, mavlink_command_long_t* payload)
{
    fmav_msg_command_long_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMMAND_LONG_H

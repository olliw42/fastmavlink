//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMMAND_INT_H
#define FASTMAVLINK_MSG_COMMAND_INT_H


//----------------------------------------
//-- Message COMMAND_INT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_command_int_t {
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t x;
    int32_t y;
    float z;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint8_t current;
    uint8_t autocontinue;
}) fmav_command_int_t;


#define FASTMAVLINK_MSG_ID_COMMAND_INT  75


#define FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MIN  35
#define FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX  35
#define FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN  35
#define FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA  158

#define FASTMAVLINK_MSG_ID_75_LEN_MIN  35
#define FASTMAVLINK_MSG_ID_75_LEN_MAX  35
#define FASTMAVLINK_MSG_ID_75_LEN  35
#define FASTMAVLINK_MSG_ID_75_CRCEXTRA  158



#define FASTMAVLINK_MSG_COMMAND_INT_FLAGS  3
#define FASTMAVLINK_MSG_COMMAND_INT_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_COMMAND_INT_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_COMMAND_INT_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_75_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_75_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message COMMAND_INT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_command_int_t* _payload = (fmav_command_int_t*)msg->payload;

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COMMAND_INT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_int_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_command_int_t* _payload = (fmav_command_int_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMMAND_INT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_INT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMMAND_INT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_command_int_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_command_int_t _payload;

    _payload.param1 = param1;
    _payload.param2 = param2;
    _payload.param3 = param3;
    _payload.param4 = param4;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.command = command;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.frame = frame;
    _payload.current = current;
    _payload.autocontinue = autocontinue;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMMAND_INT,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_int_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMMAND_INT,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_INT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMMAND_INT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_command_int_decode(fmav_command_int_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_COMMAND_INT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMMAND_INT  75

#define mavlink_command_int_t  fmav_command_int_t

#define MAVLINK_MSG_ID_COMMAND_INT_LEN  35
#define MAVLINK_MSG_ID_COMMAND_INT_MIN_LEN  35
#define MAVLINK_MSG_ID_75_LEN  35
#define MAVLINK_MSG_ID_75_MIN_LEN  35

#define MAVLINK_MSG_ID_COMMAND_INT_CRC  158
#define MAVLINK_MSG_ID_75_CRC  158




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_command_int_pack(
        msg, sysid, compid,
        target_system, target_component, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_command_int_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
    return fmav_msg_command_int_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_command_int_decode(const mavlink_message_t* msg, mavlink_command_int_t* payload)
{
    fmav_msg_command_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMMAND_INT_H

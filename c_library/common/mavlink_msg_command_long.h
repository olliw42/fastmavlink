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

#define FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA  152

#define FASTMAVLINK_MSG_COMMAND_LONG_FLAGS  3
#define FASTMAVLINK_MSG_COMMAND_LONG_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_COMMAND_LONG_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_COMMAND_LONG_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM1_OFS  0
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM2_OFS  4
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM3_OFS  8
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM4_OFS  12
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM5_OFS  16
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM6_OFS  20
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_PARAM7_OFS  24
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_COMMAND_OFS  28
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_TARGET_COMPONENT_OFS  31
#define FASTMAVLINK_MSG_COMMAND_LONG_FIELD_CONFIRMATION_OFS  32


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7,
    fmav_status_t* _status)
{
    fmav_command_long_t _payload;

    _payload.param1 = param1;
    _payload.param2 = param2;
    _payload.param3 = param3;
    _payload.param4 = param4;
    _payload.param5 = param5;
    _payload.param6 = param6;
    _payload.param7 = param7;
    _payload.command = command;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.confirmation = confirmation;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMMAND_LONG,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_command_long_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMMAND_LONG,
        FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMMAND_LONG_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMMAND_LONG unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_command_long_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_command_long_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_command_long_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_command_long_decode(fmav_command_long_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMMAND_LONG_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param5(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param6(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_command_long_get_field_param7(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_command_long_get_field_command(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_command_long_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_command_long_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_command_long_get_field_confirmation(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
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

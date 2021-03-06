//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_EXT_SET_H
#define FASTMAVLINK_MSG_PARAM_EXT_SET_H


//----------------------------------------
//-- Message PARAM_EXT_SET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ext_set_t {
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    char param_value[128];
    uint8_t param_type;
}) fmav_param_ext_set_t;


#define FASTMAVLINK_MSG_ID_PARAM_EXT_SET  323

#define FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX  147
#define FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA  78

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_EXT_SET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_SET_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FRAME_LEN_MAX  172

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_NUM  128 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN  128 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_TARGET_COMPONENT_OFS  1
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_OFS  2
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_OFS  18
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_TYPE_OFS  146


//----------------------------------------
//-- Message PARAM_EXT_SET packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_ext_set_t* _payload = (fmav_param_ext_set_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_EXT_SET;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_set_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_ext_set_t* _payload = (fmav_param_ext_set_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_set_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_ext_set_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.param_type = param_type;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload.param_value), param_value, sizeof(char)*128);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_SET,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_SET,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_EXT_SET unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_param_ext_set_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_param_ext_set_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_set_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_set_decode(fmav_param_ext_set_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_set_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_set_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_set_get_field_param_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[146]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_ext_set_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_ext_set_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[2]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_ext_set_get_field_param_value_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[18]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_ext_set_get_field_param_value(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_NUM) return 0;
    return ((char*)&(msg->payload[18]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_EXT_SET  323

#define mavlink_param_ext_set_t  fmav_param_ext_set_t

#define MAVLINK_MSG_ID_PARAM_EXT_SET_LEN  147
#define MAVLINK_MSG_ID_PARAM_EXT_SET_MIN_LEN  147
#define MAVLINK_MSG_ID_323_LEN  147
#define MAVLINK_MSG_ID_323_MIN_LEN  147

#define MAVLINK_MSG_ID_PARAM_EXT_SET_CRC  78
#define MAVLINK_MSG_ID_323_CRC  78

#define MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_set_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ext_set_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_set_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type)
{
    return fmav_msg_param_ext_set_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ext_set_decode(const mavlink_message_t* msg, mavlink_param_ext_set_t* payload)
{
    fmav_msg_param_ext_set_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_EXT_SET_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_REQUEST_READ_H
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_H


//----------------------------------------
//-- Message PARAM_REQUEST_READ
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_request_read_t {
    int16_t param_index;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
}) fmav_param_request_read_t;


#define FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ  20

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA  214

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FRAME_LEN_MAX  45

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_INDEX_OFS  0
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_OFS  4


//----------------------------------------
//-- Message PARAM_REQUEST_READ packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)msg->payload;

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t _payload;

    _payload.param_index = param_index;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_REQUEST_READ unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_param_request_read_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_param_request_read_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_request_read_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_request_read_decode(fmav_param_request_read_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_param_request_read_get_field_param_index(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_request_read_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_request_read_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_param_request_read_get_field_param_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_param_request_read_get_field_param_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_NUM) return 0;
    return ((char*)&(msg->payload[4]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ  20

#define mavlink_param_request_read_t  fmav_param_request_read_t

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN  20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_MIN_LEN  20
#define MAVLINK_MSG_ID_20_LEN  20
#define MAVLINK_MSG_ID_20_MIN_LEN  20

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_CRC  214
#define MAVLINK_MSG_ID_20_CRC  214

#define MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_request_read_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_index,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_request_read_decode(const mavlink_message_t* msg, mavlink_param_request_read_t* payload)
{
    fmav_msg_param_request_read_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_REQUEST_READ_H

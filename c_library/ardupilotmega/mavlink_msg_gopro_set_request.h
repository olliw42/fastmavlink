//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GOPRO_SET_REQUEST_H
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_H


//----------------------------------------
//-- Message GOPRO_SET_REQUEST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gopro_set_request_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t cmd_id;
    uint8_t value[4];
}) fmav_gopro_set_request_t;


#define FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST  218

#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA  17

#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FLAGS  3
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FRAME_LEN_MAX  32

#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_VALUE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_VALUE_LEN  4 // length of array = number of bytes

#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_TARGET_COMPONENT_OFS  1
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_CMD_ID_OFS  2
#define FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_VALUE_OFS  3


//----------------------------------------
//-- Message GOPRO_SET_REQUEST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value,
    fmav_status_t* _status)
{
    fmav_gopro_set_request_t* _payload = (fmav_gopro_set_request_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;
    memcpy(&(_payload->value), value, sizeof(uint8_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_request_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->cmd_id, _payload->value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value,
    fmav_status_t* _status)
{
    fmav_gopro_set_request_t* _payload = (fmav_gopro_set_request_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;
    memcpy(&(_payload->value), value, sizeof(uint8_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_request_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->cmd_id, _payload->value,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value,
    fmav_status_t* _status)
{
    fmav_gopro_set_request_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.cmd_id = cmd_id;
    memcpy(&(_payload.value), value, sizeof(uint8_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GOPRO_SET_REQUEST unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gopro_set_request_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gopro_set_request_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_set_request_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_set_request_decode(fmav_gopro_set_request_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_set_request_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_set_request_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_set_request_get_field_cmd_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_gopro_set_request_get_field_value_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[3]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_set_request_get_field_value(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_VALUE_NUM) return 0;
    return ((uint8_t*)&(msg->payload[3]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GOPRO_SET_REQUEST  218

#define mavlink_gopro_set_request_t  fmav_gopro_set_request_t

#define MAVLINK_MSG_ID_GOPRO_SET_REQUEST_LEN  7
#define MAVLINK_MSG_ID_GOPRO_SET_REQUEST_MIN_LEN  7
#define MAVLINK_MSG_ID_218_LEN  7
#define MAVLINK_MSG_ID_218_MIN_LEN  7

#define MAVLINK_MSG_ID_GOPRO_SET_REQUEST_CRC  17
#define MAVLINK_MSG_ID_218_CRC  17

#define MAVLINK_MSG_GOPRO_SET_REQUEST_FIELD_VALUE_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gopro_set_request_pack(
        msg, sysid, compid,
        target_system, target_component, cmd_id, value,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_request_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value)
{
    return fmav_msg_gopro_set_request_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, cmd_id, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gopro_set_request_decode(const mavlink_message_t* msg, mavlink_gopro_set_request_t* payload)
{
    fmav_msg_gopro_set_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GOPRO_SET_REQUEST_H

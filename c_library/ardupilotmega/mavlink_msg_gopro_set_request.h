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

// fields are ordered, as they appear on the wire
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
//-- Message GOPRO_SET_REQUEST pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value,
    fmav_status_t* _status)
{
    fmav_gopro_set_request_t* _payload = (fmav_gopro_set_request_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;
    memcpy(&(_payload->value), value, sizeof(uint8_t)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_request_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->cmd_id, _payload->value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value,
    fmav_status_t* _status)
{
    fmav_gopro_set_request_t* _payload = (fmav_gopro_set_request_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;
    memcpy(&(_payload->value), value, sizeof(uint8_t)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_SET_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_request_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_request_pack_to_frame_buf(
        _buf, sysid, compid,
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
//-- Message GOPRO_SET_REQUEST decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_set_request_decode(fmav_gopro_set_request_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_SET_REQUEST_PAYLOAD_LEN_MAX);
#endif
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
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gopro_set_request_pack(
        _msg, sysid, compid,
        target_system, target_component, cmd_id, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_request_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gopro_set_request_t* _payload)
{
    return mavlink_msg_gopro_set_request_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->cmd_id, _payload->value);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_request_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id, const uint8_t* value)
{
    return fmav_msg_gopro_set_request_pack_to_frame_buf(
        (uint8_t*)_buf,
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_CONTROL_H
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_H


//----------------------------------------
//-- Message GIMBAL_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_control_t {
    float demanded_rate_x;
    float demanded_rate_y;
    float demanded_rate_z;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_control_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_CONTROL  201

#define FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA  205

#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_TARGET_COMPONENT_OFS  13

#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FRAME_LEN_MAX  39



#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FIELD_DEMANDED_RATE_X_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FIELD_DEMANDED_RATE_Y_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FIELD_DEMANDED_RATE_Z_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FIELD_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FIELD_TARGET_COMPONENT_OFS  13


//----------------------------------------
//-- Message GIMBAL_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z,
    fmav_status_t* _status)
{
    fmav_gimbal_control_t* _payload = (fmav_gimbal_control_t*)_msg->payload;

    _payload->demanded_rate_x = demanded_rate_x;
    _payload->demanded_rate_y = demanded_rate_y;
    _payload->demanded_rate_z = demanded_rate_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_CONTROL;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_control_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->demanded_rate_x, _payload->demanded_rate_y, _payload->demanded_rate_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z,
    fmav_status_t* _status)
{
    fmav_gimbal_control_t* _payload = (fmav_gimbal_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->demanded_rate_x = demanded_rate_x;
    _payload->demanded_rate_y = demanded_rate_y;
    _payload->demanded_rate_z = demanded_rate_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->demanded_rate_x, _payload->demanded_rate_y, _payload->demanded_rate_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z,
    fmav_status_t* _status)
{
    fmav_gimbal_control_t _payload;

    _payload.demanded_rate_x = demanded_rate_x;
    _payload.demanded_rate_y = demanded_rate_y;
    _payload.demanded_rate_z = demanded_rate_z;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_CONTROL,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_CONTROL,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_control_decode(fmav_gimbal_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_control_get_field_demanded_rate_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_control_get_field_demanded_rate_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_control_get_field_demanded_rate_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_control_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_control_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_CONTROL  201

#define mavlink_gimbal_control_t  fmav_gimbal_control_t

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN  14
#define MAVLINK_MSG_ID_GIMBAL_CONTROL_MIN_LEN  14
#define MAVLINK_MSG_ID_201_LEN  14
#define MAVLINK_MSG_ID_201_MIN_LEN  14

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC  205
#define MAVLINK_MSG_ID_201_CRC  205




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_control_pack(
        _msg, sysid, compid,
        target_system, target_component, demanded_rate_x, demanded_rate_y, demanded_rate_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_control_t* _payload)
{
    return mavlink_msg_gimbal_control_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->demanded_rate_x, _payload->demanded_rate_y, _payload->demanded_rate_z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z)
{
    return fmav_msg_gimbal_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, demanded_rate_x, demanded_rate_y, demanded_rate_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_control_decode(const mavlink_message_t* msg, mavlink_gimbal_control_t* payload)
{
    fmav_msg_gimbal_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_CONTROL_H

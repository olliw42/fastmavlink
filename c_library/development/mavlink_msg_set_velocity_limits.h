//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_H
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_H


//----------------------------------------
//-- Message SET_VELOCITY_LIMITS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_velocity_limits_t {
    float horizontal_speed_limit;
    float vertical_speed_limit;
    float yaw_rate_limit;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_set_velocity_limits_t;


#define FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS  354

#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_CRCEXTRA  210

#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FLAGS  3
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_TARGET_COMPONENT_OFS  13

#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FRAME_LEN_MAX  39



#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FIELD_HORIZONTAL_SPEED_LIMIT_OFS  0
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FIELD_VERTICAL_SPEED_LIMIT_OFS  4
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FIELD_YAW_RATE_LIMIT_OFS  8
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FIELD_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_FIELD_TARGET_COMPONENT_OFS  13


//----------------------------------------
//-- Message SET_VELOCITY_LIMITS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float horizontal_speed_limit, float vertical_speed_limit, float yaw_rate_limit,
    fmav_status_t* _status)
{
    fmav_set_velocity_limits_t* _payload = (fmav_set_velocity_limits_t*)_msg->payload;

    _payload->horizontal_speed_limit = horizontal_speed_limit;
    _payload->vertical_speed_limit = vertical_speed_limit;
    _payload->yaw_rate_limit = yaw_rate_limit;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_velocity_limits_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_velocity_limits_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->horizontal_speed_limit, _payload->vertical_speed_limit, _payload->yaw_rate_limit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float horizontal_speed_limit, float vertical_speed_limit, float yaw_rate_limit,
    fmav_status_t* _status)
{
    fmav_set_velocity_limits_t* _payload = (fmav_set_velocity_limits_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->horizontal_speed_limit = horizontal_speed_limit;
    _payload->vertical_speed_limit = vertical_speed_limit;
    _payload->yaw_rate_limit = yaw_rate_limit;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_velocity_limits_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_velocity_limits_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->horizontal_speed_limit, _payload->vertical_speed_limit, _payload->yaw_rate_limit,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float horizontal_speed_limit, float vertical_speed_limit, float yaw_rate_limit,
    fmav_status_t* _status)
{
    fmav_set_velocity_limits_t _payload;

    _payload.horizontal_speed_limit = horizontal_speed_limit;
    _payload.vertical_speed_limit = vertical_speed_limit;
    _payload.yaw_rate_limit = yaw_rate_limit;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_velocity_limits_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_velocity_limits_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_VELOCITY_LIMITS,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_VELOCITY_LIMITS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_velocity_limits_decode(fmav_set_velocity_limits_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_velocity_limits_get_field_horizontal_speed_limit(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_velocity_limits_get_field_vertical_speed_limit(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_set_velocity_limits_get_field_yaw_rate_limit(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_velocity_limits_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_velocity_limits_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_VELOCITY_LIMITS  354

#define mavlink_set_velocity_limits_t  fmav_set_velocity_limits_t

#define MAVLINK_MSG_ID_SET_VELOCITY_LIMITS_LEN  14
#define MAVLINK_MSG_ID_SET_VELOCITY_LIMITS_MIN_LEN  14
#define MAVLINK_MSG_ID_354_LEN  14
#define MAVLINK_MSG_ID_354_MIN_LEN  14

#define MAVLINK_MSG_ID_SET_VELOCITY_LIMITS_CRC  210
#define MAVLINK_MSG_ID_354_CRC  210




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_velocity_limits_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, float horizontal_speed_limit, float vertical_speed_limit, float yaw_rate_limit)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_velocity_limits_pack(
        _msg, sysid, compid,
        target_system, target_component, horizontal_speed_limit, vertical_speed_limit, yaw_rate_limit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_velocity_limits_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_set_velocity_limits_t* _payload)
{
    return mavlink_msg_set_velocity_limits_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->horizontal_speed_limit, _payload->vertical_speed_limit, _payload->yaw_rate_limit);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_velocity_limits_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float horizontal_speed_limit, float vertical_speed_limit, float yaw_rate_limit)
{
    return fmav_msg_set_velocity_limits_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, horizontal_speed_limit, vertical_speed_limit, yaw_rate_limit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_velocity_limits_decode(const mavlink_message_t* msg, mavlink_set_velocity_limits_t* payload)
{
    fmav_msg_set_velocity_limits_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_VELOCITY_LIMITS_H

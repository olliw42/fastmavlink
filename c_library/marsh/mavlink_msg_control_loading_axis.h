//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_H
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_H


//----------------------------------------
//-- Message CONTROL_LOADING_AXIS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_control_loading_axis_t {
    uint32_t time_boot_ms;
    float position;
    float velocity;
    float force;
    uint8_t axis;
}) fmav_control_loading_axis_t;


#define FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS  52501

#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_CRCEXTRA  240

#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FLAGS  0
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FRAME_LEN_MAX  42



#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FIELD_POSITION_OFS  4
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FIELD_VELOCITY_OFS  8
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FIELD_FORCE_OFS  12
#define FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_FIELD_AXIS_OFS  16


//----------------------------------------
//-- Message CONTROL_LOADING_AXIS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t axis, float position, float velocity, float force,
    fmav_status_t* _status)
{
    fmav_control_loading_axis_t* _payload = (fmav_control_loading_axis_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->position = position;
    _payload->velocity = velocity;
    _payload->force = force;
    _payload->axis = axis;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_loading_axis_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_loading_axis_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->axis, _payload->position, _payload->velocity, _payload->force,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t axis, float position, float velocity, float force,
    fmav_status_t* _status)
{
    fmav_control_loading_axis_t* _payload = (fmav_control_loading_axis_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->position = position;
    _payload->velocity = velocity;
    _payload->force = force;
    _payload->axis = axis;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_loading_axis_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_loading_axis_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->axis, _payload->position, _payload->velocity, _payload->force,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t axis, float position, float velocity, float force,
    fmav_status_t* _status)
{
    fmav_control_loading_axis_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.position = position;
    _payload.velocity = velocity;
    _payload.force = force;
    _payload.axis = axis;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_loading_axis_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_loading_axis_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CONTROL_LOADING_AXIS,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CONTROL_LOADING_AXIS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_control_loading_axis_decode(fmav_control_loading_axis_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_control_loading_axis_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_loading_axis_get_field_position(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_loading_axis_get_field_velocity(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_control_loading_axis_get_field_force(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_control_loading_axis_get_field_axis(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CONTROL_LOADING_AXIS  52501

#define mavlink_control_loading_axis_t  fmav_control_loading_axis_t

#define MAVLINK_MSG_ID_CONTROL_LOADING_AXIS_LEN  17
#define MAVLINK_MSG_ID_CONTROL_LOADING_AXIS_MIN_LEN  17
#define MAVLINK_MSG_ID_52501_LEN  17
#define MAVLINK_MSG_ID_52501_MIN_LEN  17

#define MAVLINK_MSG_ID_CONTROL_LOADING_AXIS_CRC  240
#define MAVLINK_MSG_ID_52501_CRC  240




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_loading_axis_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t axis, float position, float velocity, float force)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_control_loading_axis_pack(
        _msg, sysid, compid,
        time_boot_ms, axis, position, velocity, force,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_loading_axis_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_control_loading_axis_t* _payload)
{
    return mavlink_msg_control_loading_axis_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->axis, _payload->position, _payload->velocity, _payload->force);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_loading_axis_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t axis, float position, float velocity, float force)
{
    return fmav_msg_control_loading_axis_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, axis, position, velocity, force,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_control_loading_axis_decode(const mavlink_message_t* msg, mavlink_control_loading_axis_t* payload)
{
    fmav_msg_control_loading_axis_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CONTROL_LOADING_AXIS_H

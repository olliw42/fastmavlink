//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_H
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_H


//----------------------------------------
//-- Message STORM32_GIMBAL_DEVICE_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_storm32_gimbal_device_control_t {
    float q[4];
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    uint16_t flags;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_storm32_gimbal_device_control_t;


#define FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL  60002

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA  69

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FRAME_LEN_MAX  57

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_ANGULAR_VELOCITY_X_OFS  16
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_ANGULAR_VELOCITY_Y_OFS  20
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_ANGULAR_VELOCITY_Z_OFS  24
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_FLAGS_OFS  28
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_TARGET_COMPONENT_OFS  31


//----------------------------------------
//-- Message STORM32_GIMBAL_DEVICE_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_device_control_t* _payload = (fmav_storm32_gimbal_device_control_t*)_msg->payload;

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_device_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_device_control_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_device_control_t* _payload = (fmav_storm32_gimbal_device_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_device_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_device_control_t _payload;

    _payload.angular_velocity_x = angular_velocity_x;
    _payload.angular_velocity_y = angular_velocity_y;
    _payload.angular_velocity_z = angular_velocity_z;
    _payload.flags = flags;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_device_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STORM32_GIMBAL_DEVICE_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_device_control_decode(fmav_storm32_gimbal_device_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_device_control_get_field_angular_velocity_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_device_control_get_field_angular_velocity_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_device_control_get_field_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_device_control_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_device_control_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_storm32_gimbal_device_control_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_device_control_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL  60002

#define mavlink_storm32_gimbal_device_control_t  fmav_storm32_gimbal_device_control_t

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL_LEN  32
#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL_MIN_LEN  32
#define MAVLINK_MSG_ID_60002_LEN  32
#define MAVLINK_MSG_ID_60002_MIN_LEN  32

#define MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL_CRC  69
#define MAVLINK_MSG_ID_60002_CRC  69

#define MAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_device_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storm32_gimbal_device_control_pack(
        _msg, sysid, compid,
        target_system, target_component, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_device_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    return fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_storm32_gimbal_device_control_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_device_control_t* payload)
{
    fmav_msg_storm32_gimbal_device_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_H

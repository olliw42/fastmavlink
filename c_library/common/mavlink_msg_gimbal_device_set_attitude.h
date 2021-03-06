//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_H
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_H


//----------------------------------------
//-- Message GIMBAL_DEVICE_SET_ATTITUDE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_device_set_attitude_t {
    float q[4];
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    uint16_t flags;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_device_set_attitude_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE  284

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA  99

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FRAME_LEN_MAX  57

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_ANGULAR_VELOCITY_X_OFS  16
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_ANGULAR_VELOCITY_Y_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_ANGULAR_VELOCITY_Z_OFS  24
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_FLAGS_OFS  28
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_TARGET_COMPONENT_OFS  31


//----------------------------------------
//-- Message GIMBAL_DEVICE_SET_ATTITUDE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_gimbal_device_set_attitude_t* _payload = (fmav_gimbal_device_set_attitude_t*)msg->payload;

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_set_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_set_attitude_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_gimbal_device_set_attitude_t* _payload = (fmav_gimbal_device_set_attitude_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_set_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_set_attitude_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_gimbal_device_set_attitude_t _payload;

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
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_set_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_DEVICE_SET_ATTITUDE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gimbal_device_set_attitude_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gimbal_device_set_attitude_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_set_attitude_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_set_attitude_decode(fmav_gimbal_device_set_attitude_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_set_attitude_get_field_angular_velocity_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_set_attitude_get_field_angular_velocity_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_set_attitude_get_field_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_set_attitude_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_device_set_attitude_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_device_set_attitude_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_gimbal_device_set_attitude_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_set_attitude_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE  284

#define mavlink_gimbal_device_set_attitude_t  fmav_gimbal_device_set_attitude_t

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_LEN  32
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_MIN_LEN  32
#define MAVLINK_MSG_ID_284_LEN  32
#define MAVLINK_MSG_ID_284_MIN_LEN  32

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE_CRC  99
#define MAVLINK_MSG_ID_284_CRC  99

#define MAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_set_attitude_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_device_set_attitude_pack(
        msg, sysid, compid,
        target_system, target_component, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_set_attitude_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    return fmav_msg_gimbal_device_set_attitude_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_device_set_attitude_decode(const mavlink_message_t* msg, mavlink_gimbal_device_set_attitude_t* payload)
{
    fmav_msg_gimbal_device_set_attitude_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_DEVICE_SET_ATTITUDE_H

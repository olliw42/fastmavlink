//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_H
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_H


//----------------------------------------
//-- Message GIMBAL_DEVICE_ATTITUDE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_device_attitude_status_t {
    uint32_t time_boot_ms;
    float q[4];
    float angular_velocity_x;
    float angular_velocity_y;
    float angular_velocity_z;
    uint32_t failure_flags;
    uint16_t flags;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_device_attitude_status_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS  285

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA  137

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_TARGET_SYSTEM_OFS  38
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_TARGET_COMPONENT_OFS  39

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FRAME_LEN_MAX  65

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_ANGULAR_VELOCITY_X_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_ANGULAR_VELOCITY_Y_OFS  24
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_ANGULAR_VELOCITY_Z_OFS  28
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_FAILURE_FLAGS_OFS  32
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_FLAGS_OFS  36
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_TARGET_SYSTEM_OFS  38
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_TARGET_COMPONENT_OFS  39


//----------------------------------------
//-- Message GIMBAL_DEVICE_ATTITUDE_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags,
    fmav_status_t* _status)
{
    fmav_gimbal_device_attitude_status_t* _payload = (fmav_gimbal_device_attitude_status_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->failure_flags = failure_flags;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_attitude_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_attitude_status_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_ms, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z, _payload->failure_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags,
    fmav_status_t* _status)
{
    fmav_gimbal_device_attitude_status_t* _payload = (fmav_gimbal_device_attitude_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->failure_flags = failure_flags;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_attitude_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_attitude_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_ms, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z, _payload->failure_flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags,
    fmav_status_t* _status)
{
    fmav_gimbal_device_attitude_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.angular_velocity_x = angular_velocity_x;
    _payload.angular_velocity_y = angular_velocity_y;
    _payload.angular_velocity_z = angular_velocity_z;
    _payload.failure_flags = failure_flags;
    _payload.flags = flags;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_attitude_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_DEVICE_ATTITUDE_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gimbal_device_attitude_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gimbal_device_attitude_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_attitude_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_attitude_status_decode(fmav_gimbal_device_attitude_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_device_attitude_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_attitude_status_get_field_angular_velocity_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_attitude_status_get_field_angular_velocity_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_attitude_status_get_field_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_device_attitude_status_get_field_failure_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_attitude_status_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_device_attitude_status_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_device_attitude_status_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[39]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_gimbal_device_attitude_status_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_attitude_status_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[4]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS  285

#define mavlink_gimbal_device_attitude_status_t  fmav_gimbal_device_attitude_status_t

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN  40
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_MIN_LEN  40
#define MAVLINK_MSG_ID_285_LEN  40
#define MAVLINK_MSG_ID_285_MIN_LEN  40

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_CRC  137
#define MAVLINK_MSG_ID_285_CRC  137

#define MAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_attitude_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_device_attitude_status_pack(
        msg, sysid, compid,
        target_system, target_component, time_boot_ms, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z, failure_flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_attitude_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t time_boot_ms, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z, uint32_t failure_flags)
{
    return fmav_msg_gimbal_device_attitude_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, time_boot_ms, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z, failure_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_device_attitude_status_decode(const mavlink_message_t* msg, mavlink_gimbal_device_attitude_status_t* payload)
{
    fmav_msg_gimbal_device_attitude_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_DEVICE_ATTITUDE_STATUS_H

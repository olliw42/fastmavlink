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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN  32
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA  69

#define FASTMAVLINK_MSG_ID_60002_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_60002_LEN_MAX  32
#define FASTMAVLINK_MSG_ID_60002_LEN  32
#define FASTMAVLINK_MSG_ID_60002_CRCEXTRA  69

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_60002_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_60002_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message STORM32_GIMBAL_DEVICE_CONTROL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_device_control_t* _payload = (fmav_storm32_gimbal_device_control_t*)msg->payload;

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_device_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_device_control_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->flags, _payload->q, _payload->angular_velocity_x, _payload->angular_velocity_y, _payload->angular_velocity_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_device_control_t* _payload = (fmav_storm32_gimbal_device_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->angular_velocity_x = angular_velocity_x;
    _payload->angular_velocity_y = angular_velocity_y;
    _payload->angular_velocity_z = angular_velocity_z;
    _payload->flags = flags;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_device_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_device_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
        buf, sysid, compid,
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
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STORM32_GIMBAL_DEVICE_CONTROL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_device_control_decode(fmav_storm32_gimbal_device_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_STORM32_GIMBAL_DEVICE_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storm32_gimbal_device_control_pack(
        msg, sysid, compid,
        target_system, target_component, flags, q, angular_velocity_x, angular_velocity_y, angular_velocity_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_device_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t flags, const float* q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
{
    return fmav_msg_storm32_gimbal_device_control_pack_to_frame_buf(
        (uint8_t*)buf,
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

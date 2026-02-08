//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_H
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_H


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_state_for_gimbal_device_ext_t {
    uint64_t time_boot_us;
    float wind_x;
    float wind_y;
    float wind_correction_angle;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_autopilot_state_for_gimbal_device_ext_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT  60000

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA  4

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FLAGS  3
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_TARGET_SYSTEM_OFS  20
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_TARGET_COMPONENT_OFS  21

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FRAME_LEN_MAX  47



#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_TIME_BOOT_US_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_WIND_X_OFS  8
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_WIND_Y_OFS  12
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_WIND_CORRECTION_ANGLE_OFS  16
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_TARGET_SYSTEM_OFS  20
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_FIELD_TARGET_COMPONENT_OFS  21


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, float wind_x, float wind_y, float wind_correction_angle,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_ext_t* _payload = (fmav_autopilot_state_for_gimbal_device_ext_t*)_msg->payload;

    _payload->time_boot_us = time_boot_us;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_correction_angle = wind_correction_angle;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_ext_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->wind_x, _payload->wind_y, _payload->wind_correction_angle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, float wind_x, float wind_y, float wind_correction_angle,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_ext_t* _payload = (fmav_autopilot_state_for_gimbal_device_ext_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_us = time_boot_us;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_correction_angle = wind_correction_angle;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_ext_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->wind_x, _payload->wind_y, _payload->wind_correction_angle,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, float wind_x, float wind_y, float wind_correction_angle,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_ext_t _payload;

    _payload.time_boot_us = time_boot_us;
    _payload.wind_x = wind_x;
    _payload.wind_y = wind_y;
    _payload.wind_correction_angle = wind_correction_angle;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_ext_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_state_for_gimbal_device_ext_decode(fmav_autopilot_state_for_gimbal_device_ext_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_time_boot_us(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_wind_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_wind_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_wind_correction_angle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_ext_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT  60000

#define mavlink_autopilot_state_for_gimbal_device_ext_t  fmav_autopilot_state_for_gimbal_device_ext_t

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_LEN  22
#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_MIN_LEN  22
#define MAVLINK_MSG_ID_60000_LEN  22
#define MAVLINK_MSG_ID_60000_MIN_LEN  22

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_CRC  4
#define MAVLINK_MSG_ID_60000_CRC  4




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_ext_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, float wind_x, float wind_y, float wind_correction_angle)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_state_for_gimbal_device_ext_pack(
        _msg, sysid, compid,
        target_system, target_component, time_boot_us, wind_x, wind_y, wind_correction_angle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_ext_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_autopilot_state_for_gimbal_device_ext_t* _payload)
{
    return mavlink_msg_autopilot_state_for_gimbal_device_ext_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->wind_x, _payload->wind_y, _payload->wind_correction_angle);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_ext_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, float wind_x, float wind_y, float wind_correction_angle)
{
    return fmav_msg_autopilot_state_for_gimbal_device_ext_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, time_boot_us, wind_x, wind_y, wind_correction_angle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_state_for_gimbal_device_ext_decode(const mavlink_message_t* msg, mavlink_autopilot_state_for_gimbal_device_ext_t* payload)
{
    fmav_msg_autopilot_state_for_gimbal_device_ext_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT_H

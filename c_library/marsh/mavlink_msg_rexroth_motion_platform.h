//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_H
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_H


//----------------------------------------
//-- Message REXROTH_MOTION_PLATFORM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rexroth_motion_platform_t {
    uint32_t time_boot_ms;
    uint32_t frame_count;
    uint32_t motion_status;
    float actuator1;
    float actuator2;
    float actuator3;
    float actuator4;
    float actuator5;
    float actuator6;
    float platform_setpoint_x;
    float platform_setpoint_y;
    float platform_setpoint_z;
    float platform_setpoint_roll;
    float platform_setpoint_pitch;
    float platform_setpoint_yaw;
    float effect_setpoint_x;
    float effect_setpoint_y;
    float effect_setpoint_z;
    float effect_setpoint_roll;
    float effect_setpoint_pitch;
    float effect_setpoint_yaw;
    uint8_t error_code;
}) fmav_rexroth_motion_platform_t;


#define FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM  52503

#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX  85
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_CRCEXTRA  96

#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FLAGS  0
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FRAME_LEN_MAX  110



#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_FRAME_COUNT_OFS  4
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_MOTION_STATUS_OFS  8
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR1_OFS  12
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR2_OFS  16
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR3_OFS  20
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR4_OFS  24
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR5_OFS  28
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ACTUATOR6_OFS  32
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_X_OFS  36
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_Y_OFS  40
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_Z_OFS  44
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_ROLL_OFS  48
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_PITCH_OFS  52
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_PLATFORM_SETPOINT_YAW_OFS  56
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_X_OFS  60
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_Y_OFS  64
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_Z_OFS  68
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_ROLL_OFS  72
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_PITCH_OFS  76
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_EFFECT_SETPOINT_YAW_OFS  80
#define FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_FIELD_ERROR_CODE_OFS  84


//----------------------------------------
//-- Message REXROTH_MOTION_PLATFORM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t frame_count, uint32_t motion_status, uint8_t error_code, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5, float actuator6, float platform_setpoint_x, float platform_setpoint_y, float platform_setpoint_z, float platform_setpoint_roll, float platform_setpoint_pitch, float platform_setpoint_yaw, float effect_setpoint_x, float effect_setpoint_y, float effect_setpoint_z, float effect_setpoint_roll, float effect_setpoint_pitch, float effect_setpoint_yaw,
    fmav_status_t* _status)
{
    fmav_rexroth_motion_platform_t* _payload = (fmav_rexroth_motion_platform_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->frame_count = frame_count;
    _payload->motion_status = motion_status;
    _payload->actuator1 = actuator1;
    _payload->actuator2 = actuator2;
    _payload->actuator3 = actuator3;
    _payload->actuator4 = actuator4;
    _payload->actuator5 = actuator5;
    _payload->actuator6 = actuator6;
    _payload->platform_setpoint_x = platform_setpoint_x;
    _payload->platform_setpoint_y = platform_setpoint_y;
    _payload->platform_setpoint_z = platform_setpoint_z;
    _payload->platform_setpoint_roll = platform_setpoint_roll;
    _payload->platform_setpoint_pitch = platform_setpoint_pitch;
    _payload->platform_setpoint_yaw = platform_setpoint_yaw;
    _payload->effect_setpoint_x = effect_setpoint_x;
    _payload->effect_setpoint_y = effect_setpoint_y;
    _payload->effect_setpoint_z = effect_setpoint_z;
    _payload->effect_setpoint_roll = effect_setpoint_roll;
    _payload->effect_setpoint_pitch = effect_setpoint_pitch;
    _payload->effect_setpoint_yaw = effect_setpoint_yaw;
    _payload->error_code = error_code;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rexroth_motion_platform_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rexroth_motion_platform_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->frame_count, _payload->motion_status, _payload->error_code, _payload->actuator1, _payload->actuator2, _payload->actuator3, _payload->actuator4, _payload->actuator5, _payload->actuator6, _payload->platform_setpoint_x, _payload->platform_setpoint_y, _payload->platform_setpoint_z, _payload->platform_setpoint_roll, _payload->platform_setpoint_pitch, _payload->platform_setpoint_yaw, _payload->effect_setpoint_x, _payload->effect_setpoint_y, _payload->effect_setpoint_z, _payload->effect_setpoint_roll, _payload->effect_setpoint_pitch, _payload->effect_setpoint_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t frame_count, uint32_t motion_status, uint8_t error_code, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5, float actuator6, float platform_setpoint_x, float platform_setpoint_y, float platform_setpoint_z, float platform_setpoint_roll, float platform_setpoint_pitch, float platform_setpoint_yaw, float effect_setpoint_x, float effect_setpoint_y, float effect_setpoint_z, float effect_setpoint_roll, float effect_setpoint_pitch, float effect_setpoint_yaw,
    fmav_status_t* _status)
{
    fmav_rexroth_motion_platform_t* _payload = (fmav_rexroth_motion_platform_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->frame_count = frame_count;
    _payload->motion_status = motion_status;
    _payload->actuator1 = actuator1;
    _payload->actuator2 = actuator2;
    _payload->actuator3 = actuator3;
    _payload->actuator4 = actuator4;
    _payload->actuator5 = actuator5;
    _payload->actuator6 = actuator6;
    _payload->platform_setpoint_x = platform_setpoint_x;
    _payload->platform_setpoint_y = platform_setpoint_y;
    _payload->platform_setpoint_z = platform_setpoint_z;
    _payload->platform_setpoint_roll = platform_setpoint_roll;
    _payload->platform_setpoint_pitch = platform_setpoint_pitch;
    _payload->platform_setpoint_yaw = platform_setpoint_yaw;
    _payload->effect_setpoint_x = effect_setpoint_x;
    _payload->effect_setpoint_y = effect_setpoint_y;
    _payload->effect_setpoint_z = effect_setpoint_z;
    _payload->effect_setpoint_roll = effect_setpoint_roll;
    _payload->effect_setpoint_pitch = effect_setpoint_pitch;
    _payload->effect_setpoint_yaw = effect_setpoint_yaw;
    _payload->error_code = error_code;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rexroth_motion_platform_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rexroth_motion_platform_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->frame_count, _payload->motion_status, _payload->error_code, _payload->actuator1, _payload->actuator2, _payload->actuator3, _payload->actuator4, _payload->actuator5, _payload->actuator6, _payload->platform_setpoint_x, _payload->platform_setpoint_y, _payload->platform_setpoint_z, _payload->platform_setpoint_roll, _payload->platform_setpoint_pitch, _payload->platform_setpoint_yaw, _payload->effect_setpoint_x, _payload->effect_setpoint_y, _payload->effect_setpoint_z, _payload->effect_setpoint_roll, _payload->effect_setpoint_pitch, _payload->effect_setpoint_yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t frame_count, uint32_t motion_status, uint8_t error_code, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5, float actuator6, float platform_setpoint_x, float platform_setpoint_y, float platform_setpoint_z, float platform_setpoint_roll, float platform_setpoint_pitch, float platform_setpoint_yaw, float effect_setpoint_x, float effect_setpoint_y, float effect_setpoint_z, float effect_setpoint_roll, float effect_setpoint_pitch, float effect_setpoint_yaw,
    fmav_status_t* _status)
{
    fmav_rexroth_motion_platform_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.frame_count = frame_count;
    _payload.motion_status = motion_status;
    _payload.actuator1 = actuator1;
    _payload.actuator2 = actuator2;
    _payload.actuator3 = actuator3;
    _payload.actuator4 = actuator4;
    _payload.actuator5 = actuator5;
    _payload.actuator6 = actuator6;
    _payload.platform_setpoint_x = platform_setpoint_x;
    _payload.platform_setpoint_y = platform_setpoint_y;
    _payload.platform_setpoint_z = platform_setpoint_z;
    _payload.platform_setpoint_roll = platform_setpoint_roll;
    _payload.platform_setpoint_pitch = platform_setpoint_pitch;
    _payload.platform_setpoint_yaw = platform_setpoint_yaw;
    _payload.effect_setpoint_x = effect_setpoint_x;
    _payload.effect_setpoint_y = effect_setpoint_y;
    _payload.effect_setpoint_z = effect_setpoint_z;
    _payload.effect_setpoint_roll = effect_setpoint_roll;
    _payload.effect_setpoint_pitch = effect_setpoint_pitch;
    _payload.effect_setpoint_yaw = effect_setpoint_yaw;
    _payload.error_code = error_code;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rexroth_motion_platform_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rexroth_motion_platform_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message REXROTH_MOTION_PLATFORM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rexroth_motion_platform_decode(fmav_rexroth_motion_platform_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rexroth_motion_platform_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rexroth_motion_platform_get_field_frame_count(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rexroth_motion_platform_get_field_motion_status(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator5(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_actuator6(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_platform_setpoint_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[76]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rexroth_motion_platform_get_field_effect_setpoint_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[80]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rexroth_motion_platform_get_field_error_code(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[84]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM  52503

#define mavlink_rexroth_motion_platform_t  fmav_rexroth_motion_platform_t

#define MAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM_LEN  85
#define MAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM_MIN_LEN  85
#define MAVLINK_MSG_ID_52503_LEN  85
#define MAVLINK_MSG_ID_52503_MIN_LEN  85

#define MAVLINK_MSG_ID_REXROTH_MOTION_PLATFORM_CRC  96
#define MAVLINK_MSG_ID_52503_CRC  96




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rexroth_motion_platform_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t frame_count, uint32_t motion_status, uint8_t error_code, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5, float actuator6, float platform_setpoint_x, float platform_setpoint_y, float platform_setpoint_z, float platform_setpoint_roll, float platform_setpoint_pitch, float platform_setpoint_yaw, float effect_setpoint_x, float effect_setpoint_y, float effect_setpoint_z, float effect_setpoint_roll, float effect_setpoint_pitch, float effect_setpoint_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rexroth_motion_platform_pack(
        _msg, sysid, compid,
        time_boot_ms, frame_count, motion_status, error_code, actuator1, actuator2, actuator3, actuator4, actuator5, actuator6, platform_setpoint_x, platform_setpoint_y, platform_setpoint_z, platform_setpoint_roll, platform_setpoint_pitch, platform_setpoint_yaw, effect_setpoint_x, effect_setpoint_y, effect_setpoint_z, effect_setpoint_roll, effect_setpoint_pitch, effect_setpoint_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rexroth_motion_platform_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_rexroth_motion_platform_t* _payload)
{
    return mavlink_msg_rexroth_motion_platform_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->frame_count, _payload->motion_status, _payload->error_code, _payload->actuator1, _payload->actuator2, _payload->actuator3, _payload->actuator4, _payload->actuator5, _payload->actuator6, _payload->platform_setpoint_x, _payload->platform_setpoint_y, _payload->platform_setpoint_z, _payload->platform_setpoint_roll, _payload->platform_setpoint_pitch, _payload->platform_setpoint_yaw, _payload->effect_setpoint_x, _payload->effect_setpoint_y, _payload->effect_setpoint_z, _payload->effect_setpoint_roll, _payload->effect_setpoint_pitch, _payload->effect_setpoint_yaw);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rexroth_motion_platform_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t frame_count, uint32_t motion_status, uint8_t error_code, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5, float actuator6, float platform_setpoint_x, float platform_setpoint_y, float platform_setpoint_z, float platform_setpoint_roll, float platform_setpoint_pitch, float platform_setpoint_yaw, float effect_setpoint_x, float effect_setpoint_y, float effect_setpoint_z, float effect_setpoint_roll, float effect_setpoint_pitch, float effect_setpoint_yaw)
{
    return fmav_msg_rexroth_motion_platform_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, frame_count, motion_status, error_code, actuator1, actuator2, actuator3, actuator4, actuator5, actuator6, platform_setpoint_x, platform_setpoint_y, platform_setpoint_z, platform_setpoint_roll, platform_setpoint_pitch, platform_setpoint_yaw, effect_setpoint_x, effect_setpoint_y, effect_setpoint_z, effect_setpoint_roll, effect_setpoint_pitch, effect_setpoint_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rexroth_motion_platform_decode(const mavlink_message_t* msg, mavlink_rexroth_motion_platform_t* payload)
{
    fmav_msg_rexroth_motion_platform_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_REXROTH_MOTION_PLATFORM_H

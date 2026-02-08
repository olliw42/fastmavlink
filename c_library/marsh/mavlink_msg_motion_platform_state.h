//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_H
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_H


//----------------------------------------
//-- Message MOTION_PLATFORM_STATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_motion_platform_state_t {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float vel_x;
    float vel_y;
    float vel_z;
    float vel_roll;
    float vel_pitch;
    float vel_yaw;
    float acc_x;
    float acc_y;
    float acc_z;
    float acc_roll;
    float acc_pitch;
    float acc_yaw;
    uint8_t health;
    uint8_t mode;
}) fmav_motion_platform_state_t;


#define FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE  52502

#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX  78
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_CRCEXTRA  88

#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FLAGS  0
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FRAME_LEN_MAX  103



#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_X_OFS  4
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_Y_OFS  8
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_Z_OFS  12
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ROLL_OFS  16
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_PITCH_OFS  20
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_YAW_OFS  24
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_X_OFS  28
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_Y_OFS  32
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_Z_OFS  36
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_ROLL_OFS  40
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_PITCH_OFS  44
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_VEL_YAW_OFS  48
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_X_OFS  52
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_Y_OFS  56
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_Z_OFS  60
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_ROLL_OFS  64
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_PITCH_OFS  68
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_ACC_YAW_OFS  72
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_HEALTH_OFS  76
#define FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_FIELD_MODE_OFS  77


//----------------------------------------
//-- Message MOTION_PLATFORM_STATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t health, uint8_t mode, float x, float y, float z, float roll, float pitch, float yaw, float vel_x, float vel_y, float vel_z, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z, float acc_roll, float acc_pitch, float acc_yaw,
    fmav_status_t* _status)
{
    fmav_motion_platform_state_t* _payload = (fmav_motion_platform_state_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->vel_x = vel_x;
    _payload->vel_y = vel_y;
    _payload->vel_z = vel_z;
    _payload->vel_roll = vel_roll;
    _payload->vel_pitch = vel_pitch;
    _payload->vel_yaw = vel_yaw;
    _payload->acc_x = acc_x;
    _payload->acc_y = acc_y;
    _payload->acc_z = acc_z;
    _payload->acc_roll = acc_roll;
    _payload->acc_pitch = acc_pitch;
    _payload->acc_yaw = acc_yaw;
    _payload->health = health;
    _payload->mode = mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_platform_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_motion_platform_state_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->health, _payload->mode, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->vel_x, _payload->vel_y, _payload->vel_z, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z, _payload->acc_roll, _payload->acc_pitch, _payload->acc_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t health, uint8_t mode, float x, float y, float z, float roll, float pitch, float yaw, float vel_x, float vel_y, float vel_z, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z, float acc_roll, float acc_pitch, float acc_yaw,
    fmav_status_t* _status)
{
    fmav_motion_platform_state_t* _payload = (fmav_motion_platform_state_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->vel_x = vel_x;
    _payload->vel_y = vel_y;
    _payload->vel_z = vel_z;
    _payload->vel_roll = vel_roll;
    _payload->vel_pitch = vel_pitch;
    _payload->vel_yaw = vel_yaw;
    _payload->acc_x = acc_x;
    _payload->acc_y = acc_y;
    _payload->acc_z = acc_z;
    _payload->acc_roll = acc_roll;
    _payload->acc_pitch = acc_pitch;
    _payload->acc_yaw = acc_yaw;
    _payload->health = health;
    _payload->mode = mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_platform_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_motion_platform_state_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->health, _payload->mode, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->vel_x, _payload->vel_y, _payload->vel_z, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z, _payload->acc_roll, _payload->acc_pitch, _payload->acc_yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t health, uint8_t mode, float x, float y, float z, float roll, float pitch, float yaw, float vel_x, float vel_y, float vel_z, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z, float acc_roll, float acc_pitch, float acc_yaw,
    fmav_status_t* _status)
{
    fmav_motion_platform_state_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.vel_x = vel_x;
    _payload.vel_y = vel_y;
    _payload.vel_z = vel_z;
    _payload.vel_roll = vel_roll;
    _payload.vel_pitch = vel_pitch;
    _payload.vel_yaw = vel_yaw;
    _payload.acc_x = acc_x;
    _payload.acc_y = acc_y;
    _payload.acc_z = acc_z;
    _payload.acc_roll = acc_roll;
    _payload.acc_pitch = acc_pitch;
    _payload.acc_yaw = acc_yaw;
    _payload.health = health;
    _payload.mode = mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_platform_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_platform_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MOTION_PLATFORM_STATE,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MOTION_PLATFORM_STATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_motion_platform_state_decode(fmav_motion_platform_state_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_motion_platform_state_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_vel_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_platform_state_get_field_acc_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_motion_platform_state_get_field_health(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[76]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_motion_platform_state_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[77]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOTION_PLATFORM_STATE  52502

#define mavlink_motion_platform_state_t  fmav_motion_platform_state_t

#define MAVLINK_MSG_ID_MOTION_PLATFORM_STATE_LEN  78
#define MAVLINK_MSG_ID_MOTION_PLATFORM_STATE_MIN_LEN  78
#define MAVLINK_MSG_ID_52502_LEN  78
#define MAVLINK_MSG_ID_52502_MIN_LEN  78

#define MAVLINK_MSG_ID_MOTION_PLATFORM_STATE_CRC  88
#define MAVLINK_MSG_ID_52502_CRC  88




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_platform_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t health, uint8_t mode, float x, float y, float z, float roll, float pitch, float yaw, float vel_x, float vel_y, float vel_z, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z, float acc_roll, float acc_pitch, float acc_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_motion_platform_state_pack(
        _msg, sysid, compid,
        time_boot_ms, health, mode, x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw, acc_x, acc_y, acc_z, acc_roll, acc_pitch, acc_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_platform_state_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_motion_platform_state_t* _payload)
{
    return mavlink_msg_motion_platform_state_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->health, _payload->mode, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->vel_x, _payload->vel_y, _payload->vel_z, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z, _payload->acc_roll, _payload->acc_pitch, _payload->acc_yaw);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_platform_state_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t health, uint8_t mode, float x, float y, float z, float roll, float pitch, float yaw, float vel_x, float vel_y, float vel_z, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z, float acc_roll, float acc_pitch, float acc_yaw)
{
    return fmav_msg_motion_platform_state_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, health, mode, x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw, acc_x, acc_y, acc_z, acc_roll, acc_pitch, acc_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_motion_platform_state_decode(const mavlink_message_t* msg, mavlink_motion_platform_state_t* payload)
{
    fmav_msg_motion_platform_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOTION_PLATFORM_STATE_H

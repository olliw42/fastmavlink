//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOTION_CUE_EXTRA_H
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_H


//----------------------------------------
//-- Message MOTION_CUE_EXTRA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_motion_cue_extra_t {
    uint32_t time_boot_ms;
    float vel_roll;
    float vel_pitch;
    float vel_yaw;
    float acc_x;
    float acc_y;
    float acc_z;
}) fmav_motion_cue_extra_t;


#define FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA  52504

#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_CRCEXTRA  177

#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FLAGS  0
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_VEL_ROLL_OFS  4
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_VEL_PITCH_OFS  8
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_VEL_YAW_OFS  12
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_ACC_X_OFS  16
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_ACC_Y_OFS  20
#define FASTMAVLINK_MSG_MOTION_CUE_EXTRA_FIELD_ACC_Z_OFS  24


//----------------------------------------
//-- Message MOTION_CUE_EXTRA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z,
    fmav_status_t* _status)
{
    fmav_motion_cue_extra_t* _payload = (fmav_motion_cue_extra_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->vel_roll = vel_roll;
    _payload->vel_pitch = vel_pitch;
    _payload->vel_yaw = vel_yaw;
    _payload->acc_x = acc_x;
    _payload->acc_y = acc_y;
    _payload->acc_z = acc_z;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MOTION_CUE_EXTRA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_cue_extra_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_motion_cue_extra_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z,
    fmav_status_t* _status)
{
    fmav_motion_cue_extra_t* _payload = (fmav_motion_cue_extra_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->vel_roll = vel_roll;
    _payload->vel_pitch = vel_pitch;
    _payload->vel_yaw = vel_yaw;
    _payload->acc_x = acc_x;
    _payload->acc_y = acc_y;
    _payload->acc_z = acc_z;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_cue_extra_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_motion_cue_extra_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z,
    fmav_status_t* _status)
{
    fmav_motion_cue_extra_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.vel_roll = vel_roll;
    _payload.vel_pitch = vel_pitch;
    _payload.vel_yaw = vel_yaw;
    _payload.acc_x = acc_x;
    _payload.acc_y = acc_y;
    _payload.acc_z = acc_z;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_motion_cue_extra_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_motion_cue_extra_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MOTION_CUE_EXTRA,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOTION_CUE_EXTRA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MOTION_CUE_EXTRA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_motion_cue_extra_decode(fmav_motion_cue_extra_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOTION_CUE_EXTRA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_motion_cue_extra_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_vel_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_vel_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_vel_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_acc_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_acc_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_motion_cue_extra_get_field_acc_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOTION_CUE_EXTRA  52504

#define mavlink_motion_cue_extra_t  fmav_motion_cue_extra_t

#define MAVLINK_MSG_ID_MOTION_CUE_EXTRA_LEN  28
#define MAVLINK_MSG_ID_MOTION_CUE_EXTRA_MIN_LEN  28
#define MAVLINK_MSG_ID_52504_LEN  28
#define MAVLINK_MSG_ID_52504_MIN_LEN  28

#define MAVLINK_MSG_ID_MOTION_CUE_EXTRA_CRC  177
#define MAVLINK_MSG_ID_52504_CRC  177




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_cue_extra_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_motion_cue_extra_pack(
        _msg, sysid, compid,
        time_boot_ms, vel_roll, vel_pitch, vel_yaw, acc_x, acc_y, acc_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_cue_extra_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_motion_cue_extra_t* _payload)
{
    return mavlink_msg_motion_cue_extra_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->vel_roll, _payload->vel_pitch, _payload->vel_yaw, _payload->acc_x, _payload->acc_y, _payload->acc_z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_motion_cue_extra_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float vel_roll, float vel_pitch, float vel_yaw, float acc_x, float acc_y, float acc_z)
{
    return fmav_msg_motion_cue_extra_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, vel_roll, vel_pitch, vel_yaw, acc_x, acc_y, acc_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_motion_cue_extra_decode(const mavlink_message_t* msg, mavlink_motion_cue_extra_t* payload)
{
    fmav_msg_motion_cue_extra_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOTION_CUE_EXTRA_H

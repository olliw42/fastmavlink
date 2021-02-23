//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_control_system_state_t {
    uint64_t time_usec;
    float x_acc;
    float y_acc;
    float z_acc;
    float x_vel;
    float y_vel;
    float z_vel;
    float x_pos;
    float y_pos;
    float z_pos;
    float airspeed;
    float vel_variance[3];
    float pos_variance[3];
    float q[4];
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
}) fmav_control_system_state_t;


#define FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE  146


#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MIN  100
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX  100
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN  100
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA  103

#define FASTMAVLINK_MSG_ID_146_LEN_MIN  100
#define FASTMAVLINK_MSG_ID_146_LEN_MAX  100
#define FASTMAVLINK_MSG_ID_146_LEN  100
#define FASTMAVLINK_MSG_ID_146_CRCEXTRA  103

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_LEN  3
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_LEN  3
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_FLAGS  0
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_control_system_state_t* _payload = (fmav_control_system_state_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->x_acc = x_acc;
    _payload->y_acc = y_acc;
    _payload->z_acc = z_acc;
    _payload->x_vel = x_vel;
    _payload->y_vel = y_vel;
    _payload->z_vel = z_vel;
    _payload->x_pos = x_pos;
    _payload->y_pos = y_pos;
    _payload->z_pos = z_pos;
    _payload->airspeed = airspeed;
    _payload->roll_rate = roll_rate;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    memcpy(&(_payload->vel_variance), vel_variance, sizeof(float)*3);
    memcpy(&(_payload->pos_variance), pos_variance, sizeof(float)*3);
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_system_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_system_state_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->x_acc, _payload->y_acc, _payload->z_acc, _payload->x_vel, _payload->y_vel, _payload->z_vel, _payload->x_pos, _payload->y_pos, _payload->z_pos, _payload->airspeed, _payload->vel_variance, _payload->pos_variance, _payload->q, _payload->roll_rate, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_control_system_state_t* _payload = (fmav_control_system_state_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x_acc = x_acc;
    _payload->y_acc = y_acc;
    _payload->z_acc = z_acc;
    _payload->x_vel = x_vel;
    _payload->y_vel = y_vel;
    _payload->z_vel = z_vel;
    _payload->x_pos = x_pos;
    _payload->y_pos = y_pos;
    _payload->z_pos = z_pos;
    _payload->airspeed = airspeed;
    _payload->roll_rate = roll_rate;
    _payload->pitch_rate = pitch_rate;
    _payload->yaw_rate = yaw_rate;
    memcpy(&(_payload->vel_variance), vel_variance, sizeof(float)*3);
    memcpy(&(_payload->pos_variance), pos_variance, sizeof(float)*3);
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CONTROL_SYSTEM_STATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_control_system_state_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_control_system_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_control_system_state_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->x_acc, _payload->y_acc, _payload->z_acc, _payload->x_vel, _payload->y_vel, _payload->z_vel, _payload->x_pos, _payload->y_pos, _payload->z_pos, _payload->airspeed, _payload->vel_variance, _payload->pos_variance, _payload->q, _payload->roll_rate, _payload->pitch_rate, _payload->yaw_rate,
        _status);
}


//----------------------------------------
//-- Message CONTROL_SYSTEM_STATE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_control_system_state_decode(fmav_control_system_state_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE  146

#define mavlink_control_system_state_t  fmav_control_system_state_t

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_LEN  100
#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_MIN_LEN  100
#define MAVLINK_MSG_ID_146_LEN  100
#define MAVLINK_MSG_ID_146_MIN_LEN  100

#define MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE_CRC  103
#define MAVLINK_MSG_ID_146_CRC  103

#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_VEL_VARIANCE_LEN 3
#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_POS_VARIANCE_LEN 3
#define MAVLINK_MSG_CONTROL_SYSTEM_STATE_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_system_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_control_system_state_pack(
        msg, sysid, compid,
        time_usec, x_acc, y_acc, z_acc, x_vel, y_vel, z_vel, x_pos, y_pos, z_pos, airspeed, vel_variance, pos_variance, q, roll_rate, pitch_rate, yaw_rate,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_control_system_state_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float x_acc, float y_acc, float z_acc, float x_vel, float y_vel, float z_vel, float x_pos, float y_pos, float z_pos, float airspeed, const float* vel_variance, const float* pos_variance, const float* q, float roll_rate, float pitch_rate, float yaw_rate)
{
    return fmav_msg_control_system_state_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, x_acc, y_acc, z_acc, x_vel, y_vel, z_vel, x_pos, y_pos, z_pos, airspeed, vel_variance, pos_variance, q, roll_rate, pitch_rate, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_control_system_state_decode(const mavlink_message_t* msg, mavlink_control_system_state_t* payload)
{
    fmav_msg_control_system_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CONTROL_SYSTEM_STATE_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_STATE_QUATERNION_H
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_H


//----------------------------------------
//-- Message HIL_STATE_QUATERNION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_state_quaternion_t {
    uint64_t time_usec;
    float attitude_quaternion[4];
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t ind_airspeed;
    uint16_t true_airspeed;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
}) fmav_hil_state_quaternion_t;


#define FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION  115


#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MIN  64
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX  64
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN  64
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_CRCEXTRA  4

#define FASTMAVLINK_MSG_ID_115_LEN_MIN  64
#define FASTMAVLINK_MSG_ID_115_LEN_MAX  64
#define FASTMAVLINK_MSG_ID_115_LEN  64
#define FASTMAVLINK_MSG_ID_115_CRCEXTRA  4

#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_FIELD_ATTITUDE_QUATERNION_LEN  4

#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_FLAGS  0
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_QUATERNION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIL_STATE_QUATERNION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_quaternion_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_quaternion_t* _payload = (fmav_hil_state_quaternion_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->ind_airspeed = ind_airspeed;
    _payload->true_airspeed = true_airspeed;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    memcpy(&(_payload->attitude_quaternion), attitude_quaternion, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_STATE_QUATERNION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_quaternion_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_quaternion_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->attitude_quaternion, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->ind_airspeed, _payload->true_airspeed, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_quaternion_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_quaternion_t* _payload = (fmav_hil_state_quaternion_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->ind_airspeed = ind_airspeed;
    _payload->true_airspeed = true_airspeed;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    memcpy(&(_payload->attitude_quaternion), attitude_quaternion, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE_QUATERNION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_QUATERNION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_quaternion_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_quaternion_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->attitude_quaternion, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->ind_airspeed, _payload->true_airspeed, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


//----------------------------------------
//-- Message HIL_STATE_QUATERNION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_state_quaternion_decode(fmav_hil_state_quaternion_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_STATE_QUATERNION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION  115

#define mavlink_hil_state_quaternion_t  fmav_hil_state_quaternion_t

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN  64
#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN  64
#define MAVLINK_MSG_ID_115_LEN  64
#define MAVLINK_MSG_ID_115_MIN_LEN  64

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC  4
#define MAVLINK_MSG_ID_115_CRC  4

#define MAVLINK_MSG_HIL_STATE_QUATERNION_FIELD_ATTITUDE_QUATERNION_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_quaternion_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, const float* attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_state_quaternion_pack(
        msg, sysid, compid,
        time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_quaternion_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return fmav_msg_hil_state_quaternion_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_state_quaternion_decode(const mavlink_message_t* msg, mavlink_hil_state_quaternion_t* payload)
{
    fmav_msg_hil_state_quaternion_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_STATE_QUATERNION_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_STATE_H
#define FASTMAVLINK_MSG_HIL_STATE_H


//----------------------------------------
//-- Message HIL_STATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_state_t {
    uint64_t time_usec;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
}) fmav_hil_state_t;


#define FASTMAVLINK_MSG_ID_HIL_STATE  90


#define FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MIN  56
#define FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX  56
#define FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN  56
#define FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA  183

#define FASTMAVLINK_MSG_ID_90_LEN_MIN  56
#define FASTMAVLINK_MSG_ID_90_LEN_MAX  56
#define FASTMAVLINK_MSG_ID_90_LEN  56
#define FASTMAVLINK_MSG_ID_90_CRCEXTRA  183



#define FASTMAVLINK_MSG_HIL_STATE_FLAGS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIL_STATE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_STATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_STATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


//----------------------------------------
//-- Message HIL_STATE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_state_decode(fmav_hil_state_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_STATE  90

#define mavlink_hil_state_t  fmav_hil_state_t

#define MAVLINK_MSG_ID_HIL_STATE_LEN  56
#define MAVLINK_MSG_ID_HIL_STATE_MIN_LEN  56
#define MAVLINK_MSG_ID_90_LEN  56
#define MAVLINK_MSG_ID_90_MIN_LEN  56

#define MAVLINK_MSG_ID_HIL_STATE_CRC  183
#define MAVLINK_MSG_ID_90_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_state_pack(
        msg, sysid, compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* payload)
{
    fmav_msg_hil_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_STATE_H

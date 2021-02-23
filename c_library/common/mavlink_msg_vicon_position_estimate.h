//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_H
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_H


//----------------------------------------
//-- Message VICON_POSITION_ESTIMATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vicon_position_estimate_t {
    uint64_t usec;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float covariance[21];
}) fmav_vicon_position_estimate_t;


#define FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE  104


#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX  116
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN  116
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_CRCEXTRA  56

#define FASTMAVLINK_MSG_ID_104_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_104_LEN_MAX  116
#define FASTMAVLINK_MSG_ID_104_LEN  116
#define FASTMAVLINK_MSG_ID_104_CRCEXTRA  56

#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN  21

#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_FLAGS  0
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message VICON_POSITION_ESTIMATE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vicon_position_estimate_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance,
    fmav_status_t* _status)
{
    fmav_vicon_position_estimate_t* _payload = (fmav_vicon_position_estimate_t*)msg->payload;

    _payload->usec = usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vicon_position_estimate_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vicon_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vicon_position_estimate_pack(
        msg, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vicon_position_estimate_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance,
    fmav_status_t* _status)
{
    fmav_vicon_position_estimate_t* _payload = (fmav_vicon_position_estimate_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->usec = usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VICON_POSITION_ESTIMATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vicon_position_estimate_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vicon_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vicon_position_estimate_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance,
        _status);
}


//----------------------------------------
//-- Message VICON_POSITION_ESTIMATE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vicon_position_estimate_decode(fmav_vicon_position_estimate_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE  104

#define mavlink_vicon_position_estimate_t  fmav_vicon_position_estimate_t

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN  116
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_MIN_LEN  32
#define MAVLINK_MSG_ID_104_LEN  116
#define MAVLINK_MSG_ID_104_MIN_LEN  32

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC  56
#define MAVLINK_MSG_ID_104_CRC  56

#define MAVLINK_MSG_VICON_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vicon_position_estimate_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vicon_position_estimate_pack(
        msg, sysid, compid,
        usec, x, y, z, roll, pitch, yaw, covariance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vicon_position_estimate_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance)
{
    return fmav_msg_vicon_position_estimate_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        usec, x, y, z, roll, pitch, yaw, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vicon_position_estimate_decode(const mavlink_message_t* msg, mavlink_vicon_position_estimate_t* payload)
{
    fmav_msg_vicon_position_estimate_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VICON_POSITION_ESTIMATE_H

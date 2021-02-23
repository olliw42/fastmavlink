//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H


//----------------------------------------
//-- Message ATTITUDE_QUATERNION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_attitude_quaternion_t {
    uint32_t time_boot_ms;
    float q1;
    float q2;
    float q3;
    float q4;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float repr_offset_q[4];
}) fmav_attitude_quaternion_t;


#define FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION  31


#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX  48
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN  48
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA  246

#define FASTMAVLINK_MSG_ID_31_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_31_LEN_MAX  48
#define FASTMAVLINK_MSG_ID_31_LEN  48
#define FASTMAVLINK_MSG_ID_31_CRCEXTRA  246

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_LEN  4

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ATTITUDE_QUATERNION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_t* _payload = (fmav_attitude_quaternion_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->repr_offset_q), repr_offset_q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->repr_offset_q,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_t* _payload = (fmav_attitude_quaternion_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->repr_offset_q), repr_offset_q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->repr_offset_q,
        _status);
}


//----------------------------------------
//-- Message ATTITUDE_QUATERNION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_quaternion_decode(fmav_attitude_quaternion_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION  31

#define mavlink_attitude_quaternion_t  fmav_attitude_quaternion_t

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN  48
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN  32
#define MAVLINK_MSG_ID_31_LEN  48
#define MAVLINK_MSG_ID_31_MIN_LEN  32

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_CRC  246
#define MAVLINK_MSG_ID_31_CRC  246

#define MAVLINK_MSG_ATTITUDE_QUATERNION_FIELD_REPR_OFFSET_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_attitude_quaternion_pack(
        msg, sysid, compid,
        time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed, repr_offset_q,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float* repr_offset_q)
{
    return fmav_msg_attitude_quaternion_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed, repr_offset_q,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_attitude_quaternion_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_t* payload)
{
    fmav_msg_attitude_quaternion_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATTITUDE_QUATERNION_H

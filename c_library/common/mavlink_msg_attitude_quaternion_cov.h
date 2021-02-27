//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_H
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_H


//----------------------------------------
//-- Message ATTITUDE_QUATERNION_COV
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_attitude_quaternion_cov_t {
    uint64_t time_usec;
    float q[4];
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float covariance[9];
}) fmav_attitude_quaternion_cov_t;


#define FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV  61


#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MIN  72
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX  72
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN  72
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA  167

#define FASTMAVLINK_MSG_ID_61_LEN_MIN  72
#define FASTMAVLINK_MSG_ID_61_LEN_MAX  72
#define FASTMAVLINK_MSG_ID_61_LEN  72
#define FASTMAVLINK_MSG_ID_61_CRCEXTRA  167

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_LEN  4
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_LEN  9

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_61_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_61_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message ATTITUDE_QUATERNION_COV packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float rollspeed, float pitchspeed, float yawspeed, const float* covariance,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_cov_t* _payload = (fmav_attitude_quaternion_cov_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*9);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_cov_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->q, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float rollspeed, float pitchspeed, float yawspeed, const float* covariance,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_cov_t* _payload = (fmav_attitude_quaternion_cov_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*9);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_quaternion_cov_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->q, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->covariance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float rollspeed, float pitchspeed, float yawspeed, const float* covariance,
    fmav_status_t* _status)
{
    fmav_attitude_quaternion_cov_t _payload;

    _payload.time_usec = time_usec;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;
    memcpy(&(_payload.q), q, sizeof(float)*4);
    memcpy(&(_payload.covariance), covariance, sizeof(float)*9);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_quaternion_cov_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_quaternion_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ATTITUDE_QUATERNION_COV unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_quaternion_cov_decode(fmav_attitude_quaternion_cov_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV  61

#define mavlink_attitude_quaternion_cov_t  fmav_attitude_quaternion_cov_t

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_LEN  72
#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_MIN_LEN  72
#define MAVLINK_MSG_ID_61_LEN  72
#define MAVLINK_MSG_ID_61_MIN_LEN  72

#define MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV_CRC  167
#define MAVLINK_MSG_ID_61_CRC  167

#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_LEN 9


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, const float* q, float rollspeed, float pitchspeed, float yawspeed, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_attitude_quaternion_cov_pack(
        msg, sysid, compid,
        time_usec, q, rollspeed, pitchspeed, yawspeed, covariance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_quaternion_cov_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float rollspeed, float pitchspeed, float yawspeed, const float* covariance)
{
    return fmav_msg_attitude_quaternion_cov_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, q, rollspeed, pitchspeed, yawspeed, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_attitude_quaternion_cov_decode(const mavlink_message_t* msg, mavlink_attitude_quaternion_cov_t* payload)
{
    fmav_msg_attitude_quaternion_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_H

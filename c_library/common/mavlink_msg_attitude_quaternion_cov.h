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

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX  72
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA  167

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FRAME_LEN_MAX  97

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_NUM  9 // number of elements in array
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_LEN  36 // length of array = number of bytes

#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_OFS  8
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_ROLLSPEED_OFS  24
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_PITCHSPEED_OFS  28
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_YAWSPEED_OFS  32
#define FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_OFS  36


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
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ATTITUDE_QUATERNION_COV unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_attitude_quaternion_cov_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_attitude_quaternion_cov_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_quaternion_cov_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_quaternion_cov_decode(fmav_attitude_quaternion_cov_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_attitude_quaternion_cov_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_cov_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_cov_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_cov_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_attitude_quaternion_cov_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_cov_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_attitude_quaternion_cov_get_field_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_attitude_quaternion_cov_get_field_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ATTITUDE_QUATERNION_COV_FIELD_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[36]))[index];
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_H
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_H


//----------------------------------------
//-- Message LOCAL_POSITION_NED_COV
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_local_position_ned_cov_t {
    uint64_t time_usec;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float ax;
    float ay;
    float az;
    float covariance[45];
    uint8_t estimator_type;
}) fmav_local_position_ned_cov_t;


#define FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV  64


#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MIN  225
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX  225
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN  225
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA  191

#define FASTMAVLINK_MSG_ID_64_LEN_MIN  225
#define FASTMAVLINK_MSG_ID_64_LEN_MAX  225
#define FASTMAVLINK_MSG_ID_64_LEN  225
#define FASTMAVLINK_MSG_ID_64_CRCEXTRA  191

#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_FIELD_COVARIANCE_LEN  45

#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_FLAGS  0
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_64_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_64_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message LOCAL_POSITION_NED_COV packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, const float* covariance,
    fmav_status_t* _status)
{
    fmav_local_position_ned_cov_t* _payload = (fmav_local_position_ned_cov_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->ax = ax;
    _payload->ay = ay;
    _payload->az = az;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*45);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_local_position_ned_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_local_position_ned_cov_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->ax, _payload->ay, _payload->az, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, const float* covariance,
    fmav_status_t* _status)
{
    fmav_local_position_ned_cov_t* _payload = (fmav_local_position_ned_cov_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->ax = ax;
    _payload->ay = ay;
    _payload->az = az;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*45);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_local_position_ned_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_local_position_ned_cov_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->ax, _payload->ay, _payload->az, _payload->covariance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, const float* covariance,
    fmav_status_t* _status)
{
    fmav_local_position_ned_cov_t _payload;

    _payload.time_usec = time_usec;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.ax = ax;
    _payload.ay = ay;
    _payload.az = az;
    _payload.estimator_type = estimator_type;
    memcpy(&(_payload.covariance), covariance, sizeof(float)*45);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_cov_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_local_position_ned_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_COV,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOCAL_POSITION_NED_COV unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_local_position_ned_cov_decode(fmav_local_position_ned_cov_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV  64

#define mavlink_local_position_ned_cov_t  fmav_local_position_ned_cov_t

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_LEN  225
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_MIN_LEN  225
#define MAVLINK_MSG_ID_64_LEN  225
#define MAVLINK_MSG_ID_64_MIN_LEN  225

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_CRC  191
#define MAVLINK_MSG_ID_64_CRC  191

#define MAVLINK_MSG_LOCAL_POSITION_NED_COV_FIELD_COVARIANCE_LEN 45


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_local_position_ned_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t estimator_type, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_local_position_ned_cov_pack(
        msg, sysid, compid,
        time_usec, estimator_type, x, y, z, vx, vy, vz, ax, ay, az, covariance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_local_position_ned_cov_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, const float* covariance)
{
    return fmav_msg_local_position_ned_cov_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, estimator_type, x, y, z, vx, vy, vz, ax, ay, az, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_local_position_ned_cov_decode(const mavlink_message_t* msg, mavlink_local_position_ned_cov_t* payload)
{
    fmav_msg_local_position_ned_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOCAL_POSITION_NED_COV_H

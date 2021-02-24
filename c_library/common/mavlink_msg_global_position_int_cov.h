//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_global_position_int_cov_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    float vx;
    float vy;
    float vz;
    float covariance[36];
    uint8_t estimator_type;
}) fmav_global_position_int_cov_t;


#define FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV  63


#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MIN  181
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX  181
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN  181
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA  119

#define FASTMAVLINK_MSG_ID_63_LEN_MIN  181
#define FASTMAVLINK_MSG_ID_63_LEN_MAX  181
#define FASTMAVLINK_MSG_ID_63_LEN  181
#define FASTMAVLINK_MSG_ID_63_CRCEXTRA  119

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_LEN  36

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_FLAGS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance,
    fmav_status_t* _status)
{
    fmav_global_position_int_cov_t* _payload = (fmav_global_position_int_cov_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*36);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_cov_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance,
    fmav_status_t* _status)
{
    fmav_global_position_int_cov_t* _payload = (fmav_global_position_int_cov_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*36);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_cov_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_cov_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->estimator_type, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->covariance,
        _status);
}


//----------------------------------------
//-- Message GLOBAL_POSITION_INT_COV unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_position_int_cov_decode(fmav_global_position_int_cov_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV  63

#define mavlink_global_position_int_cov_t  fmav_global_position_int_cov_t

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN  181
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN  181
#define MAVLINK_MSG_ID_63_LEN  181
#define MAVLINK_MSG_ID_63_MIN_LEN  181

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC  119
#define MAVLINK_MSG_ID_63_CRC  119

#define MAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_LEN 36


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_global_position_int_cov_pack(
        msg, sysid, compid,
        time_usec, estimator_type, lat, lon, alt, relative_alt, vx, vy, vz, covariance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_cov_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float* covariance)
{
    return fmav_msg_global_position_int_cov_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, estimator_type, lat, lon, alt, relative_alt, vx, vy, vz, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_global_position_int_cov_decode(const mavlink_message_t* msg, mavlink_global_position_int_cov_t* payload)
{
    fmav_msg_global_position_int_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GLOBAL_POSITION_INT_COV_H

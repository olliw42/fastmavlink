//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FOLLOW_TARGET_H
#define FASTMAVLINK_MSG_FOLLOW_TARGET_H


//----------------------------------------
//-- Message FOLLOW_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_follow_target_t {
    uint64_t timestamp;
    uint64_t custom_state;
    int32_t lat;
    int32_t lon;
    float alt;
    float vel[3];
    float acc[3];
    float attitude_q[4];
    float rates[3];
    float position_cov[3];
    uint8_t est_capabilities;
}) fmav_follow_target_t;


#define FASTMAVLINK_MSG_ID_FOLLOW_TARGET  144


#define FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MIN  93
#define FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX  93
#define FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN  93
#define FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA  127

#define FASTMAVLINK_MSG_ID_144_LEN_MIN  93
#define FASTMAVLINK_MSG_ID_144_LEN_MAX  93
#define FASTMAVLINK_MSG_ID_144_LEN  93
#define FASTMAVLINK_MSG_ID_144_CRCEXTRA  127

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_LEN  3
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_LEN  3
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_LEN  4
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_LEN  3
#define FASTMAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_LEN  3

#define FASTMAVLINK_MSG_FOLLOW_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_FOLLOW_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FOLLOW_TARGET_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message FOLLOW_TARGET packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state,
    fmav_status_t* _status)
{
    fmav_follow_target_t* _payload = (fmav_follow_target_t*)msg->payload;

    _payload->timestamp = timestamp;
    _payload->custom_state = custom_state;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->est_capabilities = est_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->attitude_q), attitude_q, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_cov), position_cov, sizeof(float)*3);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_FOLLOW_TARGET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_follow_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_follow_target_pack(
        msg, sysid, compid,
        _payload->timestamp, _payload->est_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->attitude_q, _payload->rates, _payload->position_cov, _payload->custom_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state,
    fmav_status_t* _status)
{
    fmav_follow_target_t* _payload = (fmav_follow_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->custom_state = custom_state;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->est_capabilities = est_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->attitude_q), attitude_q, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_cov), position_cov, sizeof(float)*3);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FOLLOW_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FOLLOW_TARGET_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_follow_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_follow_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_follow_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->timestamp, _payload->est_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->attitude_q, _payload->rates, _payload->position_cov, _payload->custom_state,
        _status);
}


//----------------------------------------
//-- Message FOLLOW_TARGET unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_follow_target_decode(fmav_follow_target_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_FOLLOW_TARGET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FOLLOW_TARGET  144

#define mavlink_follow_target_t  fmav_follow_target_t

#define MAVLINK_MSG_ID_FOLLOW_TARGET_LEN  93
#define MAVLINK_MSG_ID_FOLLOW_TARGET_MIN_LEN  93
#define MAVLINK_MSG_ID_144_LEN  93
#define MAVLINK_MSG_ID_144_MIN_LEN  93

#define MAVLINK_MSG_ID_FOLLOW_TARGET_CRC  127
#define MAVLINK_MSG_ID_144_CRC  127

#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_VEL_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_ACC_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_ATTITUDE_Q_LEN 4
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_RATES_LEN 3
#define MAVLINK_MSG_FOLLOW_TARGET_FIELD_POSITION_COV_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_follow_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_follow_target_pack(
        msg, sysid, compid,
        timestamp, est_capabilities, lat, lon, alt, vel, acc, attitude_q, rates, position_cov, custom_state,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_follow_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t est_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* attitude_q, const float* rates, const float* position_cov, uint64_t custom_state)
{
    return fmav_msg_follow_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        timestamp, est_capabilities, lat, lon, alt, vel, acc, attitude_q, rates, position_cov, custom_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_follow_target_decode(const mavlink_message_t* msg, mavlink_follow_target_t* payload)
{
    fmav_msg_follow_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FOLLOW_TARGET_H

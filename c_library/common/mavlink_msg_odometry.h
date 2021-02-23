//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ODOMETRY_H
#define FASTMAVLINK_MSG_ODOMETRY_H


//----------------------------------------
//-- Message ODOMETRY
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_odometry_t {
    uint64_t time_usec;
    float x;
    float y;
    float z;
    float q[4];
    float vx;
    float vy;
    float vz;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float pose_covariance[21];
    float velocity_covariance[21];
    uint8_t frame_id;
    uint8_t child_frame_id;
    uint8_t reset_counter;
    uint8_t estimator_type;
}) fmav_odometry_t;


#define FASTMAVLINK_MSG_ID_ODOMETRY  331


#define FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MIN  230
#define FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX  232
#define FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN  232
#define FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA  91

#define FASTMAVLINK_MSG_ID_331_LEN_MIN  230
#define FASTMAVLINK_MSG_ID_331_LEN_MAX  232
#define FASTMAVLINK_MSG_ID_331_LEN  232
#define FASTMAVLINK_MSG_ID_331_CRCEXTRA  91

#define FASTMAVLINK_MSG_ODOMETRY_FIELD_Q_LEN  4
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN  21
#define FASTMAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN  21

#define FASTMAVLINK_MSG_ODOMETRY_FLAGS  0
#define FASTMAVLINK_MSG_ODOMETRY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ODOMETRY_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ODOMETRY packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type,
    fmav_status_t* _status)
{
    fmav_odometry_t* _payload = (fmav_odometry_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->frame_id = frame_id;
    _payload->child_frame_id = child_frame_id;
    _payload->reset_counter = reset_counter;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->pose_covariance), pose_covariance, sizeof(float)*21);
    memcpy(&(_payload->velocity_covariance), velocity_covariance, sizeof(float)*21);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ODOMETRY;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_odometry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_odometry_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->frame_id, _payload->child_frame_id, _payload->x, _payload->y, _payload->z, _payload->q, _payload->vx, _payload->vy, _payload->vz, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->pose_covariance, _payload->velocity_covariance, _payload->reset_counter, _payload->estimator_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type,
    fmav_status_t* _status)
{
    fmav_odometry_t* _payload = (fmav_odometry_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->frame_id = frame_id;
    _payload->child_frame_id = child_frame_id;
    _payload->reset_counter = reset_counter;
    _payload->estimator_type = estimator_type;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->pose_covariance), pose_covariance, sizeof(float)*21);
    memcpy(&(_payload->velocity_covariance), velocity_covariance, sizeof(float)*21);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ODOMETRY;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ODOMETRY >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ODOMETRY >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ODOMETRY_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_odometry_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_odometry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_odometry_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->frame_id, _payload->child_frame_id, _payload->x, _payload->y, _payload->z, _payload->q, _payload->vx, _payload->vy, _payload->vz, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->pose_covariance, _payload->velocity_covariance, _payload->reset_counter, _payload->estimator_type,
        _status);
}


//----------------------------------------
//-- Message ODOMETRY unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_odometry_decode(fmav_odometry_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ODOMETRY_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ODOMETRY  331

#define mavlink_odometry_t  fmav_odometry_t

#define MAVLINK_MSG_ID_ODOMETRY_LEN  232
#define MAVLINK_MSG_ID_ODOMETRY_MIN_LEN  230
#define MAVLINK_MSG_ID_331_LEN  232
#define MAVLINK_MSG_ID_331_MIN_LEN  230

#define MAVLINK_MSG_ID_ODOMETRY_CRC  91
#define MAVLINK_MSG_ID_331_CRC  91

#define MAVLINK_MSG_ODOMETRY_FIELD_Q_LEN 4
#define MAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN 21
#define MAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_odometry_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_odometry_pack(
        msg, sysid, compid,
        time_usec, frame_id, child_frame_id, x, y, z, q, vx, vy, vz, rollspeed, pitchspeed, yawspeed, pose_covariance, velocity_covariance, reset_counter, estimator_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_odometry_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, float x, float y, float z, const float* q, float vx, float vy, float vz, float rollspeed, float pitchspeed, float yawspeed, const float* pose_covariance, const float* velocity_covariance, uint8_t reset_counter, uint8_t estimator_type)
{
    return fmav_msg_odometry_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, frame_id, child_frame_id, x, y, z, q, vx, vy, vz, rollspeed, pitchspeed, yawspeed, pose_covariance, velocity_covariance, reset_counter, estimator_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_odometry_decode(const mavlink_message_t* msg, mavlink_odometry_t* payload)
{
    fmav_msg_odometry_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ODOMETRY_H

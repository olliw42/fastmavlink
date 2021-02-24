//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H


//----------------------------------------
//-- Message OBSTACLE_DISTANCE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_obstacle_distance_t {
    uint64_t time_usec;
    uint16_t distances[72];
    uint16_t min_distance;
    uint16_t max_distance;
    uint8_t sensor_type;
    uint8_t increment;
    float increment_f;
    float angle_offset;
    uint8_t frame;
}) fmav_obstacle_distance_t;


#define FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE  330


#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MIN  158
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX  167
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN  167
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA  23

#define FASTMAVLINK_MSG_ID_330_LEN_MIN  158
#define FASTMAVLINK_MSG_ID_330_LEN_MAX  167
#define FASTMAVLINK_MSG_ID_330_LEN  167
#define FASTMAVLINK_MSG_ID_330_CRCEXTRA  23

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN  72

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FLAGS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message OBSTACLE_DISTANCE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t* _payload = (fmav_obstacle_distance_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->sensor_type = sensor_type;
    _payload->increment = increment;
    _payload->increment_f = increment_f;
    _payload->angle_offset = angle_offset;
    _payload->frame = frame;
    memcpy(&(_payload->distances), distances, sizeof(uint16_t)*72);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->sensor_type, _payload->distances, _payload->increment, _payload->min_distance, _payload->max_distance, _payload->increment_f, _payload->angle_offset, _payload->frame,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t* _payload = (fmav_obstacle_distance_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->sensor_type = sensor_type;
    _payload->increment = increment;
    _payload->increment_f = increment_f;
    _payload->angle_offset = angle_offset;
    _payload->frame = frame;
    memcpy(&(_payload->distances), distances, sizeof(uint16_t)*72);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->sensor_type, _payload->distances, _payload->increment, _payload->min_distance, _payload->max_distance, _payload->increment_f, _payload->angle_offset, _payload->frame,
        _status);
}


//----------------------------------------
//-- Message OBSTACLE_DISTANCE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_obstacle_distance_decode(fmav_obstacle_distance_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE  330

#define mavlink_obstacle_distance_t  fmav_obstacle_distance_t

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN  167
#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN  158
#define MAVLINK_MSG_ID_330_LEN  167
#define MAVLINK_MSG_ID_330_MIN_LEN  158

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_CRC  23
#define MAVLINK_MSG_ID_330_CRC  23

#define MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN 72


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_obstacle_distance_pack(
        msg, sysid, compid,
        time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame)
{
    return fmav_msg_obstacle_distance_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_obstacle_distance_decode(const mavlink_message_t* msg, mavlink_obstacle_distance_t* payload)
{
    fmav_msg_obstacle_distance_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OBSTACLE_DISTANCE_H

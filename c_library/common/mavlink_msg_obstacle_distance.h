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

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX  167
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA  23

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FLAGS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FRAME_LEN_MAX  192

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_NUM  72 // number of elements in array
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN  144 // length of array = number of bytes

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_OFS  8
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_MIN_DISTANCE_OFS  152
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_MAX_DISTANCE_OFS  154
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_SENSOR_TYPE_OFS  156
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_INCREMENT_OFS  157
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_INCREMENT_F_OFS  158
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_ANGLE_OFFSET_OFS  162
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_FRAME_OFS  166


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_type, const uint16_t* distances, uint8_t increment, uint16_t min_distance, uint16_t max_distance, float increment_f, float angle_offset, uint8_t frame,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_t _payload;

    _payload.time_usec = time_usec;
    _payload.min_distance = min_distance;
    _payload.max_distance = max_distance;
    _payload.sensor_type = sensor_type;
    _payload.increment = increment;
    _payload.increment_f = increment_f;
    _payload.angle_offset = angle_offset;
    _payload.frame = frame;
    memcpy(&(_payload.distances), distances, sizeof(uint16_t)*72);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OBSTACLE_DISTANCE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_obstacle_distance_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_obstacle_distance_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_obstacle_distance_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_obstacle_distance_decode(fmav_obstacle_distance_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_obstacle_distance_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_min_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[152]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_max_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[154]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_sensor_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[156]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_increment(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[157]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_get_field_increment_f(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[158]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_get_field_angle_offset(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[162]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[166]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_obstacle_distance_get_field_distances_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_get_field_distances(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_NUM) return 0;
    return ((uint16_t*)&(msg->payload[8]))[index];
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

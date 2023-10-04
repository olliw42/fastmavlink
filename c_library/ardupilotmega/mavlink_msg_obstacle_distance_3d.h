//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_H
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_H


//----------------------------------------
//-- Message OBSTACLE_DISTANCE_3D
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_obstacle_distance_3d_t {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float min_distance;
    float max_distance;
    uint16_t obstacle_id;
    uint8_t sensor_type;
    uint8_t frame;
}) fmav_obstacle_distance_3d_t;


#define FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D  11037

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA  130

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FLAGS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_X_OFS  4
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_Y_OFS  8
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_Z_OFS  12
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_MIN_DISTANCE_OFS  16
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_MAX_DISTANCE_OFS  20
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_OBSTACLE_ID_OFS  24
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_SENSOR_TYPE_OFS  26
#define FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_FIELD_FRAME_OFS  27


//----------------------------------------
//-- Message OBSTACLE_DISTANCE_3D pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_3d_t* _payload = (fmav_obstacle_distance_3d_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->obstacle_id = obstacle_id;
    _payload->sensor_type = sensor_type;
    _payload->frame = frame;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_3d_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_3d_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->sensor_type, _payload->frame, _payload->obstacle_id, _payload->x, _payload->y, _payload->z, _payload->min_distance, _payload->max_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_3d_t* _payload = (fmav_obstacle_distance_3d_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->obstacle_id = obstacle_id;
    _payload->sensor_type = sensor_type;
    _payload->frame = frame;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_3d_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_obstacle_distance_3d_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->sensor_type, _payload->frame, _payload->obstacle_id, _payload->x, _payload->y, _payload->z, _payload->min_distance, _payload->max_distance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance,
    fmav_status_t* _status)
{
    fmav_obstacle_distance_3d_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.min_distance = min_distance;
    _payload.max_distance = max_distance;
    _payload.obstacle_id = obstacle_id;
    _payload.sensor_type = sensor_type;
    _payload.frame = frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_obstacle_distance_3d_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OBSTACLE_DISTANCE_3D decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_obstacle_distance_3d_decode(fmav_obstacle_distance_3d_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_obstacle_distance_3d_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_3d_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_3d_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_3d_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_3d_get_field_min_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_obstacle_distance_3d_get_field_max_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_obstacle_distance_3d_get_field_obstacle_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_3d_get_field_sensor_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_obstacle_distance_3d_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D  11037

#define mavlink_obstacle_distance_3d_t  fmav_obstacle_distance_3d_t

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_LEN  28
#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN  28
#define MAVLINK_MSG_ID_11037_LEN  28
#define MAVLINK_MSG_ID_11037_MIN_LEN  28

#define MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_CRC  130
#define MAVLINK_MSG_ID_11037_CRC  130




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_3d_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_obstacle_distance_3d_pack(
        _msg, sysid, compid,
        time_boot_ms, sensor_type, frame, obstacle_id, x, y, z, min_distance, max_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_3d_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_obstacle_distance_3d_t* _payload)
{
    return mavlink_msg_obstacle_distance_3d_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->sensor_type, _payload->frame, _payload->obstacle_id, _payload->x, _payload->y, _payload->z, _payload->min_distance, _payload->max_distance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_obstacle_distance_3d_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t sensor_type, uint8_t frame, uint16_t obstacle_id, float x, float y, float z, float min_distance, float max_distance)
{
    return fmav_msg_obstacle_distance_3d_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, sensor_type, frame, obstacle_id, x, y, z, min_distance, max_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_obstacle_distance_3d_decode(const mavlink_message_t* msg, mavlink_obstacle_distance_3d_t* payload)
{
    fmav_msg_obstacle_distance_3d_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OBSTACLE_DISTANCE_3D_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DISTANCE_SENSOR_H
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_H


//----------------------------------------
//-- Message DISTANCE_SENSOR
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_distance_sensor_t {
    uint32_t time_boot_ms;
    uint16_t min_distance;
    uint16_t max_distance;
    uint16_t current_distance;
    uint8_t type;
    uint8_t id;
    uint8_t orientation;
    uint8_t covariance;
    float horizontal_fov;
    float vertical_fov;
    float quaternion[4];
    uint8_t signal_quality;
}) fmav_distance_sensor_t;


#define FASTMAVLINK_MSG_ID_DISTANCE_SENSOR  132


#define FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX  39
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN  39
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA  85

#define FASTMAVLINK_MSG_ID_132_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_132_LEN_MAX  39
#define FASTMAVLINK_MSG_ID_132_LEN  39
#define FASTMAVLINK_MSG_ID_132_CRCEXTRA  85

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_LEN  4

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FLAGS  0
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message DISTANCE_SENSOR packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance, float horizontal_fov, float vertical_fov, const float* quaternion, uint8_t signal_quality,
    fmav_status_t* _status)
{
    fmav_distance_sensor_t* _payload = (fmav_distance_sensor_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->current_distance = current_distance;
    _payload->type = type;
    _payload->id = id;
    _payload->orientation = orientation;
    _payload->covariance = covariance;
    _payload->horizontal_fov = horizontal_fov;
    _payload->vertical_fov = vertical_fov;
    _payload->signal_quality = signal_quality;
    memcpy(&(_payload->quaternion), quaternion, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DISTANCE_SENSOR;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_distance_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_distance_sensor_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->min_distance, _payload->max_distance, _payload->current_distance, _payload->type, _payload->id, _payload->orientation, _payload->covariance, _payload->horizontal_fov, _payload->vertical_fov, _payload->quaternion, _payload->signal_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance, float horizontal_fov, float vertical_fov, const float* quaternion, uint8_t signal_quality,
    fmav_status_t* _status)
{
    fmav_distance_sensor_t* _payload = (fmav_distance_sensor_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->min_distance = min_distance;
    _payload->max_distance = max_distance;
    _payload->current_distance = current_distance;
    _payload->type = type;
    _payload->id = id;
    _payload->orientation = orientation;
    _payload->covariance = covariance;
    _payload->horizontal_fov = horizontal_fov;
    _payload->vertical_fov = vertical_fov;
    _payload->signal_quality = signal_quality;
    memcpy(&(_payload->quaternion), quaternion, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DISTANCE_SENSOR;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DISTANCE_SENSOR >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DISTANCE_SENSOR >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_distance_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_distance_sensor_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->min_distance, _payload->max_distance, _payload->current_distance, _payload->type, _payload->id, _payload->orientation, _payload->covariance, _payload->horizontal_fov, _payload->vertical_fov, _payload->quaternion, _payload->signal_quality,
        _status);
}


//----------------------------------------
//-- Message DISTANCE_SENSOR unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_distance_sensor_decode(fmav_distance_sensor_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DISTANCE_SENSOR  132

#define mavlink_distance_sensor_t  fmav_distance_sensor_t

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN  39
#define MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN  14
#define MAVLINK_MSG_ID_132_LEN  39
#define MAVLINK_MSG_ID_132_MIN_LEN  14

#define MAVLINK_MSG_ID_DISTANCE_SENSOR_CRC  85
#define MAVLINK_MSG_ID_132_CRC  85

#define MAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_distance_sensor_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance, float horizontal_fov, float vertical_fov, const float* quaternion, uint8_t signal_quality)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_distance_sensor_pack(
        msg, sysid, compid,
        time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance, horizontal_fov, vertical_fov, quaternion, signal_quality,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_distance_sensor_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance, float horizontal_fov, float vertical_fov, const float* quaternion, uint8_t signal_quality)
{
    return fmav_msg_distance_sensor_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance, horizontal_fov, vertical_fov, quaternion, signal_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_distance_sensor_decode(const mavlink_message_t* msg, mavlink_distance_sensor_t* payload)
{
    fmav_msg_distance_sensor_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DISTANCE_SENSOR_H

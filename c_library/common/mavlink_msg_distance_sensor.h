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

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX  39
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA  85

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FLAGS  0
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FRAME_LEN_MAX  64

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_MIN_DISTANCE_OFS  4
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_MAX_DISTANCE_OFS  6
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_CURRENT_DISTANCE_OFS  8
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_TYPE_OFS  10
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_ID_OFS  11
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_ORIENTATION_OFS  12
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_COVARIANCE_OFS  13
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_HORIZONTAL_FOV_OFS  14
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_VERTICAL_FOV_OFS  18
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_OFS  22
#define FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_SIGNAL_QUALITY_OFS  38


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance, float horizontal_fov, float vertical_fov, const float* quaternion, uint8_t signal_quality,
    fmav_status_t* _status)
{
    fmav_distance_sensor_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.min_distance = min_distance;
    _payload.max_distance = max_distance;
    _payload.current_distance = current_distance;
    _payload.type = type;
    _payload.id = id;
    _payload.orientation = orientation;
    _payload.covariance = covariance;
    _payload.horizontal_fov = horizontal_fov;
    _payload.vertical_fov = vertical_fov;
    _payload.signal_quality = signal_quality;
    memcpy(&(_payload.quaternion), quaternion, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DISTANCE_SENSOR,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_distance_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DISTANCE_SENSOR,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DISTANCE_SENSOR_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DISTANCE_SENSOR unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_distance_sensor_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_distance_sensor_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_distance_sensor_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_distance_sensor_decode(fmav_distance_sensor_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DISTANCE_SENSOR_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_distance_sensor_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_get_field_min_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_get_field_max_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_distance_sensor_get_field_current_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_distance_sensor_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_distance_sensor_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_distance_sensor_get_field_orientation(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_distance_sensor_get_field_covariance(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_distance_sensor_get_field_horizontal_fov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[14]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_distance_sensor_get_field_vertical_fov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[18]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_distance_sensor_get_field_signal_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_distance_sensor_get_field_quaternion_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[22]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_distance_sensor_get_field_quaternion(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DISTANCE_SENSOR_FIELD_QUATERNION_NUM) return 0;
    return ((float*)&(msg->payload[22]))[index];
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_H
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_H


//----------------------------------------
//-- Message CAMERA_THERMAL_RANGE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_thermal_range_t {
    uint32_t time_boot_ms;
    float max;
    float max_point_x;
    float max_point_y;
    float min;
    float min_point_x;
    float min_point_y;
    uint8_t stream_id;
    uint8_t camera_device_id;
}) fmav_camera_thermal_range_t;


#define FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE  277

#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX  30
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_CRCEXTRA  62

#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FRAME_LEN_MAX  55



#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MAX_OFS  4
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MAX_POINT_X_OFS  8
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MAX_POINT_Y_OFS  12
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MIN_OFS  16
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MIN_POINT_X_OFS  20
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_MIN_POINT_Y_OFS  24
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_STREAM_ID_OFS  28
#define FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_FIELD_CAMERA_DEVICE_ID_OFS  29


//----------------------------------------
//-- Message CAMERA_THERMAL_RANGE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y,
    fmav_status_t* _status)
{
    fmav_camera_thermal_range_t* _payload = (fmav_camera_thermal_range_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->max = max;
    _payload->max_point_x = max_point_x;
    _payload->max_point_y = max_point_y;
    _payload->min = min;
    _payload->min_point_x = min_point_x;
    _payload->min_point_y = min_point_y;
    _payload->stream_id = stream_id;
    _payload->camera_device_id = camera_device_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_thermal_range_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_thermal_range_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->stream_id, _payload->camera_device_id, _payload->max, _payload->max_point_x, _payload->max_point_y, _payload->min, _payload->min_point_x, _payload->min_point_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y,
    fmav_status_t* _status)
{
    fmav_camera_thermal_range_t* _payload = (fmav_camera_thermal_range_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->max = max;
    _payload->max_point_x = max_point_x;
    _payload->max_point_y = max_point_y;
    _payload->min = min;
    _payload->min_point_x = min_point_x;
    _payload->min_point_y = min_point_y;
    _payload->stream_id = stream_id;
    _payload->camera_device_id = camera_device_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_thermal_range_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_thermal_range_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->stream_id, _payload->camera_device_id, _payload->max, _payload->max_point_x, _payload->max_point_y, _payload->min, _payload->min_point_x, _payload->min_point_y,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y,
    fmav_status_t* _status)
{
    fmav_camera_thermal_range_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.max = max;
    _payload.max_point_x = max_point_x;
    _payload.max_point_y = max_point_y;
    _payload.min = min;
    _payload.min_point_x = min_point_x;
    _payload.min_point_y = min_point_y;
    _payload.stream_id = stream_id;
    _payload.camera_device_id = camera_device_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_thermal_range_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_thermal_range_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_THERMAL_RANGE,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_THERMAL_RANGE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_thermal_range_decode(fmav_camera_thermal_range_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_thermal_range_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_max_point_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_max_point_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_min_point_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_thermal_range_get_field_min_point_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_thermal_range_get_field_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_thermal_range_get_field_camera_device_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE  277

#define mavlink_camera_thermal_range_t  fmav_camera_thermal_range_t

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN  30
#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN  30
#define MAVLINK_MSG_ID_277_LEN  30
#define MAVLINK_MSG_ID_277_MIN_LEN  30

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC  62
#define MAVLINK_MSG_ID_277_CRC  62




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_thermal_range_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_thermal_range_pack(
        _msg, sysid, compid,
        time_boot_ms, stream_id, camera_device_id, max, max_point_x, max_point_y, min, min_point_x, min_point_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_thermal_range_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_thermal_range_t* _payload)
{
    return mavlink_msg_camera_thermal_range_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->stream_id, _payload->camera_device_id, _payload->max, _payload->max_point_x, _payload->max_point_y, _payload->min, _payload->min_point_x, _payload->min_point_y);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_thermal_range_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
    return fmav_msg_camera_thermal_range_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, stream_id, camera_device_id, max, max_point_x, max_point_y, min, min_point_x, min_point_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_thermal_range_decode(const mavlink_message_t* msg, mavlink_camera_thermal_range_t* payload)
{
    fmav_msg_camera_thermal_range_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_THERMAL_RANGE_H

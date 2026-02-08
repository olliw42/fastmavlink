//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EYE_TRACKING_DATA_H
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_H


//----------------------------------------
//-- Message EYE_TRACKING_DATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_eye_tracking_data_t {
    uint64_t time_usec;
    float gaze_origin_x;
    float gaze_origin_y;
    float gaze_origin_z;
    float gaze_direction_x;
    float gaze_direction_y;
    float gaze_direction_z;
    float video_gaze_x;
    float video_gaze_y;
    float surface_gaze_x;
    float surface_gaze_y;
    uint8_t sensor_id;
    uint8_t surface_id;
}) fmav_eye_tracking_data_t;


#define FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA  52505

#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX  50
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_CRCEXTRA  215

#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FLAGS  0
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FRAME_LEN_MAX  75



#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_ORIGIN_X_OFS  8
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_ORIGIN_Y_OFS  12
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_ORIGIN_Z_OFS  16
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_DIRECTION_X_OFS  20
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_DIRECTION_Y_OFS  24
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_GAZE_DIRECTION_Z_OFS  28
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_VIDEO_GAZE_X_OFS  32
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_VIDEO_GAZE_Y_OFS  36
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_SURFACE_GAZE_X_OFS  40
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_SURFACE_GAZE_Y_OFS  44
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_SENSOR_ID_OFS  48
#define FASTMAVLINK_MSG_EYE_TRACKING_DATA_FIELD_SURFACE_ID_OFS  49


//----------------------------------------
//-- Message EYE_TRACKING_DATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, float gaze_origin_x, float gaze_origin_y, float gaze_origin_z, float gaze_direction_x, float gaze_direction_y, float gaze_direction_z, float video_gaze_x, float video_gaze_y, uint8_t surface_id, float surface_gaze_x, float surface_gaze_y,
    fmav_status_t* _status)
{
    fmav_eye_tracking_data_t* _payload = (fmav_eye_tracking_data_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->gaze_origin_x = gaze_origin_x;
    _payload->gaze_origin_y = gaze_origin_y;
    _payload->gaze_origin_z = gaze_origin_z;
    _payload->gaze_direction_x = gaze_direction_x;
    _payload->gaze_direction_y = gaze_direction_y;
    _payload->gaze_direction_z = gaze_direction_z;
    _payload->video_gaze_x = video_gaze_x;
    _payload->video_gaze_y = video_gaze_y;
    _payload->surface_gaze_x = surface_gaze_x;
    _payload->surface_gaze_y = surface_gaze_y;
    _payload->sensor_id = sensor_id;
    _payload->surface_id = surface_id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_EYE_TRACKING_DATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_eye_tracking_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_eye_tracking_data_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->gaze_origin_x, _payload->gaze_origin_y, _payload->gaze_origin_z, _payload->gaze_direction_x, _payload->gaze_direction_y, _payload->gaze_direction_z, _payload->video_gaze_x, _payload->video_gaze_y, _payload->surface_id, _payload->surface_gaze_x, _payload->surface_gaze_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, float gaze_origin_x, float gaze_origin_y, float gaze_origin_z, float gaze_direction_x, float gaze_direction_y, float gaze_direction_z, float video_gaze_x, float video_gaze_y, uint8_t surface_id, float surface_gaze_x, float surface_gaze_y,
    fmav_status_t* _status)
{
    fmav_eye_tracking_data_t* _payload = (fmav_eye_tracking_data_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->gaze_origin_x = gaze_origin_x;
    _payload->gaze_origin_y = gaze_origin_y;
    _payload->gaze_origin_z = gaze_origin_z;
    _payload->gaze_direction_x = gaze_direction_x;
    _payload->gaze_direction_y = gaze_direction_y;
    _payload->gaze_direction_z = gaze_direction_z;
    _payload->video_gaze_x = video_gaze_x;
    _payload->video_gaze_y = video_gaze_y;
    _payload->surface_gaze_x = surface_gaze_x;
    _payload->surface_gaze_y = surface_gaze_y;
    _payload->sensor_id = sensor_id;
    _payload->surface_id = surface_id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_eye_tracking_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_eye_tracking_data_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->gaze_origin_x, _payload->gaze_origin_y, _payload->gaze_origin_z, _payload->gaze_direction_x, _payload->gaze_direction_y, _payload->gaze_direction_z, _payload->video_gaze_x, _payload->video_gaze_y, _payload->surface_id, _payload->surface_gaze_x, _payload->surface_gaze_y,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, float gaze_origin_x, float gaze_origin_y, float gaze_origin_z, float gaze_direction_x, float gaze_direction_y, float gaze_direction_z, float video_gaze_x, float video_gaze_y, uint8_t surface_id, float surface_gaze_x, float surface_gaze_y,
    fmav_status_t* _status)
{
    fmav_eye_tracking_data_t _payload;

    _payload.time_usec = time_usec;
    _payload.gaze_origin_x = gaze_origin_x;
    _payload.gaze_origin_y = gaze_origin_y;
    _payload.gaze_origin_z = gaze_origin_z;
    _payload.gaze_direction_x = gaze_direction_x;
    _payload.gaze_direction_y = gaze_direction_y;
    _payload.gaze_direction_z = gaze_direction_z;
    _payload.video_gaze_x = video_gaze_x;
    _payload.video_gaze_y = video_gaze_y;
    _payload.surface_gaze_x = surface_gaze_x;
    _payload.surface_gaze_y = surface_gaze_y;
    _payload.sensor_id = sensor_id;
    _payload.surface_id = surface_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_eye_tracking_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_eye_tracking_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_EYE_TRACKING_DATA,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EYE_TRACKING_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message EYE_TRACKING_DATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_eye_tracking_data_decode(fmav_eye_tracking_data_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_EYE_TRACKING_DATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_eye_tracking_data_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_origin_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_origin_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_origin_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_direction_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_direction_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_gaze_direction_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_video_gaze_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_video_gaze_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_surface_gaze_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_eye_tracking_data_get_field_surface_gaze_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_eye_tracking_data_get_field_sensor_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_eye_tracking_data_get_field_surface_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EYE_TRACKING_DATA  52505

#define mavlink_eye_tracking_data_t  fmav_eye_tracking_data_t

#define MAVLINK_MSG_ID_EYE_TRACKING_DATA_LEN  50
#define MAVLINK_MSG_ID_EYE_TRACKING_DATA_MIN_LEN  50
#define MAVLINK_MSG_ID_52505_LEN  50
#define MAVLINK_MSG_ID_52505_MIN_LEN  50

#define MAVLINK_MSG_ID_EYE_TRACKING_DATA_CRC  215
#define MAVLINK_MSG_ID_52505_CRC  215




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_eye_tracking_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t sensor_id, float gaze_origin_x, float gaze_origin_y, float gaze_origin_z, float gaze_direction_x, float gaze_direction_y, float gaze_direction_z, float video_gaze_x, float video_gaze_y, uint8_t surface_id, float surface_gaze_x, float surface_gaze_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_eye_tracking_data_pack(
        _msg, sysid, compid,
        time_usec, sensor_id, gaze_origin_x, gaze_origin_y, gaze_origin_z, gaze_direction_x, gaze_direction_y, gaze_direction_z, video_gaze_x, video_gaze_y, surface_id, surface_gaze_x, surface_gaze_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_eye_tracking_data_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_eye_tracking_data_t* _payload)
{
    return mavlink_msg_eye_tracking_data_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->sensor_id, _payload->gaze_origin_x, _payload->gaze_origin_y, _payload->gaze_origin_z, _payload->gaze_direction_x, _payload->gaze_direction_y, _payload->gaze_direction_z, _payload->video_gaze_x, _payload->video_gaze_y, _payload->surface_id, _payload->surface_gaze_x, _payload->surface_gaze_y);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_eye_tracking_data_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, float gaze_origin_x, float gaze_origin_y, float gaze_origin_z, float gaze_direction_x, float gaze_direction_y, float gaze_direction_z, float video_gaze_x, float video_gaze_y, uint8_t surface_id, float surface_gaze_x, float surface_gaze_y)
{
    return fmav_msg_eye_tracking_data_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, sensor_id, gaze_origin_x, gaze_origin_y, gaze_origin_z, gaze_direction_x, gaze_direction_y, gaze_direction_z, video_gaze_x, video_gaze_y, surface_id, surface_gaze_x, surface_gaze_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_eye_tracking_data_decode(const mavlink_message_t* msg, mavlink_eye_tracking_data_t* payload)
{
    fmav_msg_eye_tracking_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EYE_TRACKING_DATA_H

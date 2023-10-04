//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_FEEDBACK_H
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_H


//----------------------------------------
//-- Message CAMERA_FEEDBACK
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_feedback_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lng;
    float alt_msl;
    float alt_rel;
    float roll;
    float pitch;
    float yaw;
    float foc_len;
    uint16_t img_idx;
    uint8_t target_system;
    uint8_t cam_idx;
    uint8_t flags;
    uint16_t completed_captures;
}) fmav_camera_feedback_t;


#define FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK  180

#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX  47
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA  52

#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FLAGS  1
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_TARGET_SYSTEM_OFS  42
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FRAME_LEN_MAX  72



#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_LNG_OFS  12
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_ALT_MSL_OFS  16
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_ALT_REL_OFS  20
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_ROLL_OFS  24
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_PITCH_OFS  28
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_YAW_OFS  32
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_FOC_LEN_OFS  36
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_IMG_IDX_OFS  40
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_TARGET_SYSTEM_OFS  42
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_CAM_IDX_OFS  43
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_FLAGS_OFS  44
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FIELD_COMPLETED_CAPTURES_OFS  45


//----------------------------------------
//-- Message CAMERA_FEEDBACK pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures,
    fmav_status_t* _status)
{
    fmav_camera_feedback_t* _payload = (fmav_camera_feedback_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt_msl = alt_msl;
    _payload->alt_rel = alt_rel;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->foc_len = foc_len;
    _payload->img_idx = img_idx;
    _payload->target_system = target_system;
    _payload->cam_idx = cam_idx;
    _payload->flags = flags;
    _payload->completed_captures = completed_captures;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK;
    _msg->target_sysid = target_system;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_feedback_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_feedback_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->lat, _payload->lng, _payload->alt_msl, _payload->alt_rel, _payload->roll, _payload->pitch, _payload->yaw, _payload->foc_len, _payload->flags, _payload->completed_captures,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures,
    fmav_status_t* _status)
{
    fmav_camera_feedback_t* _payload = (fmav_camera_feedback_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt_msl = alt_msl;
    _payload->alt_rel = alt_rel;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->foc_len = foc_len;
    _payload->img_idx = img_idx;
    _payload->target_system = target_system;
    _payload->cam_idx = cam_idx;
    _payload->flags = flags;
    _payload->completed_captures = completed_captures;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_feedback_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_feedback_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->lat, _payload->lng, _payload->alt_msl, _payload->alt_rel, _payload->roll, _payload->pitch, _payload->yaw, _payload->foc_len, _payload->flags, _payload->completed_captures,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures,
    fmav_status_t* _status)
{
    fmav_camera_feedback_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lng = lng;
    _payload.alt_msl = alt_msl;
    _payload.alt_rel = alt_rel;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.foc_len = foc_len;
    _payload.img_idx = img_idx;
    _payload.target_system = target_system;
    _payload.cam_idx = cam_idx;
    _payload.flags = flags;
    _payload.completed_captures = completed_captures;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_feedback_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_FEEDBACK decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_feedback_decode(fmav_camera_feedback_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_camera_feedback_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_feedback_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_feedback_get_field_lng(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_alt_msl(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_alt_rel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_feedback_get_field_foc_len(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_get_field_img_idx(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_feedback_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_feedback_get_field_cam_idx(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[43]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_feedback_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_get_field_completed_captures(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK  180

#define mavlink_camera_feedback_t  fmav_camera_feedback_t

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN  47
#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN  45
#define MAVLINK_MSG_ID_180_LEN  47
#define MAVLINK_MSG_ID_180_MIN_LEN  45

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC  52
#define MAVLINK_MSG_ID_180_CRC  52




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_feedback_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_feedback_pack(
        _msg, sysid, compid,
        time_usec, target_system, cam_idx, img_idx, lat, lng, alt_msl, alt_rel, roll, pitch, yaw, foc_len, flags, completed_captures,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_feedback_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_camera_feedback_t* _payload)
{
    return mavlink_msg_camera_feedback_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->lat, _payload->lng, _payload->alt_msl, _payload->alt_rel, _payload->roll, _payload->pitch, _payload->yaw, _payload->foc_len, _payload->flags, _payload->completed_captures);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_feedback_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
    return fmav_msg_camera_feedback_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, target_system, cam_idx, img_idx, lat, lng, alt_msl, alt_rel, roll, pitch, yaw, foc_len, flags, completed_captures,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_feedback_decode(const mavlink_message_t* msg, mavlink_camera_feedback_t* payload)
{
    fmav_msg_camera_feedback_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_FEEDBACK_H

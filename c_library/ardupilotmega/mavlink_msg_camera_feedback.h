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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MIN  45
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX  47
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN  47
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA  52

#define FASTMAVLINK_MSG_ID_180_LEN_MIN  45
#define FASTMAVLINK_MSG_ID_180_LEN_MAX  47
#define FASTMAVLINK_MSG_ID_180_LEN  47
#define FASTMAVLINK_MSG_ID_180_CRCEXTRA  52



#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_FLAGS  1
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_TARGET_SYSTEM_OFS  42
#define FASTMAVLINK_MSG_CAMERA_FEEDBACK_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_FEEDBACK packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures,
    fmav_status_t* _status)
{
    fmav_camera_feedback_t* _payload = (fmav_camera_feedback_t*)msg->payload;

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


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK;

    msg->target_sysid = target_system;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_feedback_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_feedback_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->lat, _payload->lng, _payload->alt_msl, _payload->alt_rel, _payload->roll, _payload->pitch, _payload->yaw, _payload->foc_len, _payload->flags, _payload->completed_captures,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures,
    fmav_status_t* _status)
{
    fmav_camera_feedback_t* _payload = (fmav_camera_feedback_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FEEDBACK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FEEDBACK_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_feedback_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_feedback_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_feedback_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->lat, _payload->lng, _payload->alt_msl, _payload->alt_rel, _payload->roll, _payload->pitch, _payload->yaw, _payload->foc_len, _payload->flags, _payload->completed_captures,
        _status);
}


//----------------------------------------
//-- Message CAMERA_FEEDBACK unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_feedback_decode(fmav_camera_feedback_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_FEEDBACK_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_feedback_pack(
        msg, sysid, compid,
        time_usec, target_system, cam_idx, img_idx, lat, lng, alt_msl, alt_rel, roll, pitch, yaw, foc_len, flags, completed_captures,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_feedback_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
    return fmav_msg_camera_feedback_pack_to_frame_buf(
        (uint8_t*)buf,
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

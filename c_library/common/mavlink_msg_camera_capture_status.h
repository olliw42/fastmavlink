//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_capture_status_t {
    uint32_t time_boot_ms;
    float image_interval;
    uint32_t recording_time_ms;
    float available_capacity;
    uint8_t image_status;
    uint8_t video_status;
    int32_t image_count;
}) fmav_camera_capture_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS  262


#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MIN  18
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA  12

#define FASTMAVLINK_MSG_ID_262_LEN_MIN  18
#define FASTMAVLINK_MSG_ID_262_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_262_LEN  22
#define FASTMAVLINK_MSG_ID_262_CRCEXTRA  12



#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count,
    fmav_status_t* _status)
{
    fmav_camera_capture_status_t* _payload = (fmav_camera_capture_status_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->image_interval = image_interval;
    _payload->recording_time_ms = recording_time_ms;
    _payload->available_capacity = available_capacity;
    _payload->image_status = image_status;
    _payload->video_status = video_status;
    _payload->image_count = image_count;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_capture_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_capture_status_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->image_status, _payload->video_status, _payload->image_interval, _payload->recording_time_ms, _payload->available_capacity, _payload->image_count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count,
    fmav_status_t* _status)
{
    fmav_camera_capture_status_t* _payload = (fmav_camera_capture_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->image_interval = image_interval;
    _payload->recording_time_ms = recording_time_ms;
    _payload->available_capacity = available_capacity;
    _payload->image_status = image_status;
    _payload->video_status = video_status;
    _payload->image_count = image_count;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_capture_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_capture_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_capture_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->image_status, _payload->video_status, _payload->image_interval, _payload->recording_time_ms, _payload->available_capacity, _payload->image_count,
        _status);
}


//----------------------------------------
//-- Message CAMERA_CAPTURE_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_capture_status_decode(fmav_camera_capture_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS  262

#define mavlink_camera_capture_status_t  fmav_camera_capture_status_t

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN  22
#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN  18
#define MAVLINK_MSG_ID_262_LEN  22
#define MAVLINK_MSG_ID_262_MIN_LEN  18

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC  12
#define MAVLINK_MSG_ID_262_CRC  12




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_capture_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_capture_status_pack(
        msg, sysid, compid,
        time_boot_ms, image_status, video_status, image_interval, recording_time_ms, available_capacity, image_count,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_capture_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
    return fmav_msg_camera_capture_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, image_status, video_status, image_interval, recording_time_ms, available_capacity, image_count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_capture_status_decode(const mavlink_message_t* msg, mavlink_camera_capture_status_t* payload)
{
    fmav_msg_camera_capture_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_CAPTURE_STATUS_H

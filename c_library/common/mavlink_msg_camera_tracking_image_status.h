//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_tracking_image_status_t {
    float point_x;
    float point_y;
    float radius;
    float rec_top_x;
    float rec_top_y;
    float rec_bottom_x;
    float rec_bottom_y;
    uint8_t tracking_status;
    uint8_t tracking_mode;
    uint8_t target_data;
}) fmav_camera_tracking_image_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS  275


#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MIN  31
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX  31
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN  31
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA  126

#define FASTMAVLINK_MSG_ID_275_LEN_MIN  31
#define FASTMAVLINK_MSG_ID_275_LEN_MAX  31
#define FASTMAVLINK_MSG_ID_275_LEN  31
#define FASTMAVLINK_MSG_ID_275_CRCEXTRA  126



#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y,
    fmav_status_t* _status)
{
    fmav_camera_tracking_image_status_t* _payload = (fmav_camera_tracking_image_status_t*)msg->payload;

    _payload->point_x = point_x;
    _payload->point_y = point_y;
    _payload->radius = radius;
    _payload->rec_top_x = rec_top_x;
    _payload->rec_top_y = rec_top_y;
    _payload->rec_bottom_x = rec_bottom_x;
    _payload->rec_bottom_y = rec_bottom_y;
    _payload->tracking_status = tracking_status;
    _payload->tracking_mode = tracking_mode;
    _payload->target_data = target_data;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_image_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_image_status_pack(
        msg, sysid, compid,
        _payload->tracking_status, _payload->tracking_mode, _payload->target_data, _payload->point_x, _payload->point_y, _payload->radius, _payload->rec_top_x, _payload->rec_top_y, _payload->rec_bottom_x, _payload->rec_bottom_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y,
    fmav_status_t* _status)
{
    fmav_camera_tracking_image_status_t* _payload = (fmav_camera_tracking_image_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->point_x = point_x;
    _payload->point_y = point_y;
    _payload->radius = radius;
    _payload->rec_top_x = rec_top_x;
    _payload->rec_top_y = rec_top_y;
    _payload->rec_bottom_x = rec_bottom_x;
    _payload->rec_bottom_y = rec_bottom_y;
    _payload->tracking_status = tracking_status;
    _payload->tracking_mode = tracking_mode;
    _payload->target_data = target_data;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_image_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_image_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->tracking_status, _payload->tracking_mode, _payload->target_data, _payload->point_x, _payload->point_y, _payload->radius, _payload->rec_top_x, _payload->rec_top_y, _payload->rec_bottom_x, _payload->rec_bottom_y,
        _status);
}


//----------------------------------------
//-- Message CAMERA_TRACKING_IMAGE_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_tracking_image_status_decode(fmav_camera_tracking_image_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS  275

#define mavlink_camera_tracking_image_status_t  fmav_camera_tracking_image_status_t

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN  31
#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN  31
#define MAVLINK_MSG_ID_275_LEN  31
#define MAVLINK_MSG_ID_275_MIN_LEN  31

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC  126
#define MAVLINK_MSG_ID_275_CRC  126




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_image_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_tracking_image_status_pack(
        msg, sysid, compid,
        tracking_status, tracking_mode, target_data, point_x, point_y, radius, rec_top_x, rec_top_y, rec_bottom_x, rec_bottom_y,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_image_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
    return fmav_msg_camera_tracking_image_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        tracking_status, tracking_mode, target_data, point_x, point_y, radius, rec_top_x, rec_top_y, rec_bottom_x, rec_bottom_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_tracking_image_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_image_status_t* payload)
{
    fmav_msg_camera_tracking_image_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_TRACKING_IMAGE_STATUS_H

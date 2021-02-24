//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H


//----------------------------------------
//-- Message CAMERA_FOV_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_fov_status_t {
    uint32_t time_boot_ms;
    int32_t lat_camera;
    int32_t lon_camera;
    int32_t alt_camera;
    int32_t lat_image;
    int32_t lon_image;
    int32_t alt_image;
    float q[4];
    float hfov;
    float vfov;
}) fmav_camera_fov_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS  271


#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MIN  52
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN  52
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA  22

#define FASTMAVLINK_MSG_ID_271_LEN_MIN  52
#define FASTMAVLINK_MSG_ID_271_LEN_MAX  52
#define FASTMAVLINK_MSG_ID_271_LEN  52
#define FASTMAVLINK_MSG_ID_271_CRCEXTRA  22

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_FOV_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t* _payload = (fmav_camera_fov_status_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_camera = lat_camera;
    _payload->lon_camera = lon_camera;
    _payload->alt_camera = alt_camera;
    _payload->lat_image = lat_image;
    _payload->lon_image = lon_image;
    _payload->alt_image = alt_image;
    _payload->hfov = hfov;
    _payload->vfov = vfov;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_fov_status_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->lat_camera, _payload->lon_camera, _payload->alt_camera, _payload->lat_image, _payload->lon_image, _payload->alt_image, _payload->q, _payload->hfov, _payload->vfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t* _payload = (fmav_camera_fov_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_camera = lat_camera;
    _payload->lon_camera = lon_camera;
    _payload->alt_camera = alt_camera;
    _payload->lat_image = lat_image;
    _payload->lon_image = lon_image;
    _payload->alt_image = alt_image;
    _payload->hfov = hfov;
    _payload->vfov = vfov;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_fov_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->lat_camera, _payload->lon_camera, _payload->alt_camera, _payload->lat_image, _payload->lon_image, _payload->alt_image, _payload->q, _payload->hfov, _payload->vfov,
        _status);
}


//----------------------------------------
//-- Message CAMERA_FOV_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_fov_status_decode(fmav_camera_fov_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS  271

#define mavlink_camera_fov_status_t  fmav_camera_fov_status_t

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN  52
#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN  52
#define MAVLINK_MSG_ID_271_LEN  52
#define MAVLINK_MSG_ID_271_MIN_LEN  52

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC  22
#define MAVLINK_MSG_ID_271_CRC  22

#define MAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_fov_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_fov_status_pack(
        msg, sysid, compid,
        time_boot_ms, lat_camera, lon_camera, alt_camera, lat_image, lon_image, alt_image, q, hfov, vfov,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_fov_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov)
{
    return fmav_msg_camera_fov_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, lat_camera, lon_camera, alt_camera, lat_image, lon_image, alt_image, q, hfov, vfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_fov_status_decode(const mavlink_message_t* msg, mavlink_camera_fov_status_t* payload)
{
    fmav_msg_camera_fov_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_FOV_STATUS_H

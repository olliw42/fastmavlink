//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_image_captured_t {
    uint64_t time_utc;
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    float q[4];
    int32_t image_index;
    uint8_t camera_id;
    int8_t capture_result;
    char file_url[205];
}) fmav_camera_image_captured_t;


#define FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED  263


#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MIN  255
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX  255
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN  255
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA  133

#define FASTMAVLINK_MSG_ID_263_LEN_MIN  255
#define FASTMAVLINK_MSG_ID_263_LEN_MAX  255
#define FASTMAVLINK_MSG_ID_263_LEN  255
#define FASTMAVLINK_MSG_ID_263_CRCEXTRA  133

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN  4
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN  205

#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url,
    fmav_status_t* _status)
{
    fmav_camera_image_captured_t* _payload = (fmav_camera_image_captured_t*)msg->payload;

    _payload->time_utc = time_utc;
    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->image_index = image_index;
    _payload->camera_id = camera_id;
    _payload->capture_result = capture_result;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->file_url), file_url, sizeof(char)*205);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_image_captured_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_image_captured_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->time_utc, _payload->camera_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->q, _payload->image_index, _payload->capture_result, _payload->file_url,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url,
    fmav_status_t* _status)
{
    fmav_camera_image_captured_t* _payload = (fmav_camera_image_captured_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_utc = time_utc;
    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->image_index = image_index;
    _payload->camera_id = camera_id;
    _payload->capture_result = capture_result;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->file_url), file_url, sizeof(char)*205);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_image_captured_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_image_captured_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_image_captured_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->time_utc, _payload->camera_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->q, _payload->image_index, _payload->capture_result, _payload->file_url,
        _status);
}


//----------------------------------------
//-- Message CAMERA_IMAGE_CAPTURED unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_image_captured_decode(fmav_camera_image_captured_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED  263

#define mavlink_camera_image_captured_t  fmav_camera_image_captured_t

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN  255
#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN  255
#define MAVLINK_MSG_ID_263_LEN  255
#define MAVLINK_MSG_ID_263_MIN_LEN  255

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC  133
#define MAVLINK_MSG_ID_263_CRC  133

#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN 4
#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN 205


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_image_captured_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_image_captured_pack(
        msg, sysid, compid,
        time_boot_ms, time_utc, camera_id, lat, lon, alt, relative_alt, q, image_index, capture_result, file_url,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_image_captured_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float* q, int32_t image_index, int8_t capture_result, const char* file_url)
{
    return fmav_msg_camera_image_captured_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, time_utc, camera_id, lat, lon, alt, relative_alt, q, image_index, capture_result, file_url,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_image_captured_decode(const mavlink_message_t* msg, mavlink_camera_image_captured_t* payload)
{
    fmav_msg_camera_image_captured_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_IMAGE_CAPTURED_H

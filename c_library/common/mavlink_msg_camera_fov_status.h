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

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA  22

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FRAME_LEN_MAX  77

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LAT_CAMERA_OFS  4
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LON_CAMERA_OFS  8
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_ALT_CAMERA_OFS  12
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LAT_IMAGE_OFS  16
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_LON_IMAGE_OFS  20
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_ALT_IMAGE_OFS  24
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_OFS  28
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_HFOV_OFS  44
#define FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_VFOV_OFS  48


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float* q, float hfov, float vfov,
    fmav_status_t* _status)
{
    fmav_camera_fov_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.lat_camera = lat_camera;
    _payload.lon_camera = lon_camera;
    _payload.alt_camera = alt_camera;
    _payload.lat_image = lat_image;
    _payload.lon_image = lon_image;
    _payload.alt_image = alt_image;
    _payload.hfov = hfov;
    _payload.vfov = vfov;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_fov_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_fov_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_FOV_STATUS,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_FOV_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_FOV_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_camera_fov_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_camera_fov_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_fov_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_fov_status_decode(fmav_camera_fov_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_FOV_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_fov_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lat_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lon_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_alt_camera(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lat_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_lon_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_camera_fov_status_get_field_alt_image(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_hfov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_vfov(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_camera_fov_status_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_fov_status_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
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

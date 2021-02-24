//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_INFORMATION_H
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_H


//----------------------------------------
//-- Message CAMERA_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_information_t {
    uint32_t time_boot_ms;
    uint32_t firmware_version;
    float focal_length;
    float sensor_size_h;
    float sensor_size_v;
    uint32_t flags;
    uint16_t resolution_h;
    uint16_t resolution_v;
    uint16_t cam_definition_version;
    uint8_t vendor_name[32];
    uint8_t model_name[32];
    uint8_t lens_id;
    char cam_definition_uri[140];
}) fmav_camera_information_t;


#define FASTMAVLINK_MSG_ID_CAMERA_INFORMATION  259


#define FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MIN  235
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX  235
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN  235
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_CRCEXTRA  92

#define FASTMAVLINK_MSG_ID_259_LEN_MIN  235
#define FASTMAVLINK_MSG_ID_259_LEN_MAX  235
#define FASTMAVLINK_MSG_ID_259_LEN  235
#define FASTMAVLINK_MSG_ID_259_CRCEXTRA  92

#define FASTMAVLINK_MSG_CAMERA_INFORMATION_FIELD_VENDOR_NAME_LEN  32
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_FIELD_MODEL_NAME_LEN  32
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_FIELD_CAM_DEFINITION_URI_LEN  140

#define FASTMAVLINK_MSG_CAMERA_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_INFORMATION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char* cam_definition_uri,
    fmav_status_t* _status)
{
    fmav_camera_information_t* _payload = (fmav_camera_information_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->firmware_version = firmware_version;
    _payload->focal_length = focal_length;
    _payload->sensor_size_h = sensor_size_h;
    _payload->sensor_size_v = sensor_size_v;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->cam_definition_version = cam_definition_version;
    _payload->lens_id = lens_id;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->cam_definition_uri), cam_definition_uri, sizeof(char)*140);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->firmware_version, _payload->focal_length, _payload->sensor_size_h, _payload->sensor_size_v, _payload->resolution_h, _payload->resolution_v, _payload->lens_id, _payload->flags, _payload->cam_definition_version, _payload->cam_definition_uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char* cam_definition_uri,
    fmav_status_t* _status)
{
    fmav_camera_information_t* _payload = (fmav_camera_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->firmware_version = firmware_version;
    _payload->focal_length = focal_length;
    _payload->sensor_size_h = sensor_size_h;
    _payload->sensor_size_v = sensor_size_v;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->cam_definition_version = cam_definition_version;
    _payload->lens_id = lens_id;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(uint8_t)*32);
    memcpy(&(_payload->cam_definition_uri), cam_definition_uri, sizeof(char)*140);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_INFORMATION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->firmware_version, _payload->focal_length, _payload->sensor_size_h, _payload->sensor_size_v, _payload->resolution_h, _payload->resolution_v, _payload->lens_id, _payload->flags, _payload->cam_definition_version, _payload->cam_definition_uri,
        _status);
}


//----------------------------------------
//-- Message CAMERA_INFORMATION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_information_decode(fmav_camera_information_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_INFORMATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_INFORMATION  259

#define mavlink_camera_information_t  fmav_camera_information_t

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN  235
#define MAVLINK_MSG_ID_CAMERA_INFORMATION_MIN_LEN  235
#define MAVLINK_MSG_ID_259_LEN  235
#define MAVLINK_MSG_ID_259_MIN_LEN  235

#define MAVLINK_MSG_ID_CAMERA_INFORMATION_CRC  92
#define MAVLINK_MSG_ID_259_CRC  92

#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_CAMERA_INFORMATION_FIELD_CAM_DEFINITION_URI_LEN 140


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char* cam_definition_uri)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_information_pack(
        msg, sysid, compid,
        time_boot_ms, vendor_name, model_name, firmware_version, focal_length, sensor_size_h, sensor_size_v, resolution_h, resolution_v, lens_id, flags, cam_definition_version, cam_definition_uri,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const uint8_t* vendor_name, const uint8_t* model_name, uint32_t firmware_version, float focal_length, float sensor_size_h, float sensor_size_v, uint16_t resolution_h, uint16_t resolution_v, uint8_t lens_id, uint32_t flags, uint16_t cam_definition_version, const char* cam_definition_uri)
{
    return fmav_msg_camera_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, vendor_name, model_name, firmware_version, focal_length, sensor_size_h, sensor_size_v, resolution_h, resolution_v, lens_id, flags, cam_definition_version, cam_definition_uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_information_decode(const mavlink_message_t* msg, mavlink_camera_information_t* payload)
{
    fmav_msg_camera_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_INFORMATION_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_SETTINGS_H
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_H


//----------------------------------------
//-- Message CAMERA_SETTINGS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_settings_t {
    uint32_t time_boot_ms;
    uint8_t mode_id;
    float zoomLevel;
    float focusLevel;
}) fmav_camera_settings_t;


#define FASTMAVLINK_MSG_ID_CAMERA_SETTINGS  260


#define FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MIN  5
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN  13
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA  146

#define FASTMAVLINK_MSG_ID_260_LEN_MIN  5
#define FASTMAVLINK_MSG_ID_260_LEN_MAX  13
#define FASTMAVLINK_MSG_ID_260_LEN  13
#define FASTMAVLINK_MSG_ID_260_CRCEXTRA  146



#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_SETTINGS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t mode_id, float zoomLevel, float focusLevel,
    fmav_status_t* _status)
{
    fmav_camera_settings_t* _payload = (fmav_camera_settings_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->mode_id = mode_id;
    _payload->zoomLevel = zoomLevel;
    _payload->focusLevel = focusLevel;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_SETTINGS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_settings_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_settings_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->mode_id, _payload->zoomLevel, _payload->focusLevel,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t mode_id, float zoomLevel, float focusLevel,
    fmav_status_t* _status)
{
    fmav_camera_settings_t* _payload = (fmav_camera_settings_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->mode_id = mode_id;
    _payload->zoomLevel = zoomLevel;
    _payload->focusLevel = focusLevel;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_SETTINGS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_SETTINGS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_SETTINGS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_settings_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_settings_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->mode_id, _payload->zoomLevel, _payload->focusLevel,
        _status);
}


//----------------------------------------
//-- Message CAMERA_SETTINGS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_settings_decode(fmav_camera_settings_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_SETTINGS  260

#define mavlink_camera_settings_t  fmav_camera_settings_t

#define MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN  13
#define MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN  5
#define MAVLINK_MSG_ID_260_LEN  13
#define MAVLINK_MSG_ID_260_MIN_LEN  5

#define MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC  146
#define MAVLINK_MSG_ID_260_CRC  146




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_settings_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t mode_id, float zoomLevel, float focusLevel)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_settings_pack(
        msg, sysid, compid,
        time_boot_ms, mode_id, zoomLevel, focusLevel,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_settings_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t mode_id, float zoomLevel, float focusLevel)
{
    return fmav_msg_camera_settings_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, mode_id, zoomLevel, focusLevel,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_settings_decode(const mavlink_message_t* msg, mavlink_camera_settings_t* payload)
{
    fmav_msg_camera_settings_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_SETTINGS_H

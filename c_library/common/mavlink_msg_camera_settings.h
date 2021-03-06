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

#define FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA  146

#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FRAME_LEN_MAX  38



#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FIELD_MODE_ID_OFS  4
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FIELD_ZOOMLEVEL_OFS  5
#define FASTMAVLINK_MSG_CAMERA_SETTINGS_FIELD_FOCUSLEVEL_OFS  9


//----------------------------------------
//-- Message CAMERA_SETTINGS packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t mode_id, float zoomLevel, float focusLevel,
    fmav_status_t* _status)
{
    fmav_camera_settings_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.mode_id = mode_id;
    _payload.zoomLevel = zoomLevel;
    _payload.focusLevel = focusLevel;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_SETTINGS,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_settings_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_settings_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_SETTINGS,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_SETTINGS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_SETTINGS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_camera_settings_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_camera_settings_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_settings_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_settings_decode(fmav_camera_settings_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_SETTINGS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_settings_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_settings_get_field_mode_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_settings_get_field_zoomLevel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[5]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_settings_get_field_focusLevel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[9]), sizeof(float));
    return r;
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

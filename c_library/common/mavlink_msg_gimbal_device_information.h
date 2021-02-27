//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_H
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_H


//----------------------------------------
//-- Message GIMBAL_DEVICE_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_device_information_t {
    uint64_t uid;
    uint32_t time_boot_ms;
    uint32_t firmware_version;
    uint32_t hardware_version;
    float roll_min;
    float roll_max;
    float pitch_min;
    float pitch_max;
    float yaw_min;
    float yaw_max;
    uint16_t cap_flags;
    uint16_t custom_cap_flags;
    char vendor_name[32];
    char model_name[32];
    char custom_name[32];
}) fmav_gimbal_device_information_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION  283


#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MIN  144
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX  144
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN  144
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA  74

#define FASTMAVLINK_MSG_ID_283_LEN_MIN  144
#define FASTMAVLINK_MSG_ID_283_LEN_MAX  144
#define FASTMAVLINK_MSG_ID_283_LEN  144
#define FASTMAVLINK_MSG_ID_283_CRCEXTRA  74

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_LEN  32
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_LEN  32
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_LEN  32

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_283_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_283_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message GIMBAL_DEVICE_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* vendor_name, const char* model_name, const char* custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_device_information_t* _payload = (fmav_gimbal_device_information_t*)msg->payload;

    _payload->uid = uid;
    _payload->time_boot_ms = time_boot_ms;
    _payload->firmware_version = firmware_version;
    _payload->hardware_version = hardware_version;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->cap_flags = cap_flags;
    _payload->custom_cap_flags = custom_cap_flags;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(char)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(char)*32);
    memcpy(&(_payload->custom_name), custom_name, sizeof(char)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->custom_name, _payload->firmware_version, _payload->hardware_version, _payload->uid, _payload->cap_flags, _payload->custom_cap_flags, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* vendor_name, const char* model_name, const char* custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_device_information_t* _payload = (fmav_gimbal_device_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->uid = uid;
    _payload->time_boot_ms = time_boot_ms;
    _payload->firmware_version = firmware_version;
    _payload->hardware_version = hardware_version;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->cap_flags = cap_flags;
    _payload->custom_cap_flags = custom_cap_flags;
    memcpy(&(_payload->vendor_name), vendor_name, sizeof(char)*32);
    memcpy(&(_payload->model_name), model_name, sizeof(char)*32);
    memcpy(&(_payload->custom_name), custom_name, sizeof(char)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_device_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->vendor_name, _payload->model_name, _payload->custom_name, _payload->firmware_version, _payload->hardware_version, _payload->uid, _payload->cap_flags, _payload->custom_cap_flags, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* vendor_name, const char* model_name, const char* custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_device_information_t _payload;

    _payload.uid = uid;
    _payload.time_boot_ms = time_boot_ms;
    _payload.firmware_version = firmware_version;
    _payload.hardware_version = hardware_version;
    _payload.roll_min = roll_min;
    _payload.roll_max = roll_max;
    _payload.pitch_min = pitch_min;
    _payload.pitch_max = pitch_max;
    _payload.yaw_min = yaw_min;
    _payload.yaw_max = yaw_max;
    _payload.cap_flags = cap_flags;
    _payload.custom_cap_flags = custom_cap_flags;
    memcpy(&(_payload.vendor_name), vendor_name, sizeof(char)*32);
    memcpy(&(_payload.model_name), model_name, sizeof(char)*32);
    memcpy(&(_payload.custom_name), custom_name, sizeof(char)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_device_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_DEVICE_INFORMATION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_information_decode(fmav_gimbal_device_information_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION  283

#define mavlink_gimbal_device_information_t  fmav_gimbal_device_information_t

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN  144
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN  144
#define MAVLINK_MSG_ID_283_LEN  144
#define MAVLINK_MSG_ID_283_MIN_LEN  144

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC  74
#define MAVLINK_MSG_ID_283_CRC  74

#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, const char* vendor_name, const char* model_name, const char* custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_device_information_pack(
        msg, sysid, compid,
        time_boot_ms, vendor_name, model_name, custom_name, firmware_version, hardware_version, uid, cap_flags, custom_cap_flags, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_device_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* vendor_name, const char* model_name, const char* custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    return fmav_msg_gimbal_device_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, vendor_name, model_name, custom_name, firmware_version, hardware_version, uid, cap_flags, custom_cap_flags, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_device_information_decode(const mavlink_message_t* msg, mavlink_gimbal_device_information_t* payload)
{
    fmav_msg_gimbal_device_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_H

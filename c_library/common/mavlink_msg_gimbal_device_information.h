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

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX  144
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA  74

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FRAME_LEN_MAX  169

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_UID_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_TIME_BOOT_MS_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_FIRMWARE_VERSION_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_HARDWARE_VERSION_OFS  16
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_ROLL_MIN_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_ROLL_MAX_OFS  24
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_PITCH_MIN_OFS  28
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_PITCH_MAX_OFS  32
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_YAW_MIN_OFS  36
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_YAW_MAX_OFS  40
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CAP_FLAGS_OFS  44
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_CAP_FLAGS_OFS  46
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_OFS  48
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_OFS  80
#define FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_OFS  112


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
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_DEVICE_INFORMATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gimbal_device_information_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gimbal_device_information_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_information_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_device_information_decode(fmav_gimbal_device_information_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_gimbal_device_information_get_field_uid(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_device_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_device_information_get_field_firmware_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_device_information_get_field_hardware_version(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_roll_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_roll_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_pitch_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_pitch_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_yaw_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_device_information_get_field_yaw_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_get_field_cap_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_device_information_get_field_custom_cap_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_gimbal_device_information_get_field_vendor_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[48]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_gimbal_device_information_get_field_vendor_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_NUM) return 0;
    return ((char*)&(msg->payload[48]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_gimbal_device_information_get_field_model_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[80]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_gimbal_device_information_get_field_model_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_NUM) return 0;
    return ((char*)&(msg->payload[80]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_gimbal_device_information_get_field_custom_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[112]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_gimbal_device_information_get_field_custom_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_NUM) return 0;
    return ((char*)&(msg->payload[112]))[index];
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

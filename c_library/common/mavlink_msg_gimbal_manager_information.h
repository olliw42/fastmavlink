//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_manager_information_t {
    uint32_t time_boot_ms;
    uint32_t cap_flags;
    float roll_min;
    float roll_max;
    float pitch_min;
    float pitch_max;
    float yaw_min;
    float yaw_max;
    uint8_t gimbal_device_id;
}) fmav_gimbal_manager_information_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION  280


#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MIN  33
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN  33
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA  70

#define FASTMAVLINK_MSG_ID_280_LEN_MIN  33
#define FASTMAVLINK_MSG_ID_280_LEN_MAX  33
#define FASTMAVLINK_MSG_ID_280_LEN  33
#define FASTMAVLINK_MSG_ID_280_CRCEXTRA  70



#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_280_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_280_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t* _payload = (fmav_gimbal_manager_information_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->cap_flags = cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_device_id = gimbal_device_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->cap_flags, _payload->gimbal_device_id, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t* _payload = (fmav_gimbal_manager_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->cap_flags = cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_device_id = gimbal_device_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->cap_flags, _payload->gimbal_device_id, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_information_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.cap_flags = cap_flags;
    _payload.roll_min = roll_min;
    _payload.roll_max = roll_max;
    _payload.pitch_min = pitch_min;
    _payload.pitch_max = pitch_max;
    _payload.yaw_min = yaw_min;
    _payload.yaw_max = yaw_max;
    _payload.gimbal_device_id = gimbal_device_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_MANAGER_INFORMATION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_information_decode(fmav_gimbal_manager_information_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION  280

#define mavlink_gimbal_manager_information_t  fmav_gimbal_manager_information_t

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_LEN  33
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_MIN_LEN  33
#define MAVLINK_MSG_ID_280_LEN  33
#define MAVLINK_MSG_ID_280_MIN_LEN  33

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION_CRC  70
#define MAVLINK_MSG_ID_280_CRC  70




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_manager_information_pack(
        msg, sysid, compid,
        time_boot_ms, cap_flags, gimbal_device_id, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t cap_flags, uint8_t gimbal_device_id, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    return fmav_msg_gimbal_manager_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, cap_flags, gimbal_device_id, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_manager_information_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_information_t* payload)
{
    fmav_msg_gimbal_manager_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_MANAGER_INFORMATION_H

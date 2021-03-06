//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_H
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_H


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_storm32_gimbal_manager_information_t {
    uint32_t device_cap_flags;
    uint32_t manager_cap_flags;
    float roll_min;
    float roll_max;
    float pitch_min;
    float pitch_max;
    float yaw_min;
    float yaw_max;
    uint8_t gimbal_id;
}) fmav_storm32_gimbal_manager_information_t;


#define FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION  60010

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA  208

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_DEVICE_CAP_FLAGS_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_MANAGER_CAP_FLAGS_OFS  4
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_ROLL_MIN_OFS  8
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_ROLL_MAX_OFS  12
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_PITCH_MIN_OFS  16
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_PITCH_MAX_OFS  20
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_YAW_MIN_OFS  24
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_YAW_MAX_OFS  28
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_FIELD_GIMBAL_ID_OFS  32


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_information_t* _payload = (fmav_storm32_gimbal_manager_information_t*)msg->payload;

    _payload->device_cap_flags = device_cap_flags;
    _payload->manager_cap_flags = manager_cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_id = gimbal_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_information_pack(
        msg, sysid, compid,
        _payload->gimbal_id, _payload->device_cap_flags, _payload->manager_cap_flags, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_information_t* _payload = (fmav_storm32_gimbal_manager_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->device_cap_flags = device_cap_flags;
    _payload->manager_cap_flags = manager_cap_flags;
    _payload->roll_min = roll_min;
    _payload->roll_max = roll_max;
    _payload->pitch_min = pitch_min;
    _payload->pitch_max = pitch_max;
    _payload->yaw_min = yaw_min;
    _payload->yaw_max = yaw_max;
    _payload->gimbal_id = gimbal_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->gimbal_id, _payload->device_cap_flags, _payload->manager_cap_flags, _payload->roll_min, _payload->roll_max, _payload->pitch_min, _payload->pitch_max, _payload->yaw_min, _payload->yaw_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_information_t _payload;

    _payload.device_cap_flags = device_cap_flags;
    _payload.manager_cap_flags = manager_cap_flags;
    _payload.roll_min = roll_min;
    _payload.roll_max = roll_max;
    _payload.pitch_min = pitch_min;
    _payload.pitch_max = pitch_max;
    _payload.yaw_min = yaw_min;
    _payload.yaw_max = yaw_max;
    _payload.gimbal_id = gimbal_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_INFORMATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_storm32_gimbal_manager_information_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_storm32_gimbal_manager_information_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_manager_information_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_manager_information_decode(fmav_storm32_gimbal_manager_information_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_storm32_gimbal_manager_information_get_field_device_cap_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_storm32_gimbal_manager_information_get_field_manager_cap_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_roll_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_roll_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_pitch_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_pitch_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_yaw_min(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_storm32_gimbal_manager_information_get_field_yaw_max(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_information_get_field_gimbal_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION  60010

#define mavlink_storm32_gimbal_manager_information_t  fmav_storm32_gimbal_manager_information_t

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_LEN  33
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_MIN_LEN  33
#define MAVLINK_MSG_ID_60010_LEN  33
#define MAVLINK_MSG_ID_60010_MIN_LEN  33

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_INFORMATION_CRC  208
#define MAVLINK_MSG_ID_60010_CRC  208




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_manager_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storm32_gimbal_manager_information_pack(
        msg, sysid, compid,
        gimbal_id, device_cap_flags, manager_cap_flags, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_manager_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gimbal_id, uint32_t device_cap_flags, uint32_t manager_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
    return fmav_msg_storm32_gimbal_manager_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        gimbal_id, device_cap_flags, manager_cap_flags, roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_storm32_gimbal_manager_information_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_information_t* payload)
{
    fmav_msg_storm32_gimbal_manager_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_INFORMATION_H

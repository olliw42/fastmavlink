//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_H
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_H


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_PROFILE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_storm32_gimbal_manager_profile_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t gimbal_id;
    uint8_t profile;
    uint8_t priorities[8];
    uint8_t profile_flags;
    uint8_t rc_timeout;
    uint8_t timeouts[8];
}) fmav_storm32_gimbal_manager_profile_t;


#define FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE  60015

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA  78

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FLAGS  3
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FRAME_LEN_MAX  47

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TARGET_COMPONENT_OFS  1
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_GIMBAL_ID_OFS  2
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PROFILE_OFS  3
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_OFS  4
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PROFILE_FLAGS_OFS  12
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_RC_TIMEOUT_OFS  13
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_OFS  14


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_PROFILE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_profile_t* _payload = (fmav_storm32_gimbal_manager_profile_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_id = gimbal_id;
    _payload->profile = profile;
    _payload->profile_flags = profile_flags;
    _payload->rc_timeout = rc_timeout;
    memcpy(&(_payload->priorities), priorities, sizeof(uint8_t)*8);
    memcpy(&(_payload->timeouts), timeouts, sizeof(uint8_t)*8);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_profile_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->gimbal_id, _payload->profile, _payload->priorities, _payload->profile_flags, _payload->rc_timeout, _payload->timeouts,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_profile_t* _payload = (fmav_storm32_gimbal_manager_profile_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_id = gimbal_id;
    _payload->profile = profile;
    _payload->profile_flags = profile_flags;
    _payload->rc_timeout = rc_timeout;
    memcpy(&(_payload->priorities), priorities, sizeof(uint8_t)*8);
    memcpy(&(_payload->timeouts), timeouts, sizeof(uint8_t)*8);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_profile_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->gimbal_id, _payload->profile, _payload->priorities, _payload->profile_flags, _payload->rc_timeout, _payload->timeouts,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_profile_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.gimbal_id = gimbal_id;
    _payload.profile = profile;
    _payload.profile_flags = profile_flags;
    _payload.rc_timeout = rc_timeout;
    memcpy(&(_payload.priorities), priorities, sizeof(uint8_t)*8);
    memcpy(&(_payload.timeouts), timeouts, sizeof(uint8_t)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_profile_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_PROFILE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_manager_profile_decode(fmav_storm32_gimbal_manager_profile_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_gimbal_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_profile(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_profile_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_rc_timeout(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_storm32_gimbal_manager_profile_get_field_priorities_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_priorities(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_NUM) return 0;
    return ((uint8_t*)&(msg->payload[4]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_storm32_gimbal_manager_profile_get_field_timeouts_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[14]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_storm32_gimbal_manager_profile_get_field_timeouts(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_NUM) return 0;
    return ((uint8_t*)&(msg->payload[14]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE  60015

#define mavlink_storm32_gimbal_manager_profile_t  fmav_storm32_gimbal_manager_profile_t

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_LEN  22
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_MIN_LEN  22
#define MAVLINK_MSG_ID_60015_LEN  22
#define MAVLINK_MSG_ID_60015_MIN_LEN  22

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE_CRC  78
#define MAVLINK_MSG_ID_60015_CRC  78

#define MAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_LEN 8
#define MAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storm32_gimbal_manager_profile_pack(
        _msg, sysid, compid,
        target_system, target_component, gimbal_id, profile, priorities, profile_flags, rc_timeout, timeouts,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, gimbal_id, profile, priorities, profile_flags, rc_timeout, timeouts,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_storm32_gimbal_manager_profile_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_profile_t* payload)
{
    fmav_msg_storm32_gimbal_manager_profile_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_H

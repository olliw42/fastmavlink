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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MIN  22
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA  78

#define FASTMAVLINK_MSG_ID_60015_LEN_MIN  22
#define FASTMAVLINK_MSG_ID_60015_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_60015_LEN  22
#define FASTMAVLINK_MSG_ID_60015_CRCEXTRA  78

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_PRIORITIES_LEN  8
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FIELD_TIMEOUTS_LEN  8

#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_FLAGS  3
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_PROFILE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_profile_t* _payload = (fmav_storm32_gimbal_manager_profile_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_id = gimbal_id;
    _payload->profile = profile;
    _payload->profile_flags = profile_flags;
    _payload->rc_timeout = rc_timeout;
    memcpy(&(_payload->priorities), priorities, sizeof(uint8_t)*8);
    memcpy(&(_payload->timeouts), timeouts, sizeof(uint8_t)*8);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_profile_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->gimbal_id, _payload->profile, _payload->priorities, _payload->profile_flags, _payload->rc_timeout, _payload->timeouts,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts,
    fmav_status_t* _status)
{
    fmav_storm32_gimbal_manager_profile_t* _payload = (fmav_storm32_gimbal_manager_profile_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->gimbal_id = gimbal_id;
    _payload->profile = profile;
    _payload->profile_flags = profile_flags;
    _payload->rc_timeout = rc_timeout;
    memcpy(&(_payload->priorities), priorities, sizeof(uint8_t)*8);
    memcpy(&(_payload->timeouts), timeouts, sizeof(uint8_t)*8);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_PROFILE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storm32_gimbal_manager_profile_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storm32_gimbal_manager_profile_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->gimbal_id, _payload->profile, _payload->priorities, _payload->profile_flags, _payload->rc_timeout, _payload->timeouts,
        _status);
}


//----------------------------------------
//-- Message STORM32_GIMBAL_MANAGER_PROFILE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storm32_gimbal_manager_profile_decode(fmav_storm32_gimbal_manager_profile_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_STORM32_GIMBAL_MANAGER_PROFILE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storm32_gimbal_manager_profile_pack(
        msg, sysid, compid,
        target_system, target_component, gimbal_id, profile, priorities, profile_flags, rc_timeout, timeouts,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storm32_gimbal_manager_profile_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t profile, const uint8_t* priorities, uint8_t profile_flags, uint8_t rc_timeout, const uint8_t* timeouts)
{
    return fmav_msg_storm32_gimbal_manager_profile_pack_to_frame_buf(
        (uint8_t*)buf,
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

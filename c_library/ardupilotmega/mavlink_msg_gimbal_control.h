//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_CONTROL_H
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_H


//----------------------------------------
//-- Message GIMBAL_CONTROL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_control_t {
    float demanded_rate_x;
    float demanded_rate_y;
    float demanded_rate_z;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_control_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_CONTROL  201


#define FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN  14
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA  205

#define FASTMAVLINK_MSG_ID_201_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_201_LEN_MAX  14
#define FASTMAVLINK_MSG_ID_201_LEN  14
#define FASTMAVLINK_MSG_ID_201_CRCEXTRA  205



#define FASTMAVLINK_MSG_GIMBAL_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_CONTROL_TARGET_COMPONENT_OFS  13


//----------------------------------------
//-- Message GIMBAL_CONTROL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z,
    fmav_status_t* _status)
{
    fmav_gimbal_control_t* _payload = (fmav_gimbal_control_t*)msg->payload;

    _payload->demanded_rate_x = demanded_rate_x;
    _payload->demanded_rate_y = demanded_rate_y;
    _payload->demanded_rate_z = demanded_rate_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_CONTROL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_control_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->demanded_rate_x, _payload->demanded_rate_y, _payload->demanded_rate_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z,
    fmav_status_t* _status)
{
    fmav_gimbal_control_t* _payload = (fmav_gimbal_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->demanded_rate_x = demanded_rate_x;
    _payload->demanded_rate_y = demanded_rate_y;
    _payload->demanded_rate_z = demanded_rate_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_CONTROL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_control_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->demanded_rate_x, _payload->demanded_rate_y, _payload->demanded_rate_z,
        _status);
}


//----------------------------------------
//-- Message GIMBAL_CONTROL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_control_decode(fmav_gimbal_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GIMBAL_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_CONTROL  201

#define mavlink_gimbal_control_t  fmav_gimbal_control_t

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_LEN  14
#define MAVLINK_MSG_ID_GIMBAL_CONTROL_MIN_LEN  14
#define MAVLINK_MSG_ID_201_LEN  14
#define MAVLINK_MSG_ID_201_MIN_LEN  14

#define MAVLINK_MSG_ID_GIMBAL_CONTROL_CRC  205
#define MAVLINK_MSG_ID_201_CRC  205




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_control_pack(
        msg, sysid, compid,
        target_system, target_component, demanded_rate_x, demanded_rate_y, demanded_rate_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float demanded_rate_x, float demanded_rate_y, float demanded_rate_z)
{
    return fmav_msg_gimbal_control_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, demanded_rate_x, demanded_rate_y, demanded_rate_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_control_decode(const mavlink_message_t* msg, mavlink_gimbal_control_t* payload)
{
    fmav_msg_gimbal_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_CONTROL_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_actuator_control_target_t {
    uint64_t time_usec;
    float controls[8];
    uint8_t group_mlx;
}) fmav_actuator_control_target_t;


#define FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET  140


#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MIN  41
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN  41
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA  181

#define FASTMAVLINK_MSG_ID_140_LEN_MIN  41
#define FASTMAVLINK_MSG_ID_140_LEN_MAX  41
#define FASTMAVLINK_MSG_ID_140_LEN  41
#define FASTMAVLINK_MSG_ID_140_CRCEXTRA  181

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_LEN  8

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls,
    fmav_status_t* _status)
{
    fmav_actuator_control_target_t* _payload = (fmav_actuator_control_target_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->group_mlx = group_mlx;
    memcpy(&(_payload->controls), controls, sizeof(float)*8);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_control_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_control_target_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->group_mlx, _payload->controls,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls,
    fmav_status_t* _status)
{
    fmav_actuator_control_target_t* _payload = (fmav_actuator_control_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->group_mlx = group_mlx;
    memcpy(&(_payload->controls), controls, sizeof(float)*8);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_control_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_control_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->group_mlx, _payload->controls,
        _status);
}


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_actuator_control_target_decode(fmav_actuator_control_target_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET  140

#define mavlink_actuator_control_target_t  fmav_actuator_control_target_t

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN  41
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_MIN_LEN  41
#define MAVLINK_MSG_ID_140_LEN  41
#define MAVLINK_MSG_ID_140_MIN_LEN  41

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_CRC  181
#define MAVLINK_MSG_ID_140_CRC  181

#define MAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_control_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t group_mlx, const float* controls)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_actuator_control_target_pack(
        msg, sysid, compid,
        time_usec, group_mlx, controls,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_control_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls)
{
    return fmav_msg_actuator_control_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, group_mlx, controls,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_actuator_control_target_decode(const mavlink_message_t* msg, mavlink_actuator_control_target_t* payload)
{
    fmav_msg_actuator_control_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H
//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOUNT_CONTROL_H
#define FASTMAVLINK_MSG_MOUNT_CONTROL_H


//----------------------------------------
//-- Message MOUNT_CONTROL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mount_control_t {
    int32_t input_a;
    int32_t input_b;
    int32_t input_c;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t save_position;
}) fmav_mount_control_t;


#define FASTMAVLINK_MSG_ID_MOUNT_CONTROL  157


#define FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MIN  15
#define FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX  15
#define FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN  15
#define FASTMAVLINK_MSG_MOUNT_CONTROL_CRCEXTRA  21

#define FASTMAVLINK_MSG_ID_157_LEN_MIN  15
#define FASTMAVLINK_MSG_ID_157_LEN_MAX  15
#define FASTMAVLINK_MSG_ID_157_LEN  15
#define FASTMAVLINK_MSG_ID_157_CRCEXTRA  21



#define FASTMAVLINK_MSG_MOUNT_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_MOUNT_CONTROL_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_MOUNT_CONTROL_TARGET_COMPONENT_OFS  13


//----------------------------------------
//-- Message MOUNT_CONTROL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position,
    fmav_status_t* _status)
{
    fmav_mount_control_t* _payload = (fmav_mount_control_t*)msg->payload;

    _payload->input_a = input_a;
    _payload->input_b = input_b;
    _payload->input_c = input_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->save_position = save_position;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MOUNT_CONTROL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MOUNT_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_control_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->input_a, _payload->input_b, _payload->input_c, _payload->save_position,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position,
    fmav_status_t* _status)
{
    fmav_mount_control_t* _payload = (fmav_mount_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->input_a = input_a;
    _payload->input_b = input_b;
    _payload->input_c = input_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->save_position = save_position;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOUNT_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_CONTROL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_control_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->input_a, _payload->input_b, _payload->input_c, _payload->save_position,
        _status);
}


//----------------------------------------
//-- Message MOUNT_CONTROL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mount_control_decode(fmav_mount_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MOUNT_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOUNT_CONTROL  157

#define mavlink_mount_control_t  fmav_mount_control_t

#define MAVLINK_MSG_ID_MOUNT_CONTROL_LEN  15
#define MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN  15
#define MAVLINK_MSG_ID_157_LEN  15
#define MAVLINK_MSG_ID_157_MIN_LEN  15

#define MAVLINK_MSG_ID_MOUNT_CONTROL_CRC  21
#define MAVLINK_MSG_ID_157_CRC  21




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mount_control_pack(
        msg, sysid, compid,
        target_system, target_component, input_a, input_b, input_c, save_position,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t input_a, int32_t input_b, int32_t input_c, uint8_t save_position)
{
    return fmav_msg_mount_control_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, input_a, input_b, input_c, save_position,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mount_control_decode(const mavlink_message_t* msg, mavlink_mount_control_t* payload)
{
    fmav_msg_mount_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOUNT_CONTROL_H

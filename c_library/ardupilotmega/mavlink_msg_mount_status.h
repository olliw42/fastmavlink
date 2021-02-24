//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOUNT_STATUS_H
#define FASTMAVLINK_MSG_MOUNT_STATUS_H


//----------------------------------------
//-- Message MOUNT_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mount_status_t {
    int32_t pointing_a;
    int32_t pointing_b;
    int32_t pointing_c;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_mount_status_t;


#define FASTMAVLINK_MSG_ID_MOUNT_STATUS  158


#define FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN  14
#define FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA  134

#define FASTMAVLINK_MSG_ID_158_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_158_LEN_MAX  14
#define FASTMAVLINK_MSG_ID_158_LEN  14
#define FASTMAVLINK_MSG_ID_158_CRCEXTRA  134



#define FASTMAVLINK_MSG_MOUNT_STATUS_FLAGS  3
#define FASTMAVLINK_MSG_MOUNT_STATUS_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_MOUNT_STATUS_TARGET_COMPONENT_OFS  13


//----------------------------------------
//-- Message MOUNT_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c,
    fmav_status_t* _status)
{
    fmav_mount_status_t* _payload = (fmav_mount_status_t*)msg->payload;

    _payload->pointing_a = pointing_a;
    _payload->pointing_b = pointing_b;
    _payload->pointing_c = pointing_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MOUNT_STATUS;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_status_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->pointing_a, _payload->pointing_b, _payload->pointing_c,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c,
    fmav_status_t* _status)
{
    fmav_mount_status_t* _payload = (fmav_mount_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->pointing_a = pointing_a;
    _payload->pointing_b = pointing_b;
    _payload->pointing_c = pointing_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->pointing_a, _payload->pointing_b, _payload->pointing_c,
        _status);
}


//----------------------------------------
//-- Message MOUNT_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mount_status_decode(fmav_mount_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOUNT_STATUS  158

#define mavlink_mount_status_t  fmav_mount_status_t

#define MAVLINK_MSG_ID_MOUNT_STATUS_LEN  14
#define MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN  14
#define MAVLINK_MSG_ID_158_LEN  14
#define MAVLINK_MSG_ID_158_MIN_LEN  14

#define MAVLINK_MSG_ID_MOUNT_STATUS_CRC  134
#define MAVLINK_MSG_ID_158_CRC  134




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mount_status_pack(
        msg, sysid, compid,
        target_system, target_component, pointing_a, pointing_b, pointing_c,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c)
{
    return fmav_msg_mount_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, pointing_a, pointing_b, pointing_c,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mount_status_decode(const mavlink_message_t* msg, mavlink_mount_status_t* payload)
{
    fmav_msg_mount_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOUNT_STATUS_H

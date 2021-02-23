//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_MODE_H
#define FASTMAVLINK_MSG_SET_MODE_H


//----------------------------------------
//-- Message SET_MODE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_mode_t {
    uint32_t custom_mode;
    uint8_t target_system;
    uint8_t base_mode;
}) fmav_set_mode_t;


#define FASTMAVLINK_MSG_ID_SET_MODE  11


#define FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MIN  6
#define FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN  6
#define FASTMAVLINK_MSG_SET_MODE_CRCEXTRA  89

#define FASTMAVLINK_MSG_ID_11_LEN_MIN  6
#define FASTMAVLINK_MSG_ID_11_LEN_MAX  6
#define FASTMAVLINK_MSG_ID_11_LEN  6
#define FASTMAVLINK_MSG_ID_11_CRCEXTRA  89



#define FASTMAVLINK_MSG_SET_MODE_FLAGS  1
#define FASTMAVLINK_MSG_SET_MODE_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_SET_MODE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SET_MODE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode,
    fmav_status_t* _status)
{
    fmav_set_mode_t* _payload = (fmav_set_mode_t*)msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->target_system = target_system;
    _payload->base_mode = base_mode;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SET_MODE;

    msg->target_sysid = target_system;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SET_MODE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mode_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->base_mode, _payload->custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode,
    fmav_status_t* _status)
{
    fmav_set_mode_t* _payload = (fmav_set_mode_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->target_system = target_system;
    _payload->base_mode = base_mode;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_MODE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MODE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MODE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MODE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mode_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mode_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->base_mode, _payload->custom_mode,
        _status);
}


//----------------------------------------
//-- Message SET_MODE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_mode_decode(fmav_set_mode_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SET_MODE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_MODE  11

#define mavlink_set_mode_t  fmav_set_mode_t

#define MAVLINK_MSG_ID_SET_MODE_LEN  6
#define MAVLINK_MSG_ID_SET_MODE_MIN_LEN  6
#define MAVLINK_MSG_ID_11_LEN  6
#define MAVLINK_MSG_ID_11_MIN_LEN  6

#define MAVLINK_MSG_ID_SET_MODE_CRC  89
#define MAVLINK_MSG_ID_11_CRC  89




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mode_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_mode_pack(
        msg, sysid, compid,
        target_system, base_mode, custom_mode,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mode_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t base_mode, uint32_t custom_mode)
{
    return fmav_msg_set_mode_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, base_mode, custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_mode_decode(const mavlink_message_t* msg, mavlink_set_mode_t* payload)
{
    fmav_msg_set_mode_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_MODE_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_H
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_H


//----------------------------------------
//-- Message UALBERTA_SYS_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ualberta_sys_status_t {
    uint8_t mode;
    uint8_t nav_mode;
    uint8_t pilot;
}) fmav_ualberta_sys_status_t;


#define FASTMAVLINK_MSG_ID_UALBERTA_SYS_STATUS  222


#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MIN  3
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN  3
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_CRCEXTRA  15

#define FASTMAVLINK_MSG_ID_222_LEN_MIN  3
#define FASTMAVLINK_MSG_ID_222_LEN_MAX  3
#define FASTMAVLINK_MSG_ID_222_LEN  3
#define FASTMAVLINK_MSG_ID_222_CRCEXTRA  15



#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UALBERTA_SYS_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ualberta_sys_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mode, uint8_t nav_mode, uint8_t pilot,
    fmav_status_t* _status)
{
    fmav_ualberta_sys_status_t* _payload = (fmav_ualberta_sys_status_t*)msg->payload;

    _payload->mode = mode;
    _payload->nav_mode = nav_mode;
    _payload->pilot = pilot;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UALBERTA_SYS_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ualberta_sys_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ualberta_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ualberta_sys_status_pack(
        msg, sysid, compid,
        _payload->mode, _payload->nav_mode, _payload->pilot,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ualberta_sys_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mode, uint8_t nav_mode, uint8_t pilot,
    fmav_status_t* _status)
{
    fmav_ualberta_sys_status_t* _payload = (fmav_ualberta_sys_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mode = mode;
    _payload->nav_mode = nav_mode;
    _payload->pilot = pilot;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UALBERTA_SYS_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UALBERTA_SYS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ualberta_sys_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ualberta_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ualberta_sys_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->mode, _payload->nav_mode, _payload->pilot,
        _status);
}


//----------------------------------------
//-- Message UALBERTA_SYS_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ualberta_sys_status_decode(fmav_ualberta_sys_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS  222

#define mavlink_ualberta_sys_status_t  fmav_ualberta_sys_status_t

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_LEN  3
#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_MIN_LEN  3
#define MAVLINK_MSG_ID_222_LEN  3
#define MAVLINK_MSG_ID_222_MIN_LEN  3

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS_CRC  15
#define MAVLINK_MSG_ID_222_CRC  15




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ualberta_sys_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ualberta_sys_status_pack(
        msg, sysid, compid,
        mode, nav_mode, pilot,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ualberta_sys_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
    return fmav_msg_ualberta_sys_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        mode, nav_mode, pilot,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ualberta_sys_status_decode(const mavlink_message_t* msg, mavlink_ualberta_sys_status_t* payload)
{
    fmav_msg_ualberta_sys_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UALBERTA_SYS_STATUS_H

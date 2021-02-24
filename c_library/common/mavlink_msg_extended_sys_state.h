//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EXTENDED_SYS_STATE_H
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_H


//----------------------------------------
//-- Message EXTENDED_SYS_STATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_extended_sys_state_t {
    uint8_t vtol_state;
    uint8_t landed_state;
}) fmav_extended_sys_state_t;


#define FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE  245


#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_CRCEXTRA  130

#define FASTMAVLINK_MSG_ID_245_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_245_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_245_LEN  2
#define FASTMAVLINK_MSG_ID_245_CRCEXTRA  130



#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_FLAGS  0
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EXTENDED_SYS_STATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message EXTENDED_SYS_STATE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_extended_sys_state_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t vtol_state, uint8_t landed_state,
    fmav_status_t* _status)
{
    fmav_extended_sys_state_t* _payload = (fmav_extended_sys_state_t*)msg->payload;

    _payload->vtol_state = vtol_state;
    _payload->landed_state = landed_state;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_EXTENDED_SYS_STATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_extended_sys_state_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_extended_sys_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_extended_sys_state_pack(
        msg, sysid, compid,
        _payload->vtol_state, _payload->landed_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_extended_sys_state_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t vtol_state, uint8_t landed_state,
    fmav_status_t* _status)
{
    fmav_extended_sys_state_t* _payload = (fmav_extended_sys_state_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->vtol_state = vtol_state;
    _payload->landed_state = landed_state;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EXTENDED_SYS_STATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EXTENDED_SYS_STATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_extended_sys_state_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_extended_sys_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_extended_sys_state_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->vtol_state, _payload->landed_state,
        _status);
}


//----------------------------------------
//-- Message EXTENDED_SYS_STATE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_extended_sys_state_decode(fmav_extended_sys_state_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_EXTENDED_SYS_STATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE  245

#define mavlink_extended_sys_state_t  fmav_extended_sys_state_t

#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN  2
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN  2
#define MAVLINK_MSG_ID_245_LEN  2
#define MAVLINK_MSG_ID_245_MIN_LEN  2

#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE_CRC  130
#define MAVLINK_MSG_ID_245_CRC  130




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_extended_sys_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t vtol_state, uint8_t landed_state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_extended_sys_state_pack(
        msg, sysid, compid,
        vtol_state, landed_state,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_extended_sys_state_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t vtol_state, uint8_t landed_state)
{
    return fmav_msg_extended_sys_state_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        vtol_state, landed_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_extended_sys_state_decode(const mavlink_message_t* msg, mavlink_extended_sys_state_t* payload)
{
    fmav_msg_extended_sys_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EXTENDED_SYS_STATE_H

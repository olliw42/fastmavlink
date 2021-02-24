//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_CURRENT_H
#define FASTMAVLINK_MSG_MISSION_CURRENT_H


//----------------------------------------
//-- Message MISSION_CURRENT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_current_t {
    uint16_t seq;
}) fmav_mission_current_t;


#define FASTMAVLINK_MSG_ID_MISSION_CURRENT  42


#define FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_MISSION_CURRENT_CRCEXTRA  28

#define FASTMAVLINK_MSG_ID_42_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_42_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_42_LEN  2
#define FASTMAVLINK_MSG_ID_42_CRCEXTRA  28



#define FASTMAVLINK_MSG_MISSION_CURRENT_FLAGS  0
#define FASTMAVLINK_MSG_MISSION_CURRENT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MISSION_CURRENT_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message MISSION_CURRENT packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_current_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq,
    fmav_status_t* _status)
{
    fmav_mission_current_t* _payload = (fmav_mission_current_t*)msg->payload;

    _payload->seq = seq;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_CURRENT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_CURRENT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_current_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_current_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_current_pack(
        msg, sysid, compid,
        _payload->seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_current_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq,
    fmav_status_t* _status)
{
    fmav_mission_current_t* _payload = (fmav_mission_current_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->seq = seq;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_CURRENT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CURRENT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CURRENT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CURRENT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_current_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_current_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_current_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->seq,
        _status);
}


//----------------------------------------
//-- Message MISSION_CURRENT unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_current_decode(fmav_mission_current_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MISSION_CURRENT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_CURRENT  42

#define mavlink_mission_current_t  fmav_mission_current_t

#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN  2
#define MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN  2
#define MAVLINK_MSG_ID_42_LEN  2
#define MAVLINK_MSG_ID_42_MIN_LEN  2

#define MAVLINK_MSG_ID_MISSION_CURRENT_CRC  28
#define MAVLINK_MSG_ID_42_CRC  28




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_current_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t seq)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_current_pack(
        msg, sysid, compid,
        seq,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_current_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq)
{
    return fmav_msg_mission_current_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_current_decode(const mavlink_message_t* msg, mavlink_mission_current_t* payload)
{
    fmav_msg_mission_current_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_CURRENT_H
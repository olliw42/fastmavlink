//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_COUNT_H
#define FASTMAVLINK_MSG_MISSION_COUNT_H


//----------------------------------------
//-- Message MISSION_COUNT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_count_t {
    uint16_t count;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t mission_type;
}) fmav_mission_count_t;


#define FASTMAVLINK_MSG_ID_MISSION_COUNT  44


#define FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MIN  4
#define FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN  5
#define FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA  221

#define FASTMAVLINK_MSG_ID_44_LEN_MIN  4
#define FASTMAVLINK_MSG_ID_44_LEN_MAX  5
#define FASTMAVLINK_MSG_ID_44_LEN  5
#define FASTMAVLINK_MSG_ID_44_CRCEXTRA  221



#define FASTMAVLINK_MSG_MISSION_COUNT_FLAGS  3
#define FASTMAVLINK_MSG_MISSION_COUNT_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_MISSION_COUNT_TARGET_COMPONENT_OFS  3


//----------------------------------------
//-- Message MISSION_COUNT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_count_t* _payload = (fmav_mission_count_t*)msg->payload;

    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_COUNT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_count_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_count_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_count_t* _payload = (fmav_mission_count_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_COUNT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_COUNT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_COUNT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_count_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_count_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->mission_type,
        _status);
}


//----------------------------------------
//-- Message MISSION_COUNT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_count_decode(fmav_mission_count_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_COUNT  44

#define mavlink_mission_count_t  fmav_mission_count_t

#define MAVLINK_MSG_ID_MISSION_COUNT_LEN  5
#define MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN  4
#define MAVLINK_MSG_ID_44_LEN  5
#define MAVLINK_MSG_ID_44_MIN_LEN  4

#define MAVLINK_MSG_ID_MISSION_COUNT_CRC  221
#define MAVLINK_MSG_ID_44_CRC  221




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_count_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_count_pack(
        msg, sysid, compid,
        target_system, target_component, count, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_count_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
{
    return fmav_msg_mission_count_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, count, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_count_decode(const mavlink_message_t* msg, mavlink_mission_count_t* payload)
{
    fmav_msg_mission_count_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_COUNT_H

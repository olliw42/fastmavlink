//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_ITEM_H
#define FASTMAVLINK_MSG_MISSION_ITEM_H


//----------------------------------------
//-- Message MISSION_ITEM
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_item_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float x;
    float y;
    float z;
    uint16_t seq;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint8_t current;
    uint8_t autocontinue;
    uint8_t mission_type;
}) fmav_mission_item_t;


#define FASTMAVLINK_MSG_ID_MISSION_ITEM  39


#define FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MIN  37
#define FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN  38
#define FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA  254

#define FASTMAVLINK_MSG_ID_39_LEN_MIN  37
#define FASTMAVLINK_MSG_ID_39_LEN_MAX  38
#define FASTMAVLINK_MSG_ID_39_LEN  38
#define FASTMAVLINK_MSG_ID_39_CRCEXTRA  254



#define FASTMAVLINK_MSG_MISSION_ITEM_FLAGS  3
#define FASTMAVLINK_MSG_MISSION_ITEM_TARGET_SYSTEM_OFS  32
#define FASTMAVLINK_MSG_MISSION_ITEM_TARGET_COMPONENT_OFS  33


//----------------------------------------
//-- Message MISSION_ITEM packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_item_t* _payload = (fmav_mission_item_t*)msg->payload;

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->seq = seq;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;
    _payload->mission_type = mission_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_ITEM;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seq, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_item_t* _payload = (fmav_mission_item_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param1 = param1;
    _payload->param2 = param2;
    _payload->param3 = param3;
    _payload->param4 = param4;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->seq = seq;
    _payload->command = command;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;
    _payload->current = current;
    _payload->autocontinue = autocontinue;
    _payload->mission_type = mission_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_ITEM;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seq, _payload->frame, _payload->command, _payload->current, _payload->autocontinue, _payload->param1, _payload->param2, _payload->param3, _payload->param4, _payload->x, _payload->y, _payload->z, _payload->mission_type,
        _status);
}


//----------------------------------------
//-- Message MISSION_ITEM unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_item_decode(fmav_mission_item_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MISSION_ITEM_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_ITEM  39

#define mavlink_mission_item_t  fmav_mission_item_t

#define MAVLINK_MSG_ID_MISSION_ITEM_LEN  38
#define MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN  37
#define MAVLINK_MSG_ID_39_LEN  38
#define MAVLINK_MSG_ID_39_MIN_LEN  37

#define MAVLINK_MSG_ID_MISSION_ITEM_CRC  254
#define MAVLINK_MSG_ID_39_CRC  254




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_item_pack(
        msg, sysid, compid,
        target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
    return fmav_msg_mission_item_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_item_decode(const mavlink_message_t* msg, mavlink_mission_item_t* payload)
{
    fmav_msg_mission_item_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_ITEM_H

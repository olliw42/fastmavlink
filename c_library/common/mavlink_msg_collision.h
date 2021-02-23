//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COLLISION_H
#define FASTMAVLINK_MSG_COLLISION_H


//----------------------------------------
//-- Message COLLISION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_collision_t {
    uint32_t id;
    float time_to_minimum_delta;
    float altitude_minimum_delta;
    float horizontal_minimum_delta;
    uint8_t src;
    uint8_t action;
    uint8_t threat_level;
}) fmav_collision_t;


#define FASTMAVLINK_MSG_ID_COLLISION  247


#define FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MIN  19
#define FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN  19
#define FASTMAVLINK_MSG_COLLISION_CRCEXTRA  81

#define FASTMAVLINK_MSG_ID_247_LEN_MIN  19
#define FASTMAVLINK_MSG_ID_247_LEN_MAX  19
#define FASTMAVLINK_MSG_ID_247_LEN  19
#define FASTMAVLINK_MSG_ID_247_CRCEXTRA  81



#define FASTMAVLINK_MSG_COLLISION_FLAGS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message COLLISION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)msg->payload;

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COLLISION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_COLLISION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack(
        msg, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COLLISION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


//----------------------------------------
//-- Message COLLISION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_collision_decode(fmav_collision_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COLLISION  247

#define mavlink_collision_t  fmav_collision_t

#define MAVLINK_MSG_ID_COLLISION_LEN  19
#define MAVLINK_MSG_ID_COLLISION_MIN_LEN  19
#define MAVLINK_MSG_ID_247_LEN  19
#define MAVLINK_MSG_ID_247_MIN_LEN  19

#define MAVLINK_MSG_ID_COLLISION_CRC  81
#define MAVLINK_MSG_ID_247_CRC  81




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_collision_pack(
        msg, sysid, compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    return fmav_msg_collision_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_collision_decode(const mavlink_message_t* msg, mavlink_collision_t* payload)
{
    fmav_msg_collision_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COLLISION_H

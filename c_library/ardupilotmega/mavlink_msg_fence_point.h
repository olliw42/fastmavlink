//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FENCE_POINT_H
#define FASTMAVLINK_MSG_FENCE_POINT_H


//----------------------------------------
//-- Message FENCE_POINT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_fence_point_t {
    float lat;
    float lng;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t idx;
    uint8_t count;
}) fmav_fence_point_t;


#define FASTMAVLINK_MSG_ID_FENCE_POINT  160


#define FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MIN  12
#define FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN  12
#define FASTMAVLINK_MSG_FENCE_POINT_CRCEXTRA  78

#define FASTMAVLINK_MSG_ID_160_LEN_MIN  12
#define FASTMAVLINK_MSG_ID_160_LEN_MAX  12
#define FASTMAVLINK_MSG_ID_160_LEN  12
#define FASTMAVLINK_MSG_ID_160_CRCEXTRA  78



#define FASTMAVLINK_MSG_FENCE_POINT_FLAGS  3
#define FASTMAVLINK_MSG_FENCE_POINT_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_FENCE_POINT_TARGET_COMPONENT_OFS  9


//----------------------------------------
//-- Message FENCE_POINT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_point_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng,
    fmav_status_t* _status)
{
    fmav_fence_point_t* _payload = (fmav_fence_point_t*)msg->payload;

    _payload->lat = lat;
    _payload->lng = lng;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->idx = idx;
    _payload->count = count;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_FENCE_POINT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_FENCE_POINT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_point_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fence_point_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fence_point_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->idx, _payload->count, _payload->lat, _payload->lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_point_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng,
    fmav_status_t* _status)
{
    fmav_fence_point_t* _payload = (fmav_fence_point_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lng = lng;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->idx = idx;
    _payload->count = count;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FENCE_POINT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FENCE_POINT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FENCE_POINT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FENCE_POINT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fence_point_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fence_point_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fence_point_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->idx, _payload->count, _payload->lat, _payload->lng,
        _status);
}


//----------------------------------------
//-- Message FENCE_POINT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_fence_point_decode(fmav_fence_point_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_FENCE_POINT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FENCE_POINT  160

#define mavlink_fence_point_t  fmav_fence_point_t

#define MAVLINK_MSG_ID_FENCE_POINT_LEN  12
#define MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN  12
#define MAVLINK_MSG_ID_160_LEN  12
#define MAVLINK_MSG_ID_160_MIN_LEN  12

#define MAVLINK_MSG_ID_FENCE_POINT_CRC  78
#define MAVLINK_MSG_ID_160_CRC  78




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fence_point_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_fence_point_pack(
        msg, sysid, compid,
        target_system, target_component, idx, count, lat, lng,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fence_point_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, float lat, float lng)
{
    return fmav_msg_fence_point_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, idx, count, lat, lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_fence_point_decode(const mavlink_message_t* msg, mavlink_fence_point_t* payload)
{
    fmav_msg_fence_point_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FENCE_POINT_H

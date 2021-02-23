//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEBUG_VECT_H
#define FASTMAVLINK_MSG_DEBUG_VECT_H


//----------------------------------------
//-- Message DEBUG_VECT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_debug_vect_t {
    uint64_t time_usec;
    float x;
    float y;
    float z;
    char name[10];
}) fmav_debug_vect_t;


#define FASTMAVLINK_MSG_ID_DEBUG_VECT  250


#define FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MIN  30
#define FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX  30
#define FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN  30
#define FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA  49

#define FASTMAVLINK_MSG_ID_250_LEN_MIN  30
#define FASTMAVLINK_MSG_ID_250_LEN_MAX  30
#define FASTMAVLINK_MSG_ID_250_LEN  30
#define FASTMAVLINK_MSG_ID_250_CRCEXTRA  49

#define FASTMAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN  10

#define FASTMAVLINK_MSG_DEBUG_VECT_FLAGS  0
#define FASTMAVLINK_MSG_DEBUG_VECT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DEBUG_VECT_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message DEBUG_VECT packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z,
    fmav_status_t* _status)
{
    fmav_debug_vect_t* _payload = (fmav_debug_vect_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DEBUG_VECT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_debug_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_debug_vect_pack(
        msg, sysid, compid,
        _payload->name, _payload->time_usec, _payload->x, _payload->y, _payload->z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z,
    fmav_status_t* _status)
{
    fmav_debug_vect_t* _payload = (fmav_debug_vect_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEBUG_VECT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEBUG_VECT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEBUG_VECT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEBUG_VECT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_debug_vect_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_debug_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_debug_vect_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->name, _payload->time_usec, _payload->x, _payload->y, _payload->z,
        _status);
}


//----------------------------------------
//-- Message DEBUG_VECT unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_debug_vect_decode(fmav_debug_vect_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DEBUG_VECT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEBUG_VECT  250

#define mavlink_debug_vect_t  fmav_debug_vect_t

#define MAVLINK_MSG_ID_DEBUG_VECT_LEN  30
#define MAVLINK_MSG_ID_DEBUG_VECT_MIN_LEN  30
#define MAVLINK_MSG_ID_250_LEN  30
#define MAVLINK_MSG_ID_250_MIN_LEN  30

#define MAVLINK_MSG_ID_DEBUG_VECT_CRC  49
#define MAVLINK_MSG_ID_250_CRC  49

#define MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN 10


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_debug_vect_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const char* name, uint64_t time_usec, float x, float y, float z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_debug_vect_pack(
        msg, sysid, compid,
        name, time_usec, x, y, z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_debug_vect_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* name, uint64_t time_usec, float x, float y, float z)
{
    return fmav_msg_debug_vect_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        name, time_usec, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_debug_vect_decode(const mavlink_message_t* msg, mavlink_debug_vect_t* payload)
{
    fmav_msg_debug_vect_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEBUG_VECT_H

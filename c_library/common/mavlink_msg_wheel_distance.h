//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WHEEL_DISTANCE_H
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_H


//----------------------------------------
//-- Message WHEEL_DISTANCE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wheel_distance_t {
    uint64_t time_usec;
    double distance[16];
    uint8_t count;
}) fmav_wheel_distance_t;


#define FASTMAVLINK_MSG_ID_WHEEL_DISTANCE  9000


#define FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MIN  137
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX  137
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN  137
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_CRCEXTRA  113

#define FASTMAVLINK_MSG_ID_9000_LEN_MIN  137
#define FASTMAVLINK_MSG_ID_9000_LEN_MAX  137
#define FASTMAVLINK_MSG_ID_9000_LEN  137
#define FASTMAVLINK_MSG_ID_9000_CRCEXTRA  113

#define FASTMAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN  16

#define FASTMAVLINK_MSG_WHEEL_DISTANCE_FLAGS  0
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WHEEL_DISTANCE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message WHEEL_DISTANCE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wheel_distance_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t count, const double* distance,
    fmav_status_t* _status)
{
    fmav_wheel_distance_t* _payload = (fmav_wheel_distance_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->count = count;
    memcpy(&(_payload->distance), distance, sizeof(double)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_WHEEL_DISTANCE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_WHEEL_DISTANCE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wheel_distance_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wheel_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wheel_distance_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->count, _payload->distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wheel_distance_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t count, const double* distance,
    fmav_status_t* _status)
{
    fmav_wheel_distance_t* _payload = (fmav_wheel_distance_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->count = count;
    memcpy(&(_payload->distance), distance, sizeof(double)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WHEEL_DISTANCE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WHEEL_DISTANCE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WHEEL_DISTANCE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WHEEL_DISTANCE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wheel_distance_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wheel_distance_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wheel_distance_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->count, _payload->distance,
        _status);
}


//----------------------------------------
//-- Message WHEEL_DISTANCE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wheel_distance_decode(fmav_wheel_distance_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_WHEEL_DISTANCE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WHEEL_DISTANCE  9000

#define mavlink_wheel_distance_t  fmav_wheel_distance_t

#define MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN  137
#define MAVLINK_MSG_ID_WHEEL_DISTANCE_MIN_LEN  137
#define MAVLINK_MSG_ID_9000_LEN  137
#define MAVLINK_MSG_ID_9000_MIN_LEN  137

#define MAVLINK_MSG_ID_WHEEL_DISTANCE_CRC  113
#define MAVLINK_MSG_ID_9000_CRC  113

#define MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wheel_distance_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t count, const double* distance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wheel_distance_pack(
        msg, sysid, compid,
        time_usec, count, distance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wheel_distance_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t count, const double* distance)
{
    return fmav_msg_wheel_distance_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, count, distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wheel_distance_decode(const mavlink_message_t* msg, mavlink_wheel_distance_t* payload)
{
    fmav_msg_wheel_distance_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WHEEL_DISTANCE_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PING_H
#define FASTMAVLINK_MSG_PING_H


//----------------------------------------
//-- Message PING
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ping_t {
    uint64_t time_usec;
    uint32_t seq;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_ping_t;


#define FASTMAVLINK_MSG_ID_PING  4


#define FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_PING_PAYLOAD_LEN  14
#define FASTMAVLINK_MSG_PING_CRCEXTRA  237

#define FASTMAVLINK_MSG_ID_4_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_4_LEN_MAX  14
#define FASTMAVLINK_MSG_ID_4_LEN  14
#define FASTMAVLINK_MSG_ID_4_CRCEXTRA  237



#define FASTMAVLINK_MSG_PING_FLAGS  3
#define FASTMAVLINK_MSG_PING_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_PING_TARGET_COMPONENT_OFS  13

#define FASTMAVLINK_MSG_PING_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_4_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_4_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message PING packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_ping_t* _payload = (fmav_ping_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->seq = seq;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PING;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PING_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ping_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ping_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->seq, _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_ping_t* _payload = (fmav_ping_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->seq = seq;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PING;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PING >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PING >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ping_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ping_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->seq, _payload->target_system, _payload->target_component,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_ping_t _payload;

    _payload.time_usec = time_usec;
    _payload.seq = seq;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PING,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ping_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ping_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PING,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PING_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PING unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ping_decode(fmav_ping_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PING_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PING  4

#define mavlink_ping_t  fmav_ping_t

#define MAVLINK_MSG_ID_PING_LEN  14
#define MAVLINK_MSG_ID_PING_MIN_LEN  14
#define MAVLINK_MSG_ID_4_LEN  14
#define MAVLINK_MSG_ID_4_MIN_LEN  14

#define MAVLINK_MSG_ID_PING_CRC  237
#define MAVLINK_MSG_ID_4_CRC  237




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ping_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ping_pack(
        msg, sysid, compid,
        time_usec, seq, target_system, target_component,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ping_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq, uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_ping_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, seq, target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ping_decode(const mavlink_message_t* msg, mavlink_ping_t* payload)
{
    fmav_msg_ping_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PING_H

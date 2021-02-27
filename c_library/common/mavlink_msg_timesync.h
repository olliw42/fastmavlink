//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TIMESYNC_H
#define FASTMAVLINK_MSG_TIMESYNC_H


//----------------------------------------
//-- Message TIMESYNC
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_timesync_t {
    int64_t tc1;
    int64_t ts1;
}) fmav_timesync_t;


#define FASTMAVLINK_MSG_ID_TIMESYNC  111


#define FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MIN  16
#define FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN  16
#define FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA  34

#define FASTMAVLINK_MSG_ID_111_LEN_MIN  16
#define FASTMAVLINK_MSG_ID_111_LEN_MAX  16
#define FASTMAVLINK_MSG_ID_111_LEN  16
#define FASTMAVLINK_MSG_ID_111_CRCEXTRA  34



#define FASTMAVLINK_MSG_TIMESYNC_FLAGS  0
#define FASTMAVLINK_MSG_TIMESYNC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TIMESYNC_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TIMESYNC_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_111_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_111_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message TIMESYNC packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1,
    fmav_status_t* _status)
{
    fmav_timesync_t* _payload = (fmav_timesync_t*)msg->payload;

    _payload->tc1 = tc1;
    _payload->ts1 = ts1;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TIMESYNC;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_timesync_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_timesync_pack(
        msg, sysid, compid,
        _payload->tc1, _payload->ts1,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1,
    fmav_status_t* _status)
{
    fmav_timesync_t* _payload = (fmav_timesync_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->tc1 = tc1;
    _payload->ts1 = ts1;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TIMESYNC;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TIMESYNC >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TIMESYNC >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_timesync_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_timesync_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->tc1, _payload->ts1,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1,
    fmav_status_t* _status)
{
    fmav_timesync_t _payload;

    _payload.tc1 = tc1;
    _payload.ts1 = ts1;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TIMESYNC,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_timesync_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TIMESYNC,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TIMESYNC unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_timesync_decode(fmav_timesync_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TIMESYNC  111

#define mavlink_timesync_t  fmav_timesync_t

#define MAVLINK_MSG_ID_TIMESYNC_LEN  16
#define MAVLINK_MSG_ID_TIMESYNC_MIN_LEN  16
#define MAVLINK_MSG_ID_111_LEN  16
#define MAVLINK_MSG_ID_111_MIN_LEN  16

#define MAVLINK_MSG_ID_TIMESYNC_CRC  34
#define MAVLINK_MSG_ID_111_CRC  34




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_timesync_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int64_t tc1, int64_t ts1)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_timesync_pack(
        msg, sysid, compid,
        tc1, ts1,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_timesync_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1)
{
    return fmav_msg_timesync_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        tc1, ts1,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_timesync_decode(const mavlink_message_t* msg, mavlink_timesync_t* payload)
{
    fmav_msg_timesync_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TIMESYNC_H

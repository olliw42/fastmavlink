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

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_timesync_t {
    int64_t tc1;
    int64_t ts1;
}) fmav_timesync_t;


#define FASTMAVLINK_MSG_ID_TIMESYNC  111

#define FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA  34

#define FASTMAVLINK_MSG_TIMESYNC_FLAGS  0
#define FASTMAVLINK_MSG_TIMESYNC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TIMESYNC_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TIMESYNC_FRAME_LEN_MAX  41



#define FASTMAVLINK_MSG_TIMESYNC_FIELD_TC1_OFS  0
#define FASTMAVLINK_MSG_TIMESYNC_FIELD_TS1_OFS  8


//----------------------------------------
//-- Message TIMESYNC pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1,
    fmav_status_t* _status)
{
    fmav_timesync_t* _payload = (fmav_timesync_t*)_msg->payload;

    _payload->tc1 = tc1;
    _payload->ts1 = ts1;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TIMESYNC;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_timesync_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_timesync_pack(
        _msg, sysid, compid,
        _payload->tc1, _payload->ts1,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1,
    fmav_status_t* _status)
{
    fmav_timesync_t* _payload = (fmav_timesync_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->tc1 = tc1;
    _payload->ts1 = ts1;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TIMESYNC;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TIMESYNC >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TIMESYNC >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_timesync_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_timesync_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_timesync_pack_to_frame_buf(
        _buf, sysid, compid,
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
        FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIMESYNC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TIMESYNC decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_timesync_decode(fmav_timesync_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TIMESYNC_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int64_t fmav_msg_timesync_get_field_tc1(const fmav_message_t* msg)
{
    int64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int64_t fmav_msg_timesync_get_field_ts1(const fmav_message_t* msg)
{
    int64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int64_t));
    return r;
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
    mavlink_message_t* _msg,
    int64_t tc1, int64_t ts1)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_timesync_pack(
        _msg, sysid, compid,
        tc1, ts1,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_timesync_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int64_t tc1, int64_t ts1)
{
    return fmav_msg_timesync_pack_to_frame_buf(
        (uint8_t*)_buf,
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

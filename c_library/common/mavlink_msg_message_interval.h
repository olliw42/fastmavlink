//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MESSAGE_INTERVAL_H
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_H


//----------------------------------------
//-- Message MESSAGE_INTERVAL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_message_interval_t {
    int32_t interval_us;
    uint16_t message_id;
}) fmav_message_interval_t;


#define FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL  244


#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MIN  6
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN  6
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA  95

#define FASTMAVLINK_MSG_ID_244_LEN_MIN  6
#define FASTMAVLINK_MSG_ID_244_LEN_MAX  6
#define FASTMAVLINK_MSG_ID_244_LEN  6
#define FASTMAVLINK_MSG_ID_244_CRCEXTRA  95



#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FLAGS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_244_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_244_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message MESSAGE_INTERVAL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t* _payload = (fmav_message_interval_t*)msg->payload;

    _payload->interval_us = interval_us;
    _payload->message_id = message_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_message_interval_pack(
        msg, sysid, compid,
        _payload->message_id, _payload->interval_us,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t* _payload = (fmav_message_interval_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->interval_us = interval_us;
    _payload->message_id = message_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_message_interval_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->message_id, _payload->interval_us,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us,
    fmav_status_t* _status)
{
    fmav_message_interval_t _payload;

    _payload.interval_us = interval_us;
    _payload.message_id = message_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_message_interval_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MESSAGE_INTERVAL,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MESSAGE_INTERVAL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_message_interval_decode(fmav_message_interval_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL  244

#define mavlink_message_interval_t  fmav_message_interval_t

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_LEN  6
#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_MIN_LEN  6
#define MAVLINK_MSG_ID_244_LEN  6
#define MAVLINK_MSG_ID_244_MIN_LEN  6

#define MAVLINK_MSG_ID_MESSAGE_INTERVAL_CRC  95
#define MAVLINK_MSG_ID_244_CRC  95




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_message_interval_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t message_id, int32_t interval_us)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_message_interval_pack(
        msg, sysid, compid,
        message_id, interval_us,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_message_interval_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t message_id, int32_t interval_us)
{
    return fmav_msg_message_interval_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        message_id, interval_us,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_message_interval_decode(const mavlink_message_t* msg, mavlink_message_interval_t* payload)
{
    fmav_msg_message_interval_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MESSAGE_INTERVAL_H

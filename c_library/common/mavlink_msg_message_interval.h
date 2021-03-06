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

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA  95

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FLAGS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FIELD_INTERVAL_US_OFS  0
#define FASTMAVLINK_MSG_MESSAGE_INTERVAL_FIELD_MESSAGE_ID_OFS  4


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
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MESSAGE_INTERVAL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MESSAGE_INTERVAL unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_message_interval_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_message_interval_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_message_interval_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_message_interval_decode(fmav_message_interval_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MESSAGE_INTERVAL_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_message_interval_get_field_interval_us(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_message_interval_get_field_message_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
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

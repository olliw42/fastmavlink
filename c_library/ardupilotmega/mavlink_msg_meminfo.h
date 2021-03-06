//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MEMINFO_H
#define FASTMAVLINK_MSG_MEMINFO_H


//----------------------------------------
//-- Message MEMINFO
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_meminfo_t {
    uint16_t brkval;
    uint16_t freemem;
    uint32_t freemem32;
}) fmav_meminfo_t;


#define FASTMAVLINK_MSG_ID_MEMINFO  152

#define FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_MEMINFO_CRCEXTRA  208

#define FASTMAVLINK_MSG_MEMINFO_FLAGS  0
#define FASTMAVLINK_MSG_MEMINFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MEMINFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MEMINFO_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_MEMINFO_FIELD_BRKVAL_OFS  0
#define FASTMAVLINK_MSG_MEMINFO_FIELD_FREEMEM_OFS  2
#define FASTMAVLINK_MSG_MEMINFO_FIELD_FREEMEM32_OFS  4


//----------------------------------------
//-- Message MEMINFO packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t brkval, uint16_t freemem, uint32_t freemem32,
    fmav_status_t* _status)
{
    fmav_meminfo_t* _payload = (fmav_meminfo_t*)msg->payload;

    _payload->brkval = brkval;
    _payload->freemem = freemem;
    _payload->freemem32 = freemem32;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MEMINFO;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MEMINFO_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_meminfo_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_meminfo_pack(
        msg, sysid, compid,
        _payload->brkval, _payload->freemem, _payload->freemem32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t brkval, uint16_t freemem, uint32_t freemem32,
    fmav_status_t* _status)
{
    fmav_meminfo_t* _payload = (fmav_meminfo_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->brkval = brkval;
    _payload->freemem = freemem;
    _payload->freemem32 = freemem32;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MEMINFO;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMINFO >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMINFO >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMINFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_meminfo_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_meminfo_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->brkval, _payload->freemem, _payload->freemem32,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t brkval, uint16_t freemem, uint32_t freemem32,
    fmav_status_t* _status)
{
    fmav_meminfo_t _payload;

    _payload.brkval = brkval;
    _payload.freemem = freemem;
    _payload.freemem32 = freemem32;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MEMINFO,
        FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMINFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_meminfo_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MEMINFO,
        FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMINFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MEMINFO unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_meminfo_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_meminfo_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_meminfo_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_meminfo_decode(fmav_meminfo_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMINFO_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_get_field_brkval(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_meminfo_get_field_freemem(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_meminfo_get_field_freemem32(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MEMINFO  152

#define mavlink_meminfo_t  fmav_meminfo_t

#define MAVLINK_MSG_ID_MEMINFO_LEN  8
#define MAVLINK_MSG_ID_MEMINFO_MIN_LEN  4
#define MAVLINK_MSG_ID_152_LEN  8
#define MAVLINK_MSG_ID_152_MIN_LEN  4

#define MAVLINK_MSG_ID_MEMINFO_CRC  208
#define MAVLINK_MSG_ID_152_CRC  208




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_meminfo_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t brkval, uint16_t freemem, uint32_t freemem32)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_meminfo_pack(
        msg, sysid, compid,
        brkval, freemem, freemem32,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_meminfo_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t brkval, uint16_t freemem, uint32_t freemem32)
{
    return fmav_msg_meminfo_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        brkval, freemem, freemem32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_meminfo_decode(const mavlink_message_t* msg, mavlink_meminfo_t* payload)
{
    fmav_msg_meminfo_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MEMINFO_H

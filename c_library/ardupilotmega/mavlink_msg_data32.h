//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA32_H
#define FASTMAVLINK_MSG_DATA32_H


//----------------------------------------
//-- Message DATA32
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data32_t {
    uint8_t type;
    uint8_t len;
    uint8_t data[32];
}) fmav_data32_t;


#define FASTMAVLINK_MSG_ID_DATA32  170

#define FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX  34
#define FASTMAVLINK_MSG_DATA32_CRCEXTRA  73

#define FASTMAVLINK_MSG_DATA32_FLAGS  0
#define FASTMAVLINK_MSG_DATA32_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA32_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DATA32_FRAME_LEN_MAX  59

#define FASTMAVLINK_MSG_DATA32_FIELD_DATA_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_DATA32_FIELD_DATA_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_DATA32_FIELD_TYPE_OFS  0
#define FASTMAVLINK_MSG_DATA32_FIELD_LEN_OFS  1
#define FASTMAVLINK_MSG_DATA32_FIELD_DATA_OFS  2


//----------------------------------------
//-- Message DATA32 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data32_t* _payload = (fmav_data32_t*)msg->payload;

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DATA32;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DATA32_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data32_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data32_pack(
        msg, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data32_t* _payload = (fmav_data32_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA32;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA32 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA32 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA32_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data32_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data32_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data32_t _payload;

    _payload.type = type;
    _payload.len = len;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DATA32,
        FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA32_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data32_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_data32_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DATA32,
        FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA32_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DATA32 unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_data32_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_data32_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data32_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data32_decode(fmav_data32_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA32_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data32_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data32_get_field_len(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_data32_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data32_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DATA32_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[2]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA32  170

#define mavlink_data32_t  fmav_data32_t

#define MAVLINK_MSG_ID_DATA32_LEN  34
#define MAVLINK_MSG_ID_DATA32_MIN_LEN  34
#define MAVLINK_MSG_ID_170_LEN  34
#define MAVLINK_MSG_ID_170_MIN_LEN  34

#define MAVLINK_MSG_ID_DATA32_CRC  73
#define MAVLINK_MSG_ID_170_CRC  73

#define MAVLINK_MSG_DATA32_FIELD_DATA_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data32_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data32_pack(
        msg, sysid, compid,
        type, len, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data32_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    return fmav_msg_data32_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        type, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data32_decode(const mavlink_message_t* msg, mavlink_data32_t* payload)
{
    fmav_msg_data32_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA32_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STATUSTEXT_H
#define FASTMAVLINK_MSG_STATUSTEXT_H


//----------------------------------------
//-- Message STATUSTEXT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_statustext_t {
    uint8_t severity;
    char text[50];
    uint16_t id;
    uint8_t chunk_seq;
}) fmav_statustext_t;


#define FASTMAVLINK_MSG_ID_STATUSTEXT  253

#define FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA  83

#define FASTMAVLINK_MSG_STATUSTEXT_FLAGS  0
#define FASTMAVLINK_MSG_STATUSTEXT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STATUSTEXT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_STATUSTEXT_FRAME_LEN_MAX  79

#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_NUM  50 // number of elements in array
#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN  50 // length of array = number of bytes

#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_SEVERITY_OFS  0
#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_OFS  1
#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_ID_OFS  51
#define FASTMAVLINK_MSG_STATUSTEXT_FIELD_CHUNK_SEQ_OFS  53


//----------------------------------------
//-- Message STATUSTEXT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq,
    fmav_status_t* _status)
{
    fmav_statustext_t* _payload = (fmav_statustext_t*)msg->payload;

    _payload->severity = severity;
    _payload->id = id;
    _payload->chunk_seq = chunk_seq;
    memcpy(&(_payload->text), text, sizeof(char)*50);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_STATUSTEXT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_statustext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_statustext_pack(
        msg, sysid, compid,
        _payload->severity, _payload->text, _payload->id, _payload->chunk_seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq,
    fmav_status_t* _status)
{
    fmav_statustext_t* _payload = (fmav_statustext_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->severity = severity;
    _payload->id = id;
    _payload->chunk_seq = chunk_seq;
    memcpy(&(_payload->text), text, sizeof(char)*50);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STATUSTEXT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STATUSTEXT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STATUSTEXT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_statustext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_statustext_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->severity, _payload->text, _payload->id, _payload->chunk_seq,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq,
    fmav_status_t* _status)
{
    fmav_statustext_t _payload;

    _payload.severity = severity;
    _payload.id = id;
    _payload.chunk_seq = chunk_seq;
    memcpy(&(_payload.text), text, sizeof(char)*50);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_STATUSTEXT,
        FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_statustext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_STATUSTEXT,
        FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STATUSTEXT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message STATUSTEXT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_statustext_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_statustext_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_statustext_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_statustext_decode(fmav_statustext_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_STATUSTEXT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_statustext_get_field_severity(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_statustext_get_field_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_statustext_get_field_chunk_seq(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[53]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_statustext_get_field_text_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[1]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_statustext_get_field_text(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_NUM) return 0;
    return ((char*)&(msg->payload[1]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STATUSTEXT  253

#define mavlink_statustext_t  fmav_statustext_t

#define MAVLINK_MSG_ID_STATUSTEXT_LEN  54
#define MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN  51
#define MAVLINK_MSG_ID_253_LEN  54
#define MAVLINK_MSG_ID_253_MIN_LEN  51

#define MAVLINK_MSG_ID_STATUSTEXT_CRC  83
#define MAVLINK_MSG_ID_253_CRC  83

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 50


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_statustext_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_statustext_pack(
        msg, sysid, compid,
        severity, text, id, chunk_seq,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_statustext_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t severity, const char* text, uint16_t id, uint8_t chunk_seq)
{
    return fmav_msg_statustext_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        severity, text, id, chunk_seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* payload)
{
    fmav_msg_statustext_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STATUSTEXT_H

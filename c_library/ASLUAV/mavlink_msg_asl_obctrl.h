//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ASL_OBCTRL_H
#define FASTMAVLINK_MSG_ASL_OBCTRL_H


//----------------------------------------
//-- Message ASL_OBCTRL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_asl_obctrl_t {
    uint64_t timestamp;
    float uElev;
    float uThrot;
    float uThrot2;
    float uAilL;
    float uAilR;
    float uRud;
    uint8_t obctrl_status;
}) fmav_asl_obctrl_t;


#define FASTMAVLINK_MSG_ID_ASL_OBCTRL  8008

#define FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_ASL_OBCTRL_CRCEXTRA  234

#define FASTMAVLINK_MSG_ASL_OBCTRL_FLAGS  0
#define FASTMAVLINK_MSG_ASL_OBCTRL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ASL_OBCTRL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ASL_OBCTRL_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_UELEV_OFS  8
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_UTHROT_OFS  12
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_UTHROT2_OFS  16
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_UAILL_OFS  20
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_UAILR_OFS  24
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_URUD_OFS  28
#define FASTMAVLINK_MSG_ASL_OBCTRL_FIELD_OBCTRL_STATUS_OFS  32


//----------------------------------------
//-- Message ASL_OBCTRL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status,
    fmav_status_t* _status)
{
    fmav_asl_obctrl_t* _payload = (fmav_asl_obctrl_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->uElev = uElev;
    _payload->uThrot = uThrot;
    _payload->uThrot2 = uThrot2;
    _payload->uAilL = uAilL;
    _payload->uAilR = uAilR;
    _payload->uRud = uRud;
    _payload->obctrl_status = obctrl_status;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ASL_OBCTRL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ASL_OBCTRL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_asl_obctrl_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_asl_obctrl_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->uAilL, _payload->uAilR, _payload->uRud, _payload->obctrl_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status,
    fmav_status_t* _status)
{
    fmav_asl_obctrl_t* _payload = (fmav_asl_obctrl_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->uElev = uElev;
    _payload->uThrot = uThrot;
    _payload->uThrot2 = uThrot2;
    _payload->uAilL = uAilL;
    _payload->uAilR = uAilR;
    _payload->uRud = uRud;
    _payload->obctrl_status = obctrl_status;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ASL_OBCTRL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ASL_OBCTRL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ASL_OBCTRL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASL_OBCTRL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_asl_obctrl_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_asl_obctrl_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->uAilL, _payload->uAilR, _payload->uRud, _payload->obctrl_status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status,
    fmav_status_t* _status)
{
    fmav_asl_obctrl_t _payload;

    _payload.timestamp = timestamp;
    _payload.uElev = uElev;
    _payload.uThrot = uThrot;
    _payload.uThrot2 = uThrot2;
    _payload.uAilL = uAilL;
    _payload.uAilR = uAilR;
    _payload.uRud = uRud;
    _payload.obctrl_status = obctrl_status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ASL_OBCTRL,
        FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASL_OBCTRL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asl_obctrl_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_asl_obctrl_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ASL_OBCTRL,
        FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASL_OBCTRL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ASL_OBCTRL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_asl_obctrl_decode(fmav_asl_obctrl_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASL_OBCTRL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_asl_obctrl_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uElev(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uThrot(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uThrot2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uAilL(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uAilR(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asl_obctrl_get_field_uRud(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_asl_obctrl_get_field_obctrl_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ASL_OBCTRL  8008

#define mavlink_asl_obctrl_t  fmav_asl_obctrl_t

#define MAVLINK_MSG_ID_ASL_OBCTRL_LEN  33
#define MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN  33
#define MAVLINK_MSG_ID_8008_LEN  33
#define MAVLINK_MSG_ID_8008_MIN_LEN  33

#define MAVLINK_MSG_ID_ASL_OBCTRL_CRC  234
#define MAVLINK_MSG_ID_8008_CRC  234




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asl_obctrl_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_asl_obctrl_pack(
        _msg, sysid, compid,
        timestamp, uElev, uThrot, uThrot2, uAilL, uAilR, uRud, obctrl_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asl_obctrl_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_asl_obctrl_t* _payload)
{
    return mavlink_msg_asl_obctrl_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->uAilL, _payload->uAilR, _payload->uRud, _payload->obctrl_status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asl_obctrl_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
    return fmav_msg_asl_obctrl_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, uElev, uThrot, uThrot2, uAilL, uAilR, uRud, obctrl_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_asl_obctrl_decode(const mavlink_message_t* msg, mavlink_asl_obctrl_t* payload)
{
    fmav_msg_asl_obctrl_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ASL_OBCTRL_H

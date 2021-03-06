//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SETUP_SIGNING_H
#define FASTMAVLINK_MSG_SETUP_SIGNING_H


//----------------------------------------
//-- Message SETUP_SIGNING
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_setup_signing_t {
    uint64_t initial_timestamp;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t secret_key[32];
}) fmav_setup_signing_t;


#define FASTMAVLINK_MSG_ID_SETUP_SIGNING  256

#define FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA  71

#define FASTMAVLINK_MSG_SETUP_SIGNING_FLAGS  3
#define FASTMAVLINK_MSG_SETUP_SIGNING_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_SETUP_SIGNING_TARGET_COMPONENT_OFS  9

#define FASTMAVLINK_MSG_SETUP_SIGNING_FRAME_LEN_MAX  67

#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_INITIAL_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_TARGET_COMPONENT_OFS  9
#define FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_OFS  10


//----------------------------------------
//-- Message SETUP_SIGNING packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t* _payload = (fmav_setup_signing_t*)msg->payload;

    _payload->initial_timestamp = initial_timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->secret_key), secret_key, sizeof(uint8_t)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SETUP_SIGNING;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_setup_signing_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->secret_key, _payload->initial_timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t* _payload = (fmav_setup_signing_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->initial_timestamp = initial_timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->secret_key), secret_key, sizeof(uint8_t)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SETUP_SIGNING >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_setup_signing_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->secret_key, _payload->initial_timestamp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp,
    fmav_status_t* _status)
{
    fmav_setup_signing_t _payload;

    _payload.initial_timestamp = initial_timestamp;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.secret_key), secret_key, sizeof(uint8_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SETUP_SIGNING,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_setup_signing_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_setup_signing_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SETUP_SIGNING,
        FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SETUP_SIGNING_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SETUP_SIGNING unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_setup_signing_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_setup_signing_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_setup_signing_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_setup_signing_decode(fmav_setup_signing_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SETUP_SIGNING_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_setup_signing_get_field_initial_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_setup_signing_get_field_secret_key_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[10]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_setup_signing_get_field_secret_key(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_NUM) return 0;
    return ((uint8_t*)&(msg->payload[10]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SETUP_SIGNING  256

#define mavlink_setup_signing_t  fmav_setup_signing_t

#define MAVLINK_MSG_ID_SETUP_SIGNING_LEN  42
#define MAVLINK_MSG_ID_SETUP_SIGNING_MIN_LEN  42
#define MAVLINK_MSG_ID_256_LEN  42
#define MAVLINK_MSG_ID_256_MIN_LEN  42

#define MAVLINK_MSG_ID_SETUP_SIGNING_CRC  71
#define MAVLINK_MSG_ID_256_CRC  71

#define MAVLINK_MSG_SETUP_SIGNING_FIELD_SECRET_KEY_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_setup_signing_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_setup_signing_pack(
        msg, sysid, compid,
        target_system, target_component, secret_key, initial_timestamp,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_setup_signing_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* secret_key, uint64_t initial_timestamp)
{
    return fmav_msg_setup_signing_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, secret_key, initial_timestamp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_setup_signing_decode(const mavlink_message_t* msg, mavlink_setup_signing_t* payload)
{
    fmav_msg_setup_signing_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SETUP_SIGNING_H

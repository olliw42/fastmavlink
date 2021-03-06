//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PLAY_TUNE_V2_H
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_H


//----------------------------------------
//-- Message PLAY_TUNE_V2
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_play_tune_v2_t {
    uint32_t format;
    uint8_t target_system;
    uint8_t target_component;
    char tune[248];
}) fmav_play_tune_v2_t;


#define FASTMAVLINK_MSG_ID_PLAY_TUNE_V2  400

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX  254
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA  110

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FLAGS  3
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FRAME_LEN_MAX  279

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_NUM  248 // number of elements in array
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_LEN  248 // length of array = number of bytes

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_FORMAT_OFS  0
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_OFS  6


//----------------------------------------
//-- Message PLAY_TUNE_V2 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format, const char* tune,
    fmav_status_t* _status)
{
    fmav_play_tune_v2_t* _payload = (fmav_play_tune_v2_t*)msg->payload;

    _payload->format = format;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*248);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PLAY_TUNE_V2;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_v2_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->format, _payload->tune,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format, const char* tune,
    fmav_status_t* _status)
{
    fmav_play_tune_v2_t* _payload = (fmav_play_tune_v2_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->format = format;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*248);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PLAY_TUNE_V2;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE_V2 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE_V2 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_v2_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->format, _payload->tune,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format, const char* tune,
    fmav_status_t* _status)
{
    fmav_play_tune_v2_t _payload;

    _payload.format = format;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.tune), tune, sizeof(char)*248);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PLAY_TUNE_V2,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_v2_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PLAY_TUNE_V2,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PLAY_TUNE_V2 unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_play_tune_v2_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_play_tune_v2_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_play_tune_v2_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_play_tune_v2_decode(fmav_play_tune_v2_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_play_tune_v2_get_field_format(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_play_tune_v2_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_play_tune_v2_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_play_tune_v2_get_field_tune_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_play_tune_v2_get_field_tune(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_NUM) return 0;
    return ((char*)&(msg->payload[6]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PLAY_TUNE_V2  400

#define mavlink_play_tune_v2_t  fmav_play_tune_v2_t

#define MAVLINK_MSG_ID_PLAY_TUNE_V2_LEN  254
#define MAVLINK_MSG_ID_PLAY_TUNE_V2_MIN_LEN  254
#define MAVLINK_MSG_ID_400_LEN  254
#define MAVLINK_MSG_ID_400_MIN_LEN  254

#define MAVLINK_MSG_ID_PLAY_TUNE_V2_CRC  110
#define MAVLINK_MSG_ID_400_CRC  110

#define MAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_LEN 248


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_v2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t format, const char* tune)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_play_tune_v2_pack(
        msg, sysid, compid,
        target_system, target_component, format, tune,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_v2_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t format, const char* tune)
{
    return fmav_msg_play_tune_v2_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, format, tune,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_play_tune_v2_decode(const mavlink_message_t* msg, mavlink_play_tune_v2_t* payload)
{
    fmav_msg_play_tune_v2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PLAY_TUNE_V2_H

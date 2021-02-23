//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PLAY_TUNE_H
#define FASTMAVLINK_MSG_PLAY_TUNE_H


//----------------------------------------
//-- Message PLAY_TUNE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_play_tune_t {
    uint8_t target_system;
    uint8_t target_component;
    char tune[30];
    char tune2[200];
}) fmav_play_tune_t;


#define FASTMAVLINK_MSG_ID_PLAY_TUNE  258


#define FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX  232
#define FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN  232
#define FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA  187

#define FASTMAVLINK_MSG_ID_258_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_258_LEN_MAX  232
#define FASTMAVLINK_MSG_ID_258_LEN  232
#define FASTMAVLINK_MSG_ID_258_CRCEXTRA  187

#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_LEN  30
#define FASTMAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_LEN  200

#define FASTMAVLINK_MSG_PLAY_TUNE_FLAGS  3
#define FASTMAVLINK_MSG_PLAY_TUNE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PLAY_TUNE_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message PLAY_TUNE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2,
    fmav_status_t* _status)
{
    fmav_play_tune_t* _payload = (fmav_play_tune_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*30);
    memcpy(&(_payload->tune2), tune2, sizeof(char)*200);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PLAY_TUNE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->tune, _payload->tune2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2,
    fmav_status_t* _status)
{
    fmav_play_tune_t* _payload = (fmav_play_tune_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->tune), tune, sizeof(char)*30);
    memcpy(&(_payload->tune2), tune2, sizeof(char)*200);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PLAY_TUNE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PLAY_TUNE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PLAY_TUNE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_play_tune_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_play_tune_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_play_tune_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->tune, _payload->tune2,
        _status);
}


//----------------------------------------
//-- Message PLAY_TUNE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_play_tune_decode(fmav_play_tune_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PLAY_TUNE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PLAY_TUNE  258

#define mavlink_play_tune_t  fmav_play_tune_t

#define MAVLINK_MSG_ID_PLAY_TUNE_LEN  232
#define MAVLINK_MSG_ID_PLAY_TUNE_MIN_LEN  32
#define MAVLINK_MSG_ID_258_LEN  232
#define MAVLINK_MSG_ID_258_MIN_LEN  32

#define MAVLINK_MSG_ID_PLAY_TUNE_CRC  187
#define MAVLINK_MSG_ID_258_CRC  187

#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE_LEN 30
#define MAVLINK_MSG_PLAY_TUNE_FIELD_TUNE2_LEN 200


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_play_tune_pack(
        msg, sysid, compid,
        target_system, target_component, tune, tune2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_play_tune_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* tune, const char* tune2)
{
    return fmav_msg_play_tune_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, tune, tune2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_play_tune_decode(const mavlink_message_t* msg, mavlink_play_tune_t* payload)
{
    fmav_msg_play_tune_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PLAY_TUNE_H

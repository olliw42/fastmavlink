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


#define FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MIN  254
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX  254
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN  254
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_CRCEXTRA  110

#define FASTMAVLINK_MSG_ID_400_LEN_MIN  254
#define FASTMAVLINK_MSG_ID_400_LEN_MAX  254
#define FASTMAVLINK_MSG_ID_400_LEN  254
#define FASTMAVLINK_MSG_ID_400_CRCEXTRA  110

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FIELD_TUNE_LEN  248

#define FASTMAVLINK_MSG_PLAY_TUNE_V2_FLAGS  3
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PLAY_TUNE_V2_TARGET_COMPONENT_OFS  5


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
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message PLAY_TUNE_V2 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_play_tune_v2_decode(fmav_play_tune_v2_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PLAY_TUNE_V2_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

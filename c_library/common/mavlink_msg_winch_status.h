//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WINCH_STATUS_H
#define FASTMAVLINK_MSG_WINCH_STATUS_H


//----------------------------------------
//-- Message WINCH_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_winch_status_t {
    uint64_t time_usec;
    float line_length;
    float speed;
    float tension;
    float voltage;
    float current;
    uint32_t status;
    int16_t temperature;
}) fmav_winch_status_t;


#define FASTMAVLINK_MSG_ID_WINCH_STATUS  9005


#define FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MIN  34
#define FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX  34
#define FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN  34
#define FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA  117

#define FASTMAVLINK_MSG_ID_9005_LEN_MIN  34
#define FASTMAVLINK_MSG_ID_9005_LEN_MAX  34
#define FASTMAVLINK_MSG_ID_9005_LEN  34
#define FASTMAVLINK_MSG_ID_9005_CRCEXTRA  117



#define FASTMAVLINK_MSG_WINCH_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_WINCH_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WINCH_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message WINCH_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status,
    fmav_status_t* _status)
{
    fmav_winch_status_t* _payload = (fmav_winch_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->line_length = line_length;
    _payload->speed = speed;
    _payload->tension = tension;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->status = status;
    _payload->temperature = temperature;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_WINCH_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_winch_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_winch_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->line_length, _payload->speed, _payload->tension, _payload->voltage, _payload->current, _payload->temperature, _payload->status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status,
    fmav_status_t* _status)
{
    fmav_winch_status_t* _payload = (fmav_winch_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->line_length = line_length;
    _payload->speed = speed;
    _payload->tension = tension;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->status = status;
    _payload->temperature = temperature;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WINCH_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WINCH_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WINCH_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WINCH_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_winch_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_winch_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_winch_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->line_length, _payload->speed, _payload->tension, _payload->voltage, _payload->current, _payload->temperature, _payload->status,
        _status);
}


//----------------------------------------
//-- Message WINCH_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_winch_status_decode(fmav_winch_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_WINCH_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WINCH_STATUS  9005

#define mavlink_winch_status_t  fmav_winch_status_t

#define MAVLINK_MSG_ID_WINCH_STATUS_LEN  34
#define MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN  34
#define MAVLINK_MSG_ID_9005_LEN  34
#define MAVLINK_MSG_ID_9005_MIN_LEN  34

#define MAVLINK_MSG_ID_WINCH_STATUS_CRC  117
#define MAVLINK_MSG_ID_9005_CRC  117




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_winch_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_winch_status_pack(
        msg, sysid, compid,
        time_usec, line_length, speed, tension, voltage, current, temperature, status,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_winch_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
    return fmav_msg_winch_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, line_length, speed, tension, voltage, current, temperature, status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_winch_status_decode(const mavlink_message_t* msg, mavlink_winch_status_t* payload)
{
    fmav_msg_winch_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WINCH_STATUS_H

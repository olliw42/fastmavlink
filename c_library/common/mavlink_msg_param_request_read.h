//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_REQUEST_READ_H
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_H


//----------------------------------------
//-- Message PARAM_REQUEST_READ
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_request_read_t {
    int16_t param_index;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
}) fmav_param_request_read_t;


#define FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ  20


#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MIN  20
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN  20
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA  214

#define FASTMAVLINK_MSG_ID_20_LEN_MIN  20
#define FASTMAVLINK_MSG_ID_20_LEN_MAX  20
#define FASTMAVLINK_MSG_ID_20_LEN  20
#define FASTMAVLINK_MSG_ID_20_CRCEXTRA  214

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN  16

#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_PARAM_REQUEST_READ_TARGET_COMPONENT_OFS  3


//----------------------------------------
//-- Message PARAM_REQUEST_READ packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)msg->payload;

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_request_read_t* _payload = (fmav_param_request_read_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_REQUEST_READ_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_request_read_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_request_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index,
        _status);
}


//----------------------------------------
//-- Message PARAM_REQUEST_READ unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_request_read_decode(fmav_param_request_read_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_REQUEST_READ_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ  20

#define mavlink_param_request_read_t  fmav_param_request_read_t

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN  20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_MIN_LEN  20
#define MAVLINK_MSG_ID_20_LEN  20
#define MAVLINK_MSG_ID_20_MIN_LEN  20

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_CRC  214
#define MAVLINK_MSG_ID_20_CRC  214

#define MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_request_read_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_index,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_request_read_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index)
{
    return fmav_msg_param_request_read_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_request_read_decode(const mavlink_message_t* msg, mavlink_param_request_read_t* payload)
{
    fmav_msg_param_request_read_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_REQUEST_READ_H

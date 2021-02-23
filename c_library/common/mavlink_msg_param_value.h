//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_VALUE_H
#define FASTMAVLINK_MSG_PARAM_VALUE_H


//----------------------------------------
//-- Message PARAM_VALUE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_value_t {
    float param_value;
    uint16_t param_count;
    uint16_t param_index;
    char param_id[16];
    uint8_t param_type;
}) fmav_param_value_t;


#define FASTMAVLINK_MSG_ID_PARAM_VALUE  22


#define FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MIN  25
#define FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX  25
#define FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN  25
#define FASTMAVLINK_MSG_PARAM_VALUE_CRCEXTRA  220

#define FASTMAVLINK_MSG_ID_22_LEN_MIN  25
#define FASTMAVLINK_MSG_ID_22_LEN_MAX  25
#define FASTMAVLINK_MSG_ID_22_LEN  25
#define FASTMAVLINK_MSG_ID_22_CRCEXTRA  220

#define FASTMAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN  16

#define FASTMAVLINK_MSG_PARAM_VALUE_FLAGS  0
#define FASTMAVLINK_MSG_PARAM_VALUE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_VALUE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message PARAM_VALUE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_value_t* _payload = (fmav_param_value_t*)msg->payload;

    _payload->param_value = param_value;
    _payload->param_count = param_count;
    _payload->param_index = param_index;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_VALUE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_VALUE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_value_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_value_pack(
        msg, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_count, _payload->param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index,
    fmav_status_t* _status)
{
    fmav_param_value_t* _payload = (fmav_param_value_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_value = param_value;
    _payload->param_count = param_count;
    _payload->param_index = param_index;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_VALUE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_VALUE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_VALUE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_VALUE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_value_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_value_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_count, _payload->param_index,
        _status);
}


//----------------------------------------
//-- Message PARAM_VALUE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_value_decode(fmav_param_value_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_VALUE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_VALUE  22

#define mavlink_param_value_t  fmav_param_value_t

#define MAVLINK_MSG_ID_PARAM_VALUE_LEN  25
#define MAVLINK_MSG_ID_PARAM_VALUE_MIN_LEN  25
#define MAVLINK_MSG_ID_22_LEN  25
#define MAVLINK_MSG_ID_22_MIN_LEN  25

#define MAVLINK_MSG_ID_PARAM_VALUE_CRC  220
#define MAVLINK_MSG_ID_22_CRC  220

#define MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_value_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const char* param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_value_pack(
        msg, sysid, compid,
        param_id, param_value, param_type, param_count, param_index,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_value_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
    return fmav_msg_param_value_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        param_id, param_value, param_type, param_count, param_index,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* payload)
{
    fmav_msg_param_value_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_VALUE_H

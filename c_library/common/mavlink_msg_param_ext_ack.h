//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_EXT_ACK_H
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_H


//----------------------------------------
//-- Message PARAM_EXT_ACK
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ext_ack_t {
    char param_id[16];
    char param_value[128];
    uint8_t param_type;
    uint8_t param_result;
}) fmav_param_ext_ack_t;


#define FASTMAVLINK_MSG_ID_PARAM_EXT_ACK  324


#define FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MIN  146
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX  146
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN  146
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_CRCEXTRA  132

#define FASTMAVLINK_MSG_ID_324_LEN_MIN  146
#define FASTMAVLINK_MSG_ID_324_LEN_MAX  146
#define FASTMAVLINK_MSG_ID_324_LEN  146
#define FASTMAVLINK_MSG_ID_324_CRCEXTRA  132

#define FASTMAVLINK_MSG_PARAM_EXT_ACK_FIELD_PARAM_ID_LEN  16
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_FIELD_PARAM_VALUE_LEN  128

#define FASTMAVLINK_MSG_PARAM_EXT_ACK_FLAGS  0
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_ACK_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message PARAM_EXT_ACK packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_ack_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint8_t param_result,
    fmav_status_t* _status)
{
    fmav_param_ext_ack_t* _payload = (fmav_param_ext_ack_t*)msg->payload;

    _payload->param_type = param_type;
    _payload->param_result = param_result;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_EXT_ACK;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_ACK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_ack_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_ack_pack(
        msg, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_ack_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint8_t param_result,
    fmav_status_t* _status)
{
    fmav_param_ext_ack_t* _payload = (fmav_param_ext_ack_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_type = param_type;
    _payload->param_result = param_result;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_EXT_ACK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_ACK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_ACK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_ACK_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_ack_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_ack_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_result,
        _status);
}


//----------------------------------------
//-- Message PARAM_EXT_ACK unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_ack_decode(fmav_param_ext_ack_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_EXT_ACK_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_EXT_ACK  324

#define mavlink_param_ext_ack_t  fmav_param_ext_ack_t

#define MAVLINK_MSG_ID_PARAM_EXT_ACK_LEN  146
#define MAVLINK_MSG_ID_PARAM_EXT_ACK_MIN_LEN  146
#define MAVLINK_MSG_ID_324_LEN  146
#define MAVLINK_MSG_ID_324_MIN_LEN  146

#define MAVLINK_MSG_ID_PARAM_EXT_ACK_CRC  132
#define MAVLINK_MSG_ID_324_CRC  132

#define MAVLINK_MSG_PARAM_EXT_ACK_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_PARAM_EXT_ACK_FIELD_PARAM_VALUE_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_ack_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const char* param_id, const char* param_value, uint8_t param_type, uint8_t param_result)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ext_ack_pack(
        msg, sysid, compid,
        param_id, param_value, param_type, param_result,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_ack_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* param_id, const char* param_value, uint8_t param_type, uint8_t param_result)
{
    return fmav_msg_param_ext_ack_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        param_id, param_value, param_type, param_result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ext_ack_decode(const mavlink_message_t* msg, mavlink_param_ext_ack_t* payload)
{
    fmav_msg_param_ext_ack_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_EXT_ACK_H

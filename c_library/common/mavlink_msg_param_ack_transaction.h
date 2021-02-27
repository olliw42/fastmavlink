//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_H
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_H


//----------------------------------------
//-- Message PARAM_ACK_TRANSACTION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ack_transaction_t {
    float param_value;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t param_type;
    uint8_t param_result;
}) fmav_param_ack_transaction_t;


#define FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION  19


#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MIN  24
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN  24
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_CRCEXTRA  137

#define FASTMAVLINK_MSG_ID_19_LEN_MIN  24
#define FASTMAVLINK_MSG_ID_19_LEN_MAX  24
#define FASTMAVLINK_MSG_ID_19_LEN  24
#define FASTMAVLINK_MSG_ID_19_CRCEXTRA  137

#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_FIELD_PARAM_ID_LEN  16

#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_19_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_19_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message PARAM_ACK_TRANSACTION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type, uint8_t param_result,
    fmav_status_t* _status)
{
    fmav_param_ack_transaction_t* _payload = (fmav_param_ack_transaction_t*)msg->payload;

    _payload->param_value = param_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    _payload->param_result = param_result;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ack_transaction_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ack_transaction_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type, uint8_t param_result,
    fmav_status_t* _status)
{
    fmav_param_ack_transaction_t* _payload = (fmav_param_ack_transaction_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_value = param_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    _payload->param_result = param_result;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ack_transaction_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ack_transaction_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type, _payload->param_result,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type, uint8_t param_result,
    fmav_status_t* _status)
{
    fmav_param_ack_transaction_t _payload;

    _payload.param_value = param_value;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.param_type = param_type;
    _payload.param_result = param_result;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ack_transaction_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ack_transaction_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_ACK_TRANSACTION,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_ACK_TRANSACTION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ack_transaction_decode(fmav_param_ack_transaction_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_ACK_TRANSACTION  19

#define mavlink_param_ack_transaction_t  fmav_param_ack_transaction_t

#define MAVLINK_MSG_ID_PARAM_ACK_TRANSACTION_LEN  24
#define MAVLINK_MSG_ID_PARAM_ACK_TRANSACTION_MIN_LEN  24
#define MAVLINK_MSG_ID_19_LEN  24
#define MAVLINK_MSG_ID_19_MIN_LEN  24

#define MAVLINK_MSG_ID_PARAM_ACK_TRANSACTION_CRC  137
#define MAVLINK_MSG_ID_19_CRC  137

#define MAVLINK_MSG_PARAM_ACK_TRANSACTION_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ack_transaction_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type, uint8_t param_result)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ack_transaction_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_value, param_type, param_result,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ack_transaction_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, float param_value, uint8_t param_type, uint8_t param_result)
{
    return fmav_msg_param_ack_transaction_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_value, param_type, param_result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ack_transaction_decode(const mavlink_message_t* msg, mavlink_param_ack_transaction_t* payload)
{
    fmav_msg_param_ack_transaction_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_ACK_TRANSACTION_H

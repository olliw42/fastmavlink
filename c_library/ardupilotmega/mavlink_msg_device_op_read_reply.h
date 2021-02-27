//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_H
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_H


//----------------------------------------
//-- Message DEVICE_OP_READ_REPLY
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_device_op_read_reply_t {
    uint32_t request_id;
    uint8_t result;
    uint8_t regstart;
    uint8_t count;
    uint8_t data[128];
    uint8_t bank;
}) fmav_device_op_read_reply_t;


#define FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY  11001


#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MIN  135
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX  136
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN  136
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA  15

#define FASTMAVLINK_MSG_ID_11001_LEN_MIN  135
#define FASTMAVLINK_MSG_ID_11001_LEN_MAX  136
#define FASTMAVLINK_MSG_ID_11001_LEN  136
#define FASTMAVLINK_MSG_ID_11001_CRCEXTRA  15

#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_FIELD_DATA_LEN  128

#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_FLAGS  0
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_11001_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_11001_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message DEVICE_OP_READ_REPLY packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_reply_t* _payload = (fmav_device_op_read_reply_t*)msg->payload;

    _payload->request_id = request_id;
    _payload->result = result;
    _payload->regstart = regstart;
    _payload->count = count;
    _payload->bank = bank;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*128);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_read_reply_pack(
        msg, sysid, compid,
        _payload->request_id, _payload->result, _payload->regstart, _payload->count, _payload->data, _payload->bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_reply_t* _payload = (fmav_device_op_read_reply_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->request_id = request_id;
    _payload->result = result;
    _payload->regstart = regstart;
    _payload->count = count;
    _payload->bank = bank;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*128);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_read_reply_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->request_id, _payload->result, _payload->regstart, _payload->count, _payload->data, _payload->bank,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_reply_t _payload;

    _payload.request_id = request_id;
    _payload.result = result;
    _payload.regstart = regstart;
    _payload.count = count;
    _payload.bank = bank;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*128);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_reply_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_READ_REPLY,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEVICE_OP_READ_REPLY unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_device_op_read_reply_decode(fmav_device_op_read_reply_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY  11001

#define mavlink_device_op_read_reply_t  fmav_device_op_read_reply_t

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_LEN  136
#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN  135
#define MAVLINK_MSG_ID_11001_LEN  136
#define MAVLINK_MSG_ID_11001_MIN_LEN  135

#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_CRC  15
#define MAVLINK_MSG_ID_11001_CRC  15

#define MAVLINK_MSG_DEVICE_OP_READ_REPLY_FIELD_DATA_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_read_reply_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_device_op_read_reply_pack(
        msg, sysid, compid,
        request_id, result, regstart, count, data, bank,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_read_reply_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank)
{
    return fmav_msg_device_op_read_reply_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        request_id, result, regstart, count, data, bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_device_op_read_reply_decode(const mavlink_message_t* msg, mavlink_device_op_read_reply_t* payload)
{
    fmav_msg_device_op_read_reply_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEVICE_OP_READ_REPLY_H

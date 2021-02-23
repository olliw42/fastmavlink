//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA64_H
#define FASTMAVLINK_MSG_DATA64_H


//----------------------------------------
//-- Message DATA64
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data64_t {
    uint8_t type;
    uint8_t len;
    uint8_t data[64];
}) fmav_data64_t;


#define FASTMAVLINK_MSG_ID_DATA64  171


#define FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MIN  66
#define FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX  66
#define FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN  66
#define FASTMAVLINK_MSG_DATA64_CRCEXTRA  181

#define FASTMAVLINK_MSG_ID_171_LEN_MIN  66
#define FASTMAVLINK_MSG_ID_171_LEN_MAX  66
#define FASTMAVLINK_MSG_ID_171_LEN  66
#define FASTMAVLINK_MSG_ID_171_CRCEXTRA  181

#define FASTMAVLINK_MSG_DATA64_FIELD_DATA_LEN  64

#define FASTMAVLINK_MSG_DATA64_FLAGS  0
#define FASTMAVLINK_MSG_DATA64_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA64_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message DATA64 packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data64_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data64_t* _payload = (fmav_data64_t*)msg->payload;

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*64);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DATA64;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DATA64_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data64_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data64_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data64_pack(
        msg, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data64_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data64_t* _payload = (fmav_data64_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*64);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA64;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA64 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA64 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA64_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data64_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data64_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data64_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


//----------------------------------------
//-- Message DATA64 unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data64_decode(fmav_data64_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DATA64_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA64  171

#define mavlink_data64_t  fmav_data64_t

#define MAVLINK_MSG_ID_DATA64_LEN  66
#define MAVLINK_MSG_ID_DATA64_MIN_LEN  66
#define MAVLINK_MSG_ID_171_LEN  66
#define MAVLINK_MSG_ID_171_MIN_LEN  66

#define MAVLINK_MSG_ID_DATA64_CRC  181
#define MAVLINK_MSG_ID_171_CRC  181

#define MAVLINK_MSG_DATA64_FIELD_DATA_LEN 64


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data64_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data64_pack(
        msg, sysid, compid,
        type, len, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data64_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    return fmav_msg_data64_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        type, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data64_decode(const mavlink_message_t* msg, mavlink_data64_t* payload)
{
    fmav_msg_data64_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA64_H

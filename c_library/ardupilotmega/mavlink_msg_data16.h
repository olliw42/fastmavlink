//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA16_H
#define FASTMAVLINK_MSG_DATA16_H


//----------------------------------------
//-- Message DATA16
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data16_t {
    uint8_t type;
    uint8_t len;
    uint8_t data[16];
}) fmav_data16_t;


#define FASTMAVLINK_MSG_ID_DATA16  169


#define FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MIN  18
#define FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX  18
#define FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN  18
#define FASTMAVLINK_MSG_DATA16_CRCEXTRA  234

#define FASTMAVLINK_MSG_ID_169_LEN_MIN  18
#define FASTMAVLINK_MSG_ID_169_LEN_MAX  18
#define FASTMAVLINK_MSG_ID_169_LEN  18
#define FASTMAVLINK_MSG_ID_169_CRCEXTRA  234

#define FASTMAVLINK_MSG_DATA16_FIELD_DATA_LEN  16

#define FASTMAVLINK_MSG_DATA16_FLAGS  0
#define FASTMAVLINK_MSG_DATA16_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA16_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message DATA16 packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data16_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data16_t* _payload = (fmav_data16_t*)msg->payload;

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DATA16;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DATA16_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data16_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data16_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data16_pack(
        msg, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data16_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_data16_t* _payload = (fmav_data16_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->type = type;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA16;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA16 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA16 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA16_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data16_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data16_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data16_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->type, _payload->len, _payload->data,
        _status);
}


//----------------------------------------
//-- Message DATA16 unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data16_decode(fmav_data16_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DATA16_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA16  169

#define mavlink_data16_t  fmav_data16_t

#define MAVLINK_MSG_ID_DATA16_LEN  18
#define MAVLINK_MSG_ID_DATA16_MIN_LEN  18
#define MAVLINK_MSG_ID_169_LEN  18
#define MAVLINK_MSG_ID_169_MIN_LEN  18

#define MAVLINK_MSG_ID_DATA16_CRC  234
#define MAVLINK_MSG_ID_169_CRC  234

#define MAVLINK_MSG_DATA16_FIELD_DATA_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data16_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data16_pack(
        msg, sysid, compid,
        type, len, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data16_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint8_t len, const uint8_t* data)
{
    return fmav_msg_data16_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        type, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data16_decode(const mavlink_message_t* msg, mavlink_data16_t* payload)
{
    fmav_msg_data16_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA16_H

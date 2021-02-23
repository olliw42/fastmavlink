//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ENCAPSULATED_DATA_H
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_H


//----------------------------------------
//-- Message ENCAPSULATED_DATA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_encapsulated_data_t {
    uint16_t seqnr;
    uint8_t data[253];
}) fmav_encapsulated_data_t;


#define FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA  131


#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MIN  255
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX  255
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN  255
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_CRCEXTRA  223

#define FASTMAVLINK_MSG_ID_131_LEN_MIN  255
#define FASTMAVLINK_MSG_ID_131_LEN_MAX  255
#define FASTMAVLINK_MSG_ID_131_LEN  255
#define FASTMAVLINK_MSG_ID_131_CRCEXTRA  223

#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN  253

#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_FLAGS  0
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ENCAPSULATED_DATA_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ENCAPSULATED_DATA packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_encapsulated_data_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seqnr, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_encapsulated_data_t* _payload = (fmav_encapsulated_data_t*)msg->payload;

    _payload->seqnr = seqnr;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*253);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ENCAPSULATED_DATA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_encapsulated_data_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_encapsulated_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_encapsulated_data_pack(
        msg, sysid, compid,
        _payload->seqnr, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_encapsulated_data_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seqnr, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_encapsulated_data_t* _payload = (fmav_encapsulated_data_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->seqnr = seqnr;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*253);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ENCAPSULATED_DATA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ENCAPSULATED_DATA_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_encapsulated_data_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_encapsulated_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_encapsulated_data_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->seqnr, _payload->data,
        _status);
}


//----------------------------------------
//-- Message ENCAPSULATED_DATA unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_encapsulated_data_decode(fmav_encapsulated_data_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ENCAPSULATED_DATA_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ENCAPSULATED_DATA  131

#define mavlink_encapsulated_data_t  fmav_encapsulated_data_t

#define MAVLINK_MSG_ID_ENCAPSULATED_DATA_LEN  255
#define MAVLINK_MSG_ID_ENCAPSULATED_DATA_MIN_LEN  255
#define MAVLINK_MSG_ID_131_LEN  255
#define MAVLINK_MSG_ID_131_MIN_LEN  255

#define MAVLINK_MSG_ID_ENCAPSULATED_DATA_CRC  223
#define MAVLINK_MSG_ID_131_CRC  223

#define MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN 253


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_encapsulated_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t seqnr, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_encapsulated_data_pack(
        msg, sysid, compid,
        seqnr, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_encapsulated_data_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seqnr, const uint8_t* data)
{
    return fmav_msg_encapsulated_data_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        seqnr, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_encapsulated_data_decode(const mavlink_message_t* msg, mavlink_encapsulated_data_t* payload)
{
    fmav_msg_encapsulated_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ENCAPSULATED_DATA_H

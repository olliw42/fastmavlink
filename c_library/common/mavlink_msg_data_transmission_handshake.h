//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data_transmission_handshake_t {
    uint32_t size;
    uint16_t width;
    uint16_t height;
    uint16_t packets;
    uint8_t type;
    uint8_t payload;
    uint8_t jpg_quality;
}) fmav_data_transmission_handshake_t;


#define FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE  130


#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MIN  13
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN  13
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA  29

#define FASTMAVLINK_MSG_ID_130_LEN_MIN  13
#define FASTMAVLINK_MSG_ID_130_LEN_MAX  13
#define FASTMAVLINK_MSG_ID_130_LEN  13
#define FASTMAVLINK_MSG_ID_130_CRCEXTRA  29



#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FLAGS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t* _payload = (fmav_data_transmission_handshake_t*)msg->payload;

    _payload->size = size;
    _payload->width = width;
    _payload->height = height;
    _payload->packets = packets;
    _payload->type = type;
    _payload->payload = payload;
    _payload->jpg_quality = jpg_quality;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_transmission_handshake_pack(
        msg, sysid, compid,
        _payload->type, _payload->size, _payload->width, _payload->height, _payload->packets, _payload->payload, _payload->jpg_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t* _payload = (fmav_data_transmission_handshake_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->size = size;
    _payload->width = width;
    _payload->height = height;
    _payload->packets = packets;
    _payload->type = type;
    _payload->payload = payload;
    _payload->jpg_quality = jpg_quality;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_transmission_handshake_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->type, _payload->size, _payload->width, _payload->height, _payload->packets, _payload->payload, _payload->jpg_quality,
        _status);
}


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_transmission_handshake_decode(fmav_data_transmission_handshake_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE  130

#define mavlink_data_transmission_handshake_t  fmav_data_transmission_handshake_t

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_LEN  13
#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_MIN_LEN  13
#define MAVLINK_MSG_ID_130_LEN  13
#define MAVLINK_MSG_ID_130_MIN_LEN  13

#define MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE_CRC  29
#define MAVLINK_MSG_ID_130_CRC  29




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_transmission_handshake_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data_transmission_handshake_pack(
        msg, sysid, compid,
        type, size, width, height, packets, payload, jpg_quality,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_transmission_handshake_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
    return fmav_msg_data_transmission_handshake_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        type, size, width, height, packets, payload, jpg_quality,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data_transmission_handshake_decode(const mavlink_message_t* msg, mavlink_data_transmission_handshake_t* payload)
{
    fmav_msg_data_transmission_handshake_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_H

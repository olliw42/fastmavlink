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

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA  29

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FLAGS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FRAME_LEN_MAX  38



#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_SIZE_OFS  0
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_WIDTH_OFS  4
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_HEIGHT_OFS  6
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_PACKETS_OFS  8
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_TYPE_OFS  10
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_PAYLOAD_OFS  11
#define FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_FIELD_JPG_QUALITY_OFS  12


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality,
    fmav_status_t* _status)
{
    fmav_data_transmission_handshake_t _payload;

    _payload.size = size;
    _payload.width = width;
    _payload.height = height;
    _payload.packets = packets;
    _payload.type = type;
    _payload.payload = payload;
    _payload.jpg_quality = jpg_quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_transmission_handshake_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DATA_TRANSMISSION_HANDSHAKE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_data_transmission_handshake_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_data_transmission_handshake_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_transmission_handshake_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_transmission_handshake_decode(fmav_data_transmission_handshake_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_TRANSMISSION_HANDSHAKE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_data_transmission_handshake_get_field_size(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_width(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_height(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_transmission_handshake_get_field_packets(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_payload(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_transmission_handshake_get_field_jpg_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOGGING_DATA_ACKED_H
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_H


//----------------------------------------
//-- Message LOGGING_DATA_ACKED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_logging_data_acked_t {
    uint16_t sequence;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t length;
    uint8_t first_message_offset;
    uint8_t data[249];
}) fmav_logging_data_acked_t;


#define FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED  267


#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MIN  255
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX  255
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN  255
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA  35

#define FASTMAVLINK_MSG_ID_267_LEN_MIN  255
#define FASTMAVLINK_MSG_ID_267_LEN_MAX  255
#define FASTMAVLINK_MSG_ID_267_LEN  255
#define FASTMAVLINK_MSG_ID_267_CRCEXTRA  35

#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN  249

#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_FLAGS  3
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_LOGGING_DATA_ACKED_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_267_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_267_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message LOGGING_DATA_ACKED packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t length, uint8_t first_message_offset, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_logging_data_acked_t* _payload = (fmav_logging_data_acked_t*)msg->payload;

    _payload->sequence = sequence;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->length = length;
    _payload->first_message_offset = first_message_offset;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*249);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_logging_data_acked_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_logging_data_acked_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->sequence, _payload->length, _payload->first_message_offset, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t length, uint8_t first_message_offset, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_logging_data_acked_t* _payload = (fmav_logging_data_acked_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->sequence = sequence;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->length = length;
    _payload->first_message_offset = first_message_offset;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*249);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_logging_data_acked_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_logging_data_acked_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->sequence, _payload->length, _payload->first_message_offset, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t length, uint8_t first_message_offset, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_logging_data_acked_t _payload;

    _payload.sequence = sequence;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.length = length;
    _payload.first_message_offset = first_message_offset;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*249);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_logging_data_acked_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_logging_data_acked_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOGGING_DATA_ACKED,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOGGING_DATA_ACKED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOGGING_DATA_ACKED unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_logging_data_acked_decode(fmav_logging_data_acked_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOGGING_DATA_ACKED_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOGGING_DATA_ACKED  267

#define mavlink_logging_data_acked_t  fmav_logging_data_acked_t

#define MAVLINK_MSG_ID_LOGGING_DATA_ACKED_LEN  255
#define MAVLINK_MSG_ID_LOGGING_DATA_ACKED_MIN_LEN  255
#define MAVLINK_MSG_ID_267_LEN  255
#define MAVLINK_MSG_ID_267_MIN_LEN  255

#define MAVLINK_MSG_ID_LOGGING_DATA_ACKED_CRC  35
#define MAVLINK_MSG_ID_267_CRC  35

#define MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN 249


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_logging_data_acked_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t length, uint8_t first_message_offset, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_logging_data_acked_pack(
        msg, sysid, compid,
        target_system, target_component, sequence, length, first_message_offset, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_logging_data_acked_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t length, uint8_t first_message_offset, const uint8_t* data)
{
    return fmav_msg_logging_data_acked_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, sequence, length, first_message_offset, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_logging_data_acked_decode(const mavlink_message_t* msg, mavlink_logging_data_acked_t* payload)
{
    fmav_msg_logging_data_acked_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOGGING_DATA_ACKED_H

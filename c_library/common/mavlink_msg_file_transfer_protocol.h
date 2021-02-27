//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_H
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_H


//----------------------------------------
//-- Message FILE_TRANSFER_PROTOCOL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_file_transfer_protocol_t {
    uint8_t target_network;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t payload[251];
}) fmav_file_transfer_protocol_t;


#define FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL  110


#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MIN  254
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX  254
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN  254
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA  84

#define FASTMAVLINK_MSG_ID_110_LEN_MIN  254
#define FASTMAVLINK_MSG_ID_110_LEN_MAX  254
#define FASTMAVLINK_MSG_ID_110_LEN  254
#define FASTMAVLINK_MSG_ID_110_CRCEXTRA  84

#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN  251

#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FLAGS  3
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_TARGET_SYSTEM_OFS  1
#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_TARGET_COMPONENT_OFS  2

#define FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_110_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_110_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message FILE_TRANSFER_PROTOCOL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_file_transfer_protocol_t* _payload = (fmav_file_transfer_protocol_t*)msg->payload;

    _payload->target_network = target_network;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*251);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_file_transfer_protocol_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_file_transfer_protocol_pack(
        msg, sysid, compid,
        _payload->target_network, _payload->target_system, _payload->target_component, _payload->payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_file_transfer_protocol_t* _payload = (fmav_file_transfer_protocol_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_network = target_network;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*251);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_file_transfer_protocol_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_file_transfer_protocol_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_network, _payload->target_system, _payload->target_component, _payload->payload,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_file_transfer_protocol_t _payload;

    _payload.target_network = target_network;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.payload), payload, sizeof(uint8_t)*251);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_file_transfer_protocol_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_file_transfer_protocol_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FILE_TRANSFER_PROTOCOL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_file_transfer_protocol_decode(fmav_file_transfer_protocol_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL  110

#define mavlink_file_transfer_protocol_t  fmav_file_transfer_protocol_t

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN  254
#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN  254
#define MAVLINK_MSG_ID_110_LEN  254
#define MAVLINK_MSG_ID_110_MIN_LEN  254

#define MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_CRC  84
#define MAVLINK_MSG_ID_110_CRC  84

#define MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN 251


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_file_transfer_protocol_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t* payload)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_file_transfer_protocol_pack(
        msg, sysid, compid,
        target_network, target_system, target_component, payload,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_file_transfer_protocol_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, const uint8_t* payload)
{
    return fmav_msg_file_transfer_protocol_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_network, target_system, target_component, payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_file_transfer_protocol_decode(const mavlink_message_t* msg, mavlink_file_transfer_protocol_t* payload)
{
    fmav_msg_file_transfer_protocol_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FILE_TRANSFER_PROTOCOL_H

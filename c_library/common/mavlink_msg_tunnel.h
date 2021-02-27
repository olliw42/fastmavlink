//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TUNNEL_H
#define FASTMAVLINK_MSG_TUNNEL_H


//----------------------------------------
//-- Message TUNNEL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_tunnel_t {
    uint16_t payload_type;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t payload_length;
    uint8_t payload[128];
}) fmav_tunnel_t;


#define FASTMAVLINK_MSG_ID_TUNNEL  385


#define FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MIN  133
#define FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX  133
#define FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN  133
#define FASTMAVLINK_MSG_TUNNEL_CRCEXTRA  147

#define FASTMAVLINK_MSG_ID_385_LEN_MIN  133
#define FASTMAVLINK_MSG_ID_385_LEN_MAX  133
#define FASTMAVLINK_MSG_ID_385_LEN  133
#define FASTMAVLINK_MSG_ID_385_CRCEXTRA  147

#define FASTMAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN  128

#define FASTMAVLINK_MSG_TUNNEL_FLAGS  3
#define FASTMAVLINK_MSG_TUNNEL_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_TUNNEL_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_TUNNEL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_385_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_385_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message TUNNEL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_tunnel_t* _payload = (fmav_tunnel_t*)msg->payload;

    _payload->payload_type = payload_type;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->payload_length = payload_length;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*128);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TUNNEL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_TUNNEL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_tunnel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_tunnel_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->payload_type, _payload->payload_length, _payload->payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_tunnel_t* _payload = (fmav_tunnel_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->payload_type = payload_type;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->payload_length = payload_length;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*128);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TUNNEL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TUNNEL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TUNNEL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TUNNEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_tunnel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_tunnel_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->payload_type, _payload->payload_length, _payload->payload,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_tunnel_t _payload;

    _payload.payload_type = payload_type;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.payload_length = payload_length;
    memcpy(&(_payload.payload), payload, sizeof(uint8_t)*128);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TUNNEL,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TUNNEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_tunnel_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_tunnel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TUNNEL,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TUNNEL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TUNNEL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_tunnel_decode(fmav_tunnel_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TUNNEL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TUNNEL  385

#define mavlink_tunnel_t  fmav_tunnel_t

#define MAVLINK_MSG_ID_TUNNEL_LEN  133
#define MAVLINK_MSG_ID_TUNNEL_MIN_LEN  133
#define MAVLINK_MSG_ID_385_LEN  133
#define MAVLINK_MSG_ID_385_MIN_LEN  133

#define MAVLINK_MSG_ID_TUNNEL_CRC  147
#define MAVLINK_MSG_ID_385_CRC  147

#define MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_tunnel_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t* payload)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_tunnel_pack(
        msg, sysid, compid,
        target_system, target_component, payload_type, payload_length, payload,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_tunnel_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t payload_type, uint8_t payload_length, const uint8_t* payload)
{
    return fmav_msg_tunnel_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, payload_type, payload_length, payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_tunnel_decode(const mavlink_message_t* msg, mavlink_tunnel_t* payload)
{
    fmav_msg_tunnel_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TUNNEL_H

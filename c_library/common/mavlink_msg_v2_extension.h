//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_V2_EXTENSION_H
#define FASTMAVLINK_MSG_V2_EXTENSION_H


//----------------------------------------
//-- Message V2_EXTENSION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_v2_extension_t {
    uint16_t message_type;
    uint8_t target_network;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t payload[249];
}) fmav_v2_extension_t;


#define FASTMAVLINK_MSG_ID_V2_EXTENSION  248

#define FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX  254
#define FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA  8

#define FASTMAVLINK_MSG_V2_EXTENSION_FLAGS  3
#define FASTMAVLINK_MSG_V2_EXTENSION_TARGET_SYSTEM_OFS  3
#define FASTMAVLINK_MSG_V2_EXTENSION_TARGET_COMPONENT_OFS  4

#define FASTMAVLINK_MSG_V2_EXTENSION_FRAME_LEN_MAX  279

#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_NUM  249 // number of elements in array
#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN  249 // length of array = number of bytes

#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_MESSAGE_TYPE_OFS  0
#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_TARGET_NETWORK_OFS  2
#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_TARGET_SYSTEM_OFS  3
#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_TARGET_COMPONENT_OFS  4
#define FASTMAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_OFS  5


//----------------------------------------
//-- Message V2_EXTENSION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_v2_extension_t* _payload = (fmav_v2_extension_t*)msg->payload;

    _payload->message_type = message_type;
    _payload->target_network = target_network;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*249);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_V2_EXTENSION;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_v2_extension_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_v2_extension_pack(
        msg, sysid, compid,
        _payload->target_network, _payload->target_system, _payload->target_component, _payload->message_type, _payload->payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_v2_extension_t* _payload = (fmav_v2_extension_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->message_type = message_type;
    _payload->target_network = target_network;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->payload), payload, sizeof(uint8_t)*249);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_V2_EXTENSION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_V2_EXTENSION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_V2_EXTENSION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_v2_extension_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_v2_extension_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_network, _payload->target_system, _payload->target_component, _payload->message_type, _payload->payload,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t* payload,
    fmav_status_t* _status)
{
    fmav_v2_extension_t _payload;

    _payload.message_type = message_type;
    _payload.target_network = target_network;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.payload), payload, sizeof(uint8_t)*249);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_V2_EXTENSION,
        FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_v2_extension_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_V2_EXTENSION,
        FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_V2_EXTENSION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message V2_EXTENSION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_v2_extension_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_v2_extension_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_v2_extension_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_v2_extension_decode(fmav_v2_extension_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_V2_EXTENSION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_v2_extension_get_field_message_type(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_v2_extension_get_field_target_network(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_v2_extension_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_v2_extension_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_v2_extension_get_field_payload_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[5]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_v2_extension_get_field_payload(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_NUM) return 0;
    return ((uint8_t*)&(msg->payload[5]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_V2_EXTENSION  248

#define mavlink_v2_extension_t  fmav_v2_extension_t

#define MAVLINK_MSG_ID_V2_EXTENSION_LEN  254
#define MAVLINK_MSG_ID_V2_EXTENSION_MIN_LEN  254
#define MAVLINK_MSG_ID_248_LEN  254
#define MAVLINK_MSG_ID_248_MIN_LEN  254

#define MAVLINK_MSG_ID_V2_EXTENSION_CRC  8
#define MAVLINK_MSG_ID_248_CRC  8

#define MAVLINK_MSG_V2_EXTENSION_FIELD_PAYLOAD_LEN 249


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_v2_extension_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t* payload)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_v2_extension_pack(
        msg, sysid, compid,
        target_network, target_system, target_component, message_type, payload,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_v2_extension_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_network, uint8_t target_system, uint8_t target_component, uint16_t message_type, const uint8_t* payload)
{
    return fmav_msg_v2_extension_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_network, target_system, target_component, message_type, payload,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_v2_extension_decode(const mavlink_message_t* msg, mavlink_v2_extension_t* payload)
{
    fmav_msg_v2_extension_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_V2_EXTENSION_H

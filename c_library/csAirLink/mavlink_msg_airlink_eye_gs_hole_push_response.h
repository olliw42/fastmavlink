//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_H
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_H


//----------------------------------------
//-- Message AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_airlink_eye_gs_hole_push_response_t {
    uint32_t ip_port;
    uint8_t resp_type;
    uint8_t ip_version;
    uint8_t ip_address_4[4];
    uint8_t ip_address_6[16];
}) fmav_airlink_eye_gs_hole_push_response_t;


#define FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE  52003

#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA  166

#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FLAGS  0
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FRAME_LEN_MAX  51

#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_PORT_OFS  0
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_RESP_TYPE_OFS  4
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_VERSION_OFS  5
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_OFS  6
#define FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_OFS  10


//----------------------------------------
//-- Message AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t resp_type, uint8_t ip_version, const uint8_t* ip_address_4, const uint8_t* ip_address_6, uint32_t ip_port,
    fmav_status_t* _status)
{
    fmav_airlink_eye_gs_hole_push_response_t* _payload = (fmav_airlink_eye_gs_hole_push_response_t*)_msg->payload;

    _payload->ip_port = ip_port;
    _payload->resp_type = resp_type;
    _payload->ip_version = ip_version;
    memcpy(&(_payload->ip_address_4), ip_address_4, sizeof(uint8_t)*4);
    memcpy(&(_payload->ip_address_6), ip_address_6, sizeof(uint8_t)*16);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airlink_eye_gs_hole_push_response_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airlink_eye_gs_hole_push_response_pack(
        _msg, sysid, compid,
        _payload->resp_type, _payload->ip_version, _payload->ip_address_4, _payload->ip_address_6, _payload->ip_port,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t resp_type, uint8_t ip_version, const uint8_t* ip_address_4, const uint8_t* ip_address_6, uint32_t ip_port,
    fmav_status_t* _status)
{
    fmav_airlink_eye_gs_hole_push_response_t* _payload = (fmav_airlink_eye_gs_hole_push_response_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ip_port = ip_port;
    _payload->resp_type = resp_type;
    _payload->ip_version = ip_version;
    memcpy(&(_payload->ip_address_4), ip_address_4, sizeof(uint8_t)*4);
    memcpy(&(_payload->ip_address_6), ip_address_6, sizeof(uint8_t)*16);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airlink_eye_gs_hole_push_response_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airlink_eye_gs_hole_push_response_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->resp_type, _payload->ip_version, _payload->ip_address_4, _payload->ip_address_6, _payload->ip_port,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t resp_type, uint8_t ip_version, const uint8_t* ip_address_4, const uint8_t* ip_address_6, uint32_t ip_port,
    fmav_status_t* _status)
{
    fmav_airlink_eye_gs_hole_push_response_t _payload;

    _payload.ip_port = ip_port;
    _payload.resp_type = resp_type;
    _payload.ip_version = ip_version;
    memcpy(&(_payload.ip_address_4), ip_address_4, sizeof(uint8_t)*4);
    memcpy(&(_payload.ip_address_6), ip_address_6, sizeof(uint8_t)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airlink_eye_gs_hole_push_response_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_airlink_eye_gs_hole_push_response_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airlink_eye_gs_hole_push_response_decode(fmav_airlink_eye_gs_hole_push_response_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_port(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airlink_eye_gs_hole_push_response_get_field_resp_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_version(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_address_4_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_address_4(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_NUM) return 0;
    return ((uint8_t*)&(msg->payload[6]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_address_6_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[10]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airlink_eye_gs_hole_push_response_get_field_ip_address_6(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_NUM) return 0;
    return ((uint8_t*)&(msg->payload[10]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE  52003

#define mavlink_airlink_eye_gs_hole_push_response_t  fmav_airlink_eye_gs_hole_push_response_t

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_LEN  26
#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_MIN_LEN  26
#define MAVLINK_MSG_ID_52003_LEN  26
#define MAVLINK_MSG_ID_52003_MIN_LEN  26

#define MAVLINK_MSG_ID_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_CRC  166
#define MAVLINK_MSG_ID_52003_CRC  166

#define MAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_4_LEN 4
#define MAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_FIELD_IP_ADDRESS_6_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t resp_type, uint8_t ip_version, const uint8_t* ip_address_4, const uint8_t* ip_address_6, uint32_t ip_port)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_airlink_eye_gs_hole_push_response_pack(
        _msg, sysid, compid,
        resp_type, ip_version, ip_address_4, ip_address_6, ip_port,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_airlink_eye_gs_hole_push_response_t* _payload)
{
    return mavlink_msg_airlink_eye_gs_hole_push_response_pack(
        sysid,
        compid,
        _msg,
        _payload->resp_type, _payload->ip_version, _payload->ip_address_4, _payload->ip_address_6, _payload->ip_port);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airlink_eye_gs_hole_push_response_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t resp_type, uint8_t ip_version, const uint8_t* ip_address_4, const uint8_t* ip_address_6, uint32_t ip_port)
{
    return fmav_msg_airlink_eye_gs_hole_push_response_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        resp_type, ip_version, ip_address_4, ip_address_6, ip_port,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_airlink_eye_gs_hole_push_response_decode(const mavlink_message_t* msg, mavlink_airlink_eye_gs_hole_push_response_t* payload)
{
    fmav_msg_airlink_eye_gs_hole_push_response_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIRLINK_EYE_GS_HOLE_PUSH_RESPONSE_H

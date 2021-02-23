//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LED_CONTROL_H
#define FASTMAVLINK_MSG_LED_CONTROL_H


//----------------------------------------
//-- Message LED_CONTROL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_led_control_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t instance;
    uint8_t pattern;
    uint8_t custom_len;
    uint8_t custom_bytes[24];
}) fmav_led_control_t;


#define FASTMAVLINK_MSG_ID_LED_CONTROL  186


#define FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MIN  29
#define FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX  29
#define FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN  29
#define FASTMAVLINK_MSG_LED_CONTROL_CRCEXTRA  72

#define FASTMAVLINK_MSG_ID_186_LEN_MIN  29
#define FASTMAVLINK_MSG_ID_186_LEN_MAX  29
#define FASTMAVLINK_MSG_ID_186_LEN  29
#define FASTMAVLINK_MSG_ID_186_CRCEXTRA  72

#define FASTMAVLINK_MSG_LED_CONTROL_FIELD_CUSTOM_BYTES_LEN  24

#define FASTMAVLINK_MSG_LED_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_LED_CONTROL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LED_CONTROL_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message LED_CONTROL packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t custom_len, const uint8_t* custom_bytes,
    fmav_status_t* _status)
{
    fmav_led_control_t* _payload = (fmav_led_control_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->instance = instance;
    _payload->pattern = pattern;
    _payload->custom_len = custom_len;
    memcpy(&(_payload->custom_bytes), custom_bytes, sizeof(uint8_t)*24);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LED_CONTROL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_LED_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_led_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_led_control_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->instance, _payload->pattern, _payload->custom_len, _payload->custom_bytes,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t custom_len, const uint8_t* custom_bytes,
    fmav_status_t* _status)
{
    fmav_led_control_t* _payload = (fmav_led_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->instance = instance;
    _payload->pattern = pattern;
    _payload->custom_len = custom_len;
    memcpy(&(_payload->custom_bytes), custom_bytes, sizeof(uint8_t)*24);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LED_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LED_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LED_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LED_CONTROL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_led_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_led_control_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->instance, _payload->pattern, _payload->custom_len, _payload->custom_bytes,
        _status);
}


//----------------------------------------
//-- Message LED_CONTROL unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_led_control_decode(fmav_led_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LED_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LED_CONTROL  186

#define mavlink_led_control_t  fmav_led_control_t

#define MAVLINK_MSG_ID_LED_CONTROL_LEN  29
#define MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN  29
#define MAVLINK_MSG_ID_186_LEN  29
#define MAVLINK_MSG_ID_186_MIN_LEN  29

#define MAVLINK_MSG_ID_LED_CONTROL_CRC  72
#define MAVLINK_MSG_ID_186_CRC  72

#define MAVLINK_MSG_LED_CONTROL_FIELD_CUSTOM_BYTES_LEN 24


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_led_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t custom_len, const uint8_t* custom_bytes)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_led_control_pack(
        msg, sysid, compid,
        target_system, target_component, instance, pattern, custom_len, custom_bytes,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_led_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t instance, uint8_t pattern, uint8_t custom_len, const uint8_t* custom_bytes)
{
    return fmav_msg_led_control_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, instance, pattern, custom_len, custom_bytes,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_led_control_decode(const mavlink_message_t* msg, mavlink_led_control_t* payload)
{
    fmav_msg_led_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LED_CONTROL_H

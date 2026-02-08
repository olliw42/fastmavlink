//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LED_STRIP_STATE_H
#define FASTMAVLINK_MSG_LED_STRIP_STATE_H


//----------------------------------------
//-- Message LED_STRIP_STATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_led_strip_state_t {
    uint32_t colors[8];
    uint8_t length;
    uint8_t index;
    uint8_t id;
    uint8_t following_flight_mode;
}) fmav_led_strip_state_t;


#define FASTMAVLINK_MSG_ID_LED_STRIP_STATE  52601

#define FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX  36
#define FASTMAVLINK_MSG_LED_STRIP_STATE_CRCEXTRA  102

#define FASTMAVLINK_MSG_LED_STRIP_STATE_FLAGS  0
#define FASTMAVLINK_MSG_LED_STRIP_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LED_STRIP_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LED_STRIP_STATE_FRAME_LEN_MAX  61

#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_COLORS_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_COLORS_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_COLORS_OFS  0
#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_LENGTH_OFS  32
#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_INDEX_OFS  33
#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_ID_OFS  34
#define FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_FOLLOWING_FLIGHT_MODE_OFS  35


//----------------------------------------
//-- Message LED_STRIP_STATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t length, uint8_t index, uint8_t id, uint8_t following_flight_mode, const uint32_t* colors,
    fmav_status_t* _status)
{
    fmav_led_strip_state_t* _payload = (fmav_led_strip_state_t*)_msg->payload;

    _payload->length = length;
    _payload->index = index;
    _payload->id = id;
    _payload->following_flight_mode = following_flight_mode;
    memcpy(&(_payload->colors), colors, sizeof(uint32_t)*8);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LED_STRIP_STATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_LED_STRIP_STATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_led_strip_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_led_strip_state_pack(
        _msg, sysid, compid,
        _payload->length, _payload->index, _payload->id, _payload->following_flight_mode, _payload->colors,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t length, uint8_t index, uint8_t id, uint8_t following_flight_mode, const uint32_t* colors,
    fmav_status_t* _status)
{
    fmav_led_strip_state_t* _payload = (fmav_led_strip_state_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->length = length;
    _payload->index = index;
    _payload->id = id;
    _payload->following_flight_mode = following_flight_mode;
    memcpy(&(_payload->colors), colors, sizeof(uint32_t)*8);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LED_STRIP_STATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LED_STRIP_STATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LED_STRIP_STATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LED_STRIP_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_led_strip_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_led_strip_state_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->length, _payload->index, _payload->id, _payload->following_flight_mode, _payload->colors,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t length, uint8_t index, uint8_t id, uint8_t following_flight_mode, const uint32_t* colors,
    fmav_status_t* _status)
{
    fmav_led_strip_state_t _payload;

    _payload.length = length;
    _payload.index = index;
    _payload.id = id;
    _payload.following_flight_mode = following_flight_mode;
    memcpy(&(_payload.colors), colors, sizeof(uint32_t)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LED_STRIP_STATE,
        FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LED_STRIP_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_led_strip_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_led_strip_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LED_STRIP_STATE,
        FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LED_STRIP_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LED_STRIP_STATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_led_strip_state_decode(fmav_led_strip_state_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LED_STRIP_STATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_led_strip_state_get_field_length(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_led_strip_state_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_led_strip_state_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_led_strip_state_get_field_following_flight_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_led_strip_state_get_field_colors_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_led_strip_state_get_field_colors(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_LED_STRIP_STATE_FIELD_COLORS_NUM) return 0;
    return ((uint32_t*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LED_STRIP_STATE  52601

#define mavlink_led_strip_state_t  fmav_led_strip_state_t

#define MAVLINK_MSG_ID_LED_STRIP_STATE_LEN  36
#define MAVLINK_MSG_ID_LED_STRIP_STATE_MIN_LEN  36
#define MAVLINK_MSG_ID_52601_LEN  36
#define MAVLINK_MSG_ID_52601_MIN_LEN  36

#define MAVLINK_MSG_ID_LED_STRIP_STATE_CRC  102
#define MAVLINK_MSG_ID_52601_CRC  102

#define MAVLINK_MSG_LED_STRIP_STATE_FIELD_COLORS_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_led_strip_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t length, uint8_t index, uint8_t id, uint8_t following_flight_mode, const uint32_t* colors)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_led_strip_state_pack(
        _msg, sysid, compid,
        length, index, id, following_flight_mode, colors,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_led_strip_state_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_led_strip_state_t* _payload)
{
    return mavlink_msg_led_strip_state_pack(
        sysid,
        compid,
        _msg,
        _payload->length, _payload->index, _payload->id, _payload->following_flight_mode, _payload->colors);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_led_strip_state_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t length, uint8_t index, uint8_t id, uint8_t following_flight_mode, const uint32_t* colors)
{
    return fmav_msg_led_strip_state_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        length, index, id, following_flight_mode, colors,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_led_strip_state_decode(const mavlink_message_t* msg, mavlink_led_strip_state_t* payload)
{
    fmav_msg_led_strip_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LED_STRIP_STATE_H

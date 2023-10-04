//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CURRENT_MODE_H
#define FASTMAVLINK_MSG_CURRENT_MODE_H


//----------------------------------------
//-- Message CURRENT_MODE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_current_mode_t {
    uint32_t custom_mode;
    uint32_t intended_custom_mode;
    uint8_t standard_mode;
}) fmav_current_mode_t;


#define FASTMAVLINK_MSG_ID_CURRENT_MODE  436

#define FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_CURRENT_MODE_CRCEXTRA  193

#define FASTMAVLINK_MSG_CURRENT_MODE_FLAGS  0
#define FASTMAVLINK_MSG_CURRENT_MODE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CURRENT_MODE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CURRENT_MODE_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_CURRENT_MODE_FIELD_CUSTOM_MODE_OFS  0
#define FASTMAVLINK_MSG_CURRENT_MODE_FIELD_INTENDED_CUSTOM_MODE_OFS  4
#define FASTMAVLINK_MSG_CURRENT_MODE_FIELD_STANDARD_MODE_OFS  8


//----------------------------------------
//-- Message CURRENT_MODE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode,
    fmav_status_t* _status)
{
    fmav_current_mode_t* _payload = (fmav_current_mode_t*)_msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->intended_custom_mode = intended_custom_mode;
    _payload->standard_mode = standard_mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CURRENT_MODE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_CURRENT_MODE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_current_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_current_mode_pack(
        _msg, sysid, compid,
        _payload->standard_mode, _payload->custom_mode, _payload->intended_custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode,
    fmav_status_t* _status)
{
    fmav_current_mode_t* _payload = (fmav_current_mode_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->intended_custom_mode = intended_custom_mode;
    _payload->standard_mode = standard_mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CURRENT_MODE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CURRENT_MODE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CURRENT_MODE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CURRENT_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_current_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_current_mode_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->standard_mode, _payload->custom_mode, _payload->intended_custom_mode,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode,
    fmav_status_t* _status)
{
    fmav_current_mode_t _payload;

    _payload.custom_mode = custom_mode;
    _payload.intended_custom_mode = intended_custom_mode;
    _payload.standard_mode = standard_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CURRENT_MODE,
        FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CURRENT_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_current_mode_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_current_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CURRENT_MODE,
        FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CURRENT_MODE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CURRENT_MODE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_current_mode_decode(fmav_current_mode_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CURRENT_MODE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_current_mode_get_field_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_current_mode_get_field_intended_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_current_mode_get_field_standard_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CURRENT_MODE  436

#define mavlink_current_mode_t  fmav_current_mode_t

#define MAVLINK_MSG_ID_CURRENT_MODE_LEN  9
#define MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN  9
#define MAVLINK_MSG_ID_436_LEN  9
#define MAVLINK_MSG_ID_436_MIN_LEN  9

#define MAVLINK_MSG_ID_CURRENT_MODE_CRC  193
#define MAVLINK_MSG_ID_436_CRC  193




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_current_mode_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_current_mode_pack(
        _msg, sysid, compid,
        standard_mode, custom_mode, intended_custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_current_mode_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_current_mode_t* _payload)
{
    return mavlink_msg_current_mode_pack(
        sysid,
        compid,
        _msg,
        _payload->standard_mode, _payload->custom_mode, _payload->intended_custom_mode);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_current_mode_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t standard_mode, uint32_t custom_mode, uint32_t intended_custom_mode)
{
    return fmav_msg_current_mode_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        standard_mode, custom_mode, intended_custom_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_current_mode_decode(const mavlink_message_t* msg, mavlink_current_mode_t* payload)
{
    fmav_msg_current_mode_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CURRENT_MODE_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AVAILABLE_MODES_H
#define FASTMAVLINK_MSG_AVAILABLE_MODES_H


//----------------------------------------
//-- Message AVAILABLE_MODES
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_available_modes_t {
    uint32_t custom_mode;
    uint32_t properties;
    uint8_t number_modes;
    uint8_t mode_index;
    uint8_t standard_mode;
    char mode_name[35];
}) fmav_available_modes_t;


#define FASTMAVLINK_MSG_ID_AVAILABLE_MODES  435

#define FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX  46
#define FASTMAVLINK_MSG_AVAILABLE_MODES_CRCEXTRA  134

#define FASTMAVLINK_MSG_AVAILABLE_MODES_FLAGS  0
#define FASTMAVLINK_MSG_AVAILABLE_MODES_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AVAILABLE_MODES_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AVAILABLE_MODES_FRAME_LEN_MAX  71

#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_NUM  35 // number of elements in array
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_LEN  35 // length of array = number of bytes

#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_CUSTOM_MODE_OFS  0
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_PROPERTIES_OFS  4
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_NUMBER_MODES_OFS  8
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_INDEX_OFS  9
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_STANDARD_MODE_OFS  10
#define FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_OFS  11


//----------------------------------------
//-- Message AVAILABLE_MODES pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char* mode_name,
    fmav_status_t* _status)
{
    fmav_available_modes_t* _payload = (fmav_available_modes_t*)_msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->properties = properties;
    _payload->number_modes = number_modes;
    _payload->mode_index = mode_index;
    _payload->standard_mode = standard_mode;
    memcpy(&(_payload->mode_name), mode_name, sizeof(char)*35);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AVAILABLE_MODES;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AVAILABLE_MODES_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_available_modes_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_available_modes_pack(
        _msg, sysid, compid,
        _payload->number_modes, _payload->mode_index, _payload->standard_mode, _payload->custom_mode, _payload->properties, _payload->mode_name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char* mode_name,
    fmav_status_t* _status)
{
    fmav_available_modes_t* _payload = (fmav_available_modes_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->properties = properties;
    _payload->number_modes = number_modes;
    _payload->mode_index = mode_index;
    _payload->standard_mode = standard_mode;
    memcpy(&(_payload->mode_name), mode_name, sizeof(char)*35);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AVAILABLE_MODES;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AVAILABLE_MODES >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AVAILABLE_MODES >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVAILABLE_MODES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_available_modes_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_available_modes_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->number_modes, _payload->mode_index, _payload->standard_mode, _payload->custom_mode, _payload->properties, _payload->mode_name,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char* mode_name,
    fmav_status_t* _status)
{
    fmav_available_modes_t _payload;

    _payload.custom_mode = custom_mode;
    _payload.properties = properties;
    _payload.number_modes = number_modes;
    _payload.mode_index = mode_index;
    _payload.standard_mode = standard_mode;
    memcpy(&(_payload.mode_name), mode_name, sizeof(char)*35);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AVAILABLE_MODES,
        FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVAILABLE_MODES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_available_modes_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_available_modes_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AVAILABLE_MODES,
        FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVAILABLE_MODES_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AVAILABLE_MODES decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_available_modes_decode(fmav_available_modes_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVAILABLE_MODES_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_available_modes_get_field_custom_mode(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_available_modes_get_field_properties(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_available_modes_get_field_number_modes(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_available_modes_get_field_mode_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_available_modes_get_field_standard_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_available_modes_get_field_mode_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[11]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_available_modes_get_field_mode_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_NUM) return 0;
    return ((char*)&(msg->payload[11]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AVAILABLE_MODES  435

#define mavlink_available_modes_t  fmav_available_modes_t

#define MAVLINK_MSG_ID_AVAILABLE_MODES_LEN  46
#define MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN  46
#define MAVLINK_MSG_ID_435_LEN  46
#define MAVLINK_MSG_ID_435_MIN_LEN  46

#define MAVLINK_MSG_ID_AVAILABLE_MODES_CRC  134
#define MAVLINK_MSG_ID_435_CRC  134

#define MAVLINK_MSG_AVAILABLE_MODES_FIELD_MODE_NAME_LEN 35


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_available_modes_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char* mode_name)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_available_modes_pack(
        _msg, sysid, compid,
        number_modes, mode_index, standard_mode, custom_mode, properties, mode_name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_available_modes_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_available_modes_t* _payload)
{
    return mavlink_msg_available_modes_pack(
        sysid,
        compid,
        _msg,
        _payload->number_modes, _payload->mode_index, _payload->standard_mode, _payload->custom_mode, _payload->properties, _payload->mode_name);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_available_modes_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t number_modes, uint8_t mode_index, uint8_t standard_mode, uint32_t custom_mode, uint32_t properties, const char* mode_name)
{
    return fmav_msg_available_modes_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        number_modes, mode_index, standard_mode, custom_mode, properties, mode_name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_available_modes_decode(const mavlink_message_t* msg, mavlink_available_modes_t* payload)
{
    fmav_msg_available_modes_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AVAILABLE_MODES_H

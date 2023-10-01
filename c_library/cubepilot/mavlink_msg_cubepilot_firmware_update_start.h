//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_H
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_H


//----------------------------------------
//-- Message CUBEPILOT_FIRMWARE_UPDATE_START
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_cubepilot_firmware_update_start_t {
    uint32_t size;
    uint32_t crc;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_cubepilot_firmware_update_start_t;


#define FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START  50004

#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX  10
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA  240

#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FLAGS  3
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_TARGET_COMPONENT_OFS  9

#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FRAME_LEN_MAX  35



#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FIELD_SIZE_OFS  0
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FIELD_CRC_OFS  4
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FIELD_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_FIELD_TARGET_COMPONENT_OFS  9


//----------------------------------------
//-- Message CUBEPILOT_FIRMWARE_UPDATE_START pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc,
    fmav_status_t* _status)
{
    fmav_cubepilot_firmware_update_start_t* _payload = (fmav_cubepilot_firmware_update_start_t*)_msg->payload;

    _payload->size = size;
    _payload->crc = crc;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cubepilot_firmware_update_start_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cubepilot_firmware_update_start_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->size, _payload->crc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc,
    fmav_status_t* _status)
{
    fmav_cubepilot_firmware_update_start_t* _payload = (fmav_cubepilot_firmware_update_start_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->size = size;
    _payload->crc = crc;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cubepilot_firmware_update_start_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cubepilot_firmware_update_start_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->size, _payload->crc,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc,
    fmav_status_t* _status)
{
    fmav_cubepilot_firmware_update_start_t _payload;

    _payload.size = size;
    _payload.crc = crc;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cubepilot_firmware_update_start_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_cubepilot_firmware_update_start_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CUBEPILOT_FIRMWARE_UPDATE_START decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cubepilot_firmware_update_start_decode(fmav_cubepilot_firmware_update_start_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_cubepilot_firmware_update_start_get_field_size(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_cubepilot_firmware_update_start_get_field_crc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cubepilot_firmware_update_start_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cubepilot_firmware_update_start_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START  50004

#define mavlink_cubepilot_firmware_update_start_t  fmav_cubepilot_firmware_update_start_t

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_LEN  10
#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_MIN_LEN  10
#define MAVLINK_MSG_ID_50004_LEN  10
#define MAVLINK_MSG_ID_50004_MIN_LEN  10

#define MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_START_CRC  240
#define MAVLINK_MSG_ID_50004_CRC  240




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cubepilot_firmware_update_start_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_cubepilot_firmware_update_start_pack(
        _msg, sysid, compid,
        target_system, target_component, size, crc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cubepilot_firmware_update_start_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_cubepilot_firmware_update_start_t* _payload)
{
    return mavlink_msg_cubepilot_firmware_update_start_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->size, _payload->crc);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cubepilot_firmware_update_start_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t size, uint32_t crc)
{
    return fmav_msg_cubepilot_firmware_update_start_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, size, crc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_cubepilot_firmware_update_start_decode(const mavlink_message_t* msg, mavlink_cubepilot_firmware_update_start_t* payload)
{
    fmav_msg_cubepilot_firmware_update_start_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CUBEPILOT_FIRMWARE_UPDATE_START_H
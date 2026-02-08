//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_EEPROM_H
#define FASTMAVLINK_MSG_ESC_EEPROM_H


//----------------------------------------
//-- Message ESC_EEPROM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_eeprom_t {
    uint32_t write_mask[6];
    uint8_t target_system;
    uint8_t target_component;
    uint8_t firmware;
    uint8_t msg_index;
    uint8_t msg_count;
    uint8_t esc_index;
    uint8_t length;
    uint8_t data[192];
}) fmav_esc_eeprom_t;


#define FASTMAVLINK_MSG_ID_ESC_EEPROM  292

#define FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX  223
#define FASTMAVLINK_MSG_ESC_EEPROM_CRCEXTRA  227

#define FASTMAVLINK_MSG_ESC_EEPROM_FLAGS  3
#define FASTMAVLINK_MSG_ESC_EEPROM_TARGET_SYSTEM_OFS  24
#define FASTMAVLINK_MSG_ESC_EEPROM_TARGET_COMPONENT_OFS  25

#define FASTMAVLINK_MSG_ESC_EEPROM_FRAME_LEN_MAX  248

#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_WRITE_MASK_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_WRITE_MASK_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_DATA_NUM  192 // number of elements in array
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_DATA_LEN  192 // length of array = number of bytes

#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_WRITE_MASK_OFS  0
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_TARGET_SYSTEM_OFS  24
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_TARGET_COMPONENT_OFS  25
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_FIRMWARE_OFS  26
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_MSG_INDEX_OFS  27
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_MSG_COUNT_OFS  28
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_ESC_INDEX_OFS  29
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_LENGTH_OFS  30
#define FASTMAVLINK_MSG_ESC_EEPROM_FIELD_DATA_OFS  31


//----------------------------------------
//-- Message ESC_EEPROM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t firmware, uint8_t msg_index, uint8_t msg_count, uint8_t esc_index, const uint32_t* write_mask, uint8_t length, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_esc_eeprom_t* _payload = (fmav_esc_eeprom_t*)_msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->firmware = firmware;
    _payload->msg_index = msg_index;
    _payload->msg_count = msg_count;
    _payload->esc_index = esc_index;
    _payload->length = length;
    memcpy(&(_payload->write_mask), write_mask, sizeof(uint32_t)*6);
    memcpy(&(_payload->data), data, sizeof(uint8_t)*192);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ESC_EEPROM;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_ESC_EEPROM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_eeprom_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_eeprom_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->firmware, _payload->msg_index, _payload->msg_count, _payload->esc_index, _payload->write_mask, _payload->length, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t firmware, uint8_t msg_index, uint8_t msg_count, uint8_t esc_index, const uint32_t* write_mask, uint8_t length, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_esc_eeprom_t* _payload = (fmav_esc_eeprom_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->firmware = firmware;
    _payload->msg_index = msg_index;
    _payload->msg_count = msg_count;
    _payload->esc_index = esc_index;
    _payload->length = length;
    memcpy(&(_payload->write_mask), write_mask, sizeof(uint32_t)*6);
    memcpy(&(_payload->data), data, sizeof(uint8_t)*192);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_EEPROM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_EEPROM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_EEPROM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_EEPROM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_eeprom_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_eeprom_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->firmware, _payload->msg_index, _payload->msg_count, _payload->esc_index, _payload->write_mask, _payload->length, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t firmware, uint8_t msg_index, uint8_t msg_count, uint8_t esc_index, const uint32_t* write_mask, uint8_t length, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_esc_eeprom_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.firmware = firmware;
    _payload.msg_index = msg_index;
    _payload.msg_count = msg_count;
    _payload.esc_index = esc_index;
    _payload.length = length;
    memcpy(&(_payload.write_mask), write_mask, sizeof(uint32_t)*6);
    memcpy(&(_payload.data), data, sizeof(uint8_t)*192);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_EEPROM,
        FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_EEPROM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_eeprom_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_eeprom_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_EEPROM,
        FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_EEPROM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_EEPROM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_eeprom_decode(fmav_esc_eeprom_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_EEPROM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_firmware(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_msg_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_msg_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_esc_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_length(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_esc_eeprom_get_field_write_mask_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_esc_eeprom_get_field_write_mask(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_EEPROM_FIELD_WRITE_MASK_NUM) return 0;
    return ((uint32_t*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_esc_eeprom_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[31]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_eeprom_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_EEPROM_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[31]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_EEPROM  292

#define mavlink_esc_eeprom_t  fmav_esc_eeprom_t

#define MAVLINK_MSG_ID_ESC_EEPROM_LEN  223
#define MAVLINK_MSG_ID_ESC_EEPROM_MIN_LEN  223
#define MAVLINK_MSG_ID_292_LEN  223
#define MAVLINK_MSG_ID_292_MIN_LEN  223

#define MAVLINK_MSG_ID_ESC_EEPROM_CRC  227
#define MAVLINK_MSG_ID_292_CRC  227

#define MAVLINK_MSG_ESC_EEPROM_FIELD_WRITE_MASK_LEN 6
#define MAVLINK_MSG_ESC_EEPROM_FIELD_DATA_LEN 192


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_eeprom_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t firmware, uint8_t msg_index, uint8_t msg_count, uint8_t esc_index, const uint32_t* write_mask, uint8_t length, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_eeprom_pack(
        _msg, sysid, compid,
        target_system, target_component, firmware, msg_index, msg_count, esc_index, write_mask, length, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_eeprom_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_esc_eeprom_t* _payload)
{
    return mavlink_msg_esc_eeprom_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->firmware, _payload->msg_index, _payload->msg_count, _payload->esc_index, _payload->write_mask, _payload->length, _payload->data);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_eeprom_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t firmware, uint8_t msg_index, uint8_t msg_count, uint8_t esc_index, const uint32_t* write_mask, uint8_t length, const uint8_t* data)
{
    return fmav_msg_esc_eeprom_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, firmware, msg_index, msg_count, esc_index, write_mask, length, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_eeprom_decode(const mavlink_message_t* msg, mavlink_esc_eeprom_t* payload)
{
    fmav_msg_esc_eeprom_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_EEPROM_H

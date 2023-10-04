//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MCU_STATUS_H
#define FASTMAVLINK_MSG_MCU_STATUS_H


//----------------------------------------
//-- Message MCU_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mcu_status_t {
    int16_t MCU_temperature;
    uint16_t MCU_voltage;
    uint16_t MCU_voltage_min;
    uint16_t MCU_voltage_max;
    uint8_t id;
}) fmav_mcu_status_t;


#define FASTMAVLINK_MSG_ID_MCU_STATUS  11039

#define FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA  142

#define FASTMAVLINK_MSG_MCU_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_MCU_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MCU_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MCU_STATUS_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_MCU_STATUS_FIELD_MCU_TEMPERATURE_OFS  0
#define FASTMAVLINK_MSG_MCU_STATUS_FIELD_MCU_VOLTAGE_OFS  2
#define FASTMAVLINK_MSG_MCU_STATUS_FIELD_MCU_VOLTAGE_MIN_OFS  4
#define FASTMAVLINK_MSG_MCU_STATUS_FIELD_MCU_VOLTAGE_MAX_OFS  6
#define FASTMAVLINK_MSG_MCU_STATUS_FIELD_ID_OFS  8


//----------------------------------------
//-- Message MCU_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max,
    fmav_status_t* _status)
{
    fmav_mcu_status_t* _payload = (fmav_mcu_status_t*)_msg->payload;

    _payload->MCU_temperature = MCU_temperature;
    _payload->MCU_voltage = MCU_voltage;
    _payload->MCU_voltage_min = MCU_voltage_min;
    _payload->MCU_voltage_max = MCU_voltage_max;
    _payload->id = id;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MCU_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mcu_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mcu_status_pack(
        _msg, sysid, compid,
        _payload->id, _payload->MCU_temperature, _payload->MCU_voltage, _payload->MCU_voltage_min, _payload->MCU_voltage_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max,
    fmav_status_t* _status)
{
    fmav_mcu_status_t* _payload = (fmav_mcu_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->MCU_temperature = MCU_temperature;
    _payload->MCU_voltage = MCU_voltage;
    _payload->MCU_voltage_min = MCU_voltage_min;
    _payload->MCU_voltage_max = MCU_voltage_max;
    _payload->id = id;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MCU_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MCU_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MCU_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mcu_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mcu_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->MCU_temperature, _payload->MCU_voltage, _payload->MCU_voltage_min, _payload->MCU_voltage_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max,
    fmav_status_t* _status)
{
    fmav_mcu_status_t _payload;

    _payload.MCU_temperature = MCU_temperature;
    _payload.MCU_voltage = MCU_voltage;
    _payload.MCU_voltage_min = MCU_voltage_min;
    _payload.MCU_voltage_max = MCU_voltage_max;
    _payload.id = id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MCU_STATUS,
        FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mcu_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MCU_STATUS,
        FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MCU_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MCU_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mcu_status_decode(fmav_mcu_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MCU_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_mcu_status_get_field_MCU_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_get_field_MCU_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_get_field_MCU_voltage_min(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mcu_status_get_field_MCU_voltage_max(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mcu_status_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MCU_STATUS  11039

#define mavlink_mcu_status_t  fmav_mcu_status_t

#define MAVLINK_MSG_ID_MCU_STATUS_LEN  9
#define MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN  9
#define MAVLINK_MSG_ID_11039_LEN  9
#define MAVLINK_MSG_ID_11039_MIN_LEN  9

#define MAVLINK_MSG_ID_MCU_STATUS_CRC  142
#define MAVLINK_MSG_ID_11039_CRC  142




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mcu_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mcu_status_pack(
        _msg, sysid, compid,
        id, MCU_temperature, MCU_voltage, MCU_voltage_min, MCU_voltage_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mcu_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mcu_status_t* _payload)
{
    return mavlink_msg_mcu_status_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->MCU_temperature, _payload->MCU_voltage, _payload->MCU_voltage_min, _payload->MCU_voltage_max);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mcu_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
    return fmav_msg_mcu_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, MCU_temperature, MCU_voltage, MCU_voltage_min, MCU_voltage_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mcu_status_decode(const mavlink_message_t* msg, mavlink_mcu_status_t* payload)
{
    fmav_msg_mcu_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MCU_STATUS_H

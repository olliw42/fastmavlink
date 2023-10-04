//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BATTERY_STATUS_V2_H
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_H


//----------------------------------------
//-- Message BATTERY_STATUS_V2
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_battery_status_v2_t {
    float voltage;
    float current;
    float capacity_consumed;
    float capacity_remaining;
    uint32_t status_flags;
    int16_t temperature;
    uint8_t id;
    uint8_t percent_remaining;
}) fmav_battery_status_v2_t;


#define FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2  369

#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_CRCEXTRA  151

#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FLAGS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FRAME_LEN_MAX  49



#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_VOLTAGE_OFS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_CURRENT_OFS  4
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_CAPACITY_CONSUMED_OFS  8
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_CAPACITY_REMAINING_OFS  12
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_STATUS_FLAGS_OFS  16
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_TEMPERATURE_OFS  20
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_ID_OFS  22
#define FASTMAVLINK_MSG_BATTERY_STATUS_V2_FIELD_PERCENT_REMAINING_OFS  23


//----------------------------------------
//-- Message BATTERY_STATUS_V2 pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, float voltage, float current, float capacity_consumed, float capacity_remaining, uint8_t percent_remaining, uint32_t status_flags,
    fmav_status_t* _status)
{
    fmav_battery_status_v2_t* _payload = (fmav_battery_status_v2_t*)_msg->payload;

    _payload->voltage = voltage;
    _payload->current = current;
    _payload->capacity_consumed = capacity_consumed;
    _payload->capacity_remaining = capacity_remaining;
    _payload->status_flags = status_flags;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->percent_remaining = percent_remaining;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_BATTERY_STATUS_V2_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_v2_pack(
        _msg, sysid, compid,
        _payload->id, _payload->temperature, _payload->voltage, _payload->current, _payload->capacity_consumed, _payload->capacity_remaining, _payload->percent_remaining, _payload->status_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, float voltage, float current, float capacity_consumed, float capacity_remaining, uint8_t percent_remaining, uint32_t status_flags,
    fmav_status_t* _status)
{
    fmav_battery_status_v2_t* _payload = (fmav_battery_status_v2_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->voltage = voltage;
    _payload->current = current;
    _payload->capacity_consumed = capacity_consumed;
    _payload->capacity_remaining = capacity_remaining;
    _payload->status_flags = status_flags;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->percent_remaining = percent_remaining;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2 >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2 >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_v2_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->temperature, _payload->voltage, _payload->current, _payload->capacity_consumed, _payload->capacity_remaining, _payload->percent_remaining, _payload->status_flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, float voltage, float current, float capacity_consumed, float capacity_remaining, uint8_t percent_remaining, uint32_t status_flags,
    fmav_status_t* _status)
{
    fmav_battery_status_v2_t _payload;

    _payload.voltage = voltage;
    _payload.current = current;
    _payload.capacity_consumed = capacity_consumed;
    _payload.capacity_remaining = capacity_remaining;
    _payload.status_flags = status_flags;
    _payload.temperature = temperature;
    _payload.id = id;
    _payload.percent_remaining = percent_remaining;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_v2_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_v2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS_V2,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_V2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BATTERY_STATUS_V2 decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_battery_status_v2_decode(fmav_battery_status_v2_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_BATTERY_STATUS_V2_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_status_v2_get_field_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_status_v2_get_field_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_status_v2_get_field_capacity_consumed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_battery_status_v2_get_field_capacity_remaining(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_battery_status_v2_get_field_status_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_battery_status_v2_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_v2_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_battery_status_v2_get_field_percent_remaining(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[23]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BATTERY_STATUS_V2  369

#define mavlink_battery_status_v2_t  fmav_battery_status_v2_t

#define MAVLINK_MSG_ID_BATTERY_STATUS_V2_LEN  24
#define MAVLINK_MSG_ID_BATTERY_STATUS_V2_MIN_LEN  24
#define MAVLINK_MSG_ID_369_LEN  24
#define MAVLINK_MSG_ID_369_MIN_LEN  24

#define MAVLINK_MSG_ID_BATTERY_STATUS_V2_CRC  151
#define MAVLINK_MSG_ID_369_CRC  151




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_v2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, int16_t temperature, float voltage, float current, float capacity_consumed, float capacity_remaining, uint8_t percent_remaining, uint32_t status_flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_battery_status_v2_pack(
        _msg, sysid, compid,
        id, temperature, voltage, current, capacity_consumed, capacity_remaining, percent_remaining, status_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_v2_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_battery_status_v2_t* _payload)
{
    return mavlink_msg_battery_status_v2_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->temperature, _payload->voltage, _payload->current, _payload->capacity_consumed, _payload->capacity_remaining, _payload->percent_remaining, _payload->status_flags);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_v2_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, int16_t temperature, float voltage, float current, float capacity_consumed, float capacity_remaining, uint8_t percent_remaining, uint32_t status_flags)
{
    return fmav_msg_battery_status_v2_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, temperature, voltage, current, capacity_consumed, capacity_remaining, percent_remaining, status_flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_battery_status_v2_decode(const mavlink_message_t* msg, mavlink_battery_status_v2_t* payload)
{
    fmav_msg_battery_status_v2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BATTERY_STATUS_V2_H

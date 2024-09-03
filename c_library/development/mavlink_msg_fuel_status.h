//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FUEL_STATUS_H
#define FASTMAVLINK_MSG_FUEL_STATUS_H


//----------------------------------------
//-- Message FUEL_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_fuel_status_t {
    float maximum_fuel;
    float consumed_fuel;
    float remaining_fuel;
    float flow_rate;
    float temperature;
    uint32_t fuel_type;
    uint8_t id;
    uint8_t percent_remaining;
}) fmav_fuel_status_t;


#define FASTMAVLINK_MSG_ID_FUEL_STATUS  371

#define FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_FUEL_STATUS_CRCEXTRA  10

#define FASTMAVLINK_MSG_FUEL_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_FUEL_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FUEL_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FUEL_STATUS_FRAME_LEN_MAX  51



#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_MAXIMUM_FUEL_OFS  0
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_CONSUMED_FUEL_OFS  4
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_REMAINING_FUEL_OFS  8
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_FLOW_RATE_OFS  12
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_TEMPERATURE_OFS  16
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_FUEL_TYPE_OFS  20
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_ID_OFS  24
#define FASTMAVLINK_MSG_FUEL_STATUS_FIELD_PERCENT_REMAINING_OFS  25


//----------------------------------------
//-- Message FUEL_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type,
    fmav_status_t* _status)
{
    fmav_fuel_status_t* _payload = (fmav_fuel_status_t*)_msg->payload;

    _payload->maximum_fuel = maximum_fuel;
    _payload->consumed_fuel = consumed_fuel;
    _payload->remaining_fuel = remaining_fuel;
    _payload->flow_rate = flow_rate;
    _payload->temperature = temperature;
    _payload->fuel_type = fuel_type;
    _payload->id = id;
    _payload->percent_remaining = percent_remaining;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_FUEL_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_FUEL_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fuel_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fuel_status_pack(
        _msg, sysid, compid,
        _payload->id, _payload->maximum_fuel, _payload->consumed_fuel, _payload->remaining_fuel, _payload->percent_remaining, _payload->flow_rate, _payload->temperature, _payload->fuel_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type,
    fmav_status_t* _status)
{
    fmav_fuel_status_t* _payload = (fmav_fuel_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->maximum_fuel = maximum_fuel;
    _payload->consumed_fuel = consumed_fuel;
    _payload->remaining_fuel = remaining_fuel;
    _payload->flow_rate = flow_rate;
    _payload->temperature = temperature;
    _payload->fuel_type = fuel_type;
    _payload->id = id;
    _payload->percent_remaining = percent_remaining;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FUEL_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FUEL_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FUEL_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FUEL_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fuel_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fuel_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->maximum_fuel, _payload->consumed_fuel, _payload->remaining_fuel, _payload->percent_remaining, _payload->flow_rate, _payload->temperature, _payload->fuel_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type,
    fmav_status_t* _status)
{
    fmav_fuel_status_t _payload;

    _payload.maximum_fuel = maximum_fuel;
    _payload.consumed_fuel = consumed_fuel;
    _payload.remaining_fuel = remaining_fuel;
    _payload.flow_rate = flow_rate;
    _payload.temperature = temperature;
    _payload.fuel_type = fuel_type;
    _payload.id = id;
    _payload.percent_remaining = percent_remaining;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FUEL_STATUS,
        FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FUEL_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fuel_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_fuel_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FUEL_STATUS,
        FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FUEL_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FUEL_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_fuel_status_decode(fmav_fuel_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_FUEL_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fuel_status_get_field_maximum_fuel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fuel_status_get_field_consumed_fuel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fuel_status_get_field_remaining_fuel(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fuel_status_get_field_flow_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fuel_status_get_field_temperature(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_fuel_status_get_field_fuel_type(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_fuel_status_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_fuel_status_get_field_percent_remaining(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FUEL_STATUS  371

#define mavlink_fuel_status_t  fmav_fuel_status_t

#define MAVLINK_MSG_ID_FUEL_STATUS_LEN  26
#define MAVLINK_MSG_ID_FUEL_STATUS_MIN_LEN  26
#define MAVLINK_MSG_ID_371_LEN  26
#define MAVLINK_MSG_ID_371_MIN_LEN  26

#define MAVLINK_MSG_ID_FUEL_STATUS_CRC  10
#define MAVLINK_MSG_ID_371_CRC  10




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fuel_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_fuel_status_pack(
        _msg, sysid, compid,
        id, maximum_fuel, consumed_fuel, remaining_fuel, percent_remaining, flow_rate, temperature, fuel_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fuel_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_fuel_status_t* _payload)
{
    return mavlink_msg_fuel_status_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->maximum_fuel, _payload->consumed_fuel, _payload->remaining_fuel, _payload->percent_remaining, _payload->flow_rate, _payload->temperature, _payload->fuel_type);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fuel_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float maximum_fuel, float consumed_fuel, float remaining_fuel, uint8_t percent_remaining, float flow_rate, float temperature, uint32_t fuel_type)
{
    return fmav_msg_fuel_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, maximum_fuel, consumed_fuel, remaining_fuel, percent_remaining, flow_rate, temperature, fuel_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_fuel_status_decode(const mavlink_message_t* msg, mavlink_fuel_status_t* payload)
{
    fmav_msg_fuel_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FUEL_STATUS_H

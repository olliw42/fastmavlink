//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AIRSPEED_H
#define FASTMAVLINK_MSG_AIRSPEED_H


//----------------------------------------
//-- Message AIRSPEED
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_airspeed_t {
    float airspeed;
    float raw_press;
    int16_t temperature;
    uint8_t id;
    uint8_t flags;
}) fmav_airspeed_t;


#define FASTMAVLINK_MSG_ID_AIRSPEED  295

#define FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA  234

#define FASTMAVLINK_MSG_AIRSPEED_FLAGS  0
#define FASTMAVLINK_MSG_AIRSPEED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIRSPEED_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_AIRSPEED_FIELD_AIRSPEED_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_RAW_PRESS_OFS  4
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_TEMPERATURE_OFS  8
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_ID_OFS  10
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_FLAGS_OFS  11


//----------------------------------------
//-- Message AIRSPEED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_airspeed_t* _payload = (fmav_airspeed_t*)_msg->payload;

    _payload->airspeed = airspeed;
    _payload->raw_press = raw_press;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->flags = flags;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AIRSPEED;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airspeed_pack(
        _msg, sysid, compid,
        _payload->id, _payload->airspeed, _payload->temperature, _payload->raw_press, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_airspeed_t* _payload = (fmav_airspeed_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->airspeed = airspeed;
    _payload->raw_press = raw_press;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->flags = flags;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AIRSPEED;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRSPEED >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRSPEED >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airspeed_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->airspeed, _payload->temperature, _payload->raw_press, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_airspeed_t _payload;

    _payload.airspeed = airspeed;
    _payload.raw_press = raw_press;
    _payload.temperature = temperature;
    _payload.id = id;
    _payload.flags = flags;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AIRSPEED,
        FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AIRSPEED,
        FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AIRSPEED decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airspeed_decode(fmav_airspeed_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_raw_press(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_airspeed_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airspeed_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airspeed_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIRSPEED  295

#define mavlink_airspeed_t  fmav_airspeed_t

#define MAVLINK_MSG_ID_AIRSPEED_LEN  12
#define MAVLINK_MSG_ID_AIRSPEED_MIN_LEN  12
#define MAVLINK_MSG_ID_295_LEN  12
#define MAVLINK_MSG_ID_295_MIN_LEN  12

#define MAVLINK_MSG_ID_AIRSPEED_CRC  234
#define MAVLINK_MSG_ID_295_CRC  234




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_airspeed_pack(
        _msg, sysid, compid,
        id, airspeed, temperature, raw_press, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_airspeed_t* _payload)
{
    return mavlink_msg_airspeed_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->airspeed, _payload->temperature, _payload->raw_press, _payload->flags);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
    return fmav_msg_airspeed_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, airspeed, temperature, raw_press, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_airspeed_decode(const mavlink_message_t* msg, mavlink_airspeed_t* payload)
{
    fmav_msg_airspeed_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIRSPEED_H

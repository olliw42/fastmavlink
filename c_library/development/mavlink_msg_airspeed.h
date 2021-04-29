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
    float press_diff;
    float press_static;
    float error;
    int16_t temperature;
    uint8_t id;
    uint8_t type;
}) fmav_airspeed_t;


#define FASTMAVLINK_MSG_ID_AIRSPEED  295

#define FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_AIRSPEED_CRCEXTRA  41

#define FASTMAVLINK_MSG_AIRSPEED_FLAGS  0
#define FASTMAVLINK_MSG_AIRSPEED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIRSPEED_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_AIRSPEED_FIELD_AIRSPEED_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_PRESS_DIFF_OFS  4
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_PRESS_STATIC_OFS  8
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_ERROR_OFS  12
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_TEMPERATURE_OFS  16
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_ID_OFS  18
#define FASTMAVLINK_MSG_AIRSPEED_FIELD_TYPE_OFS  19


//----------------------------------------
//-- Message AIRSPEED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float press_diff, float press_static, float error, uint8_t type,
    fmav_status_t* _status)
{
    fmav_airspeed_t* _payload = (fmav_airspeed_t*)_msg->payload;

    _payload->airspeed = airspeed;
    _payload->press_diff = press_diff;
    _payload->press_static = press_static;
    _payload->error = error;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->type = type;


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
        _payload->id, _payload->airspeed, _payload->temperature, _payload->press_diff, _payload->press_static, _payload->error, _payload->type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float press_diff, float press_static, float error, uint8_t type,
    fmav_status_t* _status)
{
    fmav_airspeed_t* _payload = (fmav_airspeed_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->airspeed = airspeed;
    _payload->press_diff = press_diff;
    _payload->press_static = press_static;
    _payload->error = error;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->type = type;


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
        _payload->id, _payload->airspeed, _payload->temperature, _payload->press_diff, _payload->press_static, _payload->error, _payload->type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float press_diff, float press_static, float error, uint8_t type,
    fmav_status_t* _status)
{
    fmav_airspeed_t _payload;

    _payload.airspeed = airspeed;
    _payload.press_diff = press_diff;
    _payload.press_static = press_static;
    _payload.error = error;
    _payload.temperature = temperature;
    _payload.id = id;
    _payload.type = type;


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
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airspeed_decode(fmav_airspeed_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRSPEED_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_press_diff(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_press_static(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_get_field_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_airspeed_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airspeed_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_airspeed_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIRSPEED  295

#define mavlink_airspeed_t  fmav_airspeed_t

#define MAVLINK_MSG_ID_AIRSPEED_LEN  20
#define MAVLINK_MSG_ID_AIRSPEED_MIN_LEN  20
#define MAVLINK_MSG_ID_295_LEN  20
#define MAVLINK_MSG_ID_295_MIN_LEN  20

#define MAVLINK_MSG_ID_AIRSPEED_CRC  41
#define MAVLINK_MSG_ID_295_CRC  41




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, float airspeed, int16_t temperature, float press_diff, float press_static, float error, uint8_t type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_airspeed_pack(
        _msg, sysid, compid,
        id, airspeed, temperature, press_diff, press_static, error, type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, float airspeed, int16_t temperature, float press_diff, float press_static, float error, uint8_t type)
{
    return fmav_msg_airspeed_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, airspeed, temperature, press_diff, press_static, error, type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_airspeed_decode(const mavlink_message_t* msg, mavlink_airspeed_t* payload)
{
    fmav_msg_airspeed_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIRSPEED_H

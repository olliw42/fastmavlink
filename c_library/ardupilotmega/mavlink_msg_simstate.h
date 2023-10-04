//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SIMSTATE_H
#define FASTMAVLINK_MSG_SIMSTATE_H


//----------------------------------------
//-- Message SIMSTATE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_simstate_t {
    float roll;
    float pitch;
    float yaw;
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
    int32_t lat;
    int32_t lng;
}) fmav_simstate_t;


#define FASTMAVLINK_MSG_ID_SIMSTATE  164

#define FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX  44
#define FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA  154

#define FASTMAVLINK_MSG_SIMSTATE_FLAGS  0
#define FASTMAVLINK_MSG_SIMSTATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SIMSTATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SIMSTATE_FRAME_LEN_MAX  69



#define FASTMAVLINK_MSG_SIMSTATE_FIELD_ROLL_OFS  0
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_PITCH_OFS  4
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_YAW_OFS  8
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_XACC_OFS  12
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_YACC_OFS  16
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_ZACC_OFS  20
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_XGYRO_OFS  24
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_YGYRO_OFS  28
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_ZGYRO_OFS  32
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_LAT_OFS  36
#define FASTMAVLINK_MSG_SIMSTATE_FIELD_LNG_OFS  40


//----------------------------------------
//-- Message SIMSTATE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_simstate_t* _payload = (fmav_simstate_t*)_msg->payload;

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lng = lng;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SIMSTATE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_simstate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_simstate_pack(
        _msg, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_simstate_t* _payload = (fmav_simstate_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lng = lng;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SIMSTATE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SIMSTATE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SIMSTATE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_simstate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_simstate_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lng,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_simstate_t _payload;

    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;
    _payload.xgyro = xgyro;
    _payload.ygyro = ygyro;
    _payload.zgyro = zgyro;
    _payload.lat = lat;
    _payload.lng = lng;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SIMSTATE,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_simstate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SIMSTATE,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SIMSTATE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_simstate_decode(fmav_simstate_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_xacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_yacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_zacc(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_xgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_ygyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_simstate_get_field_zgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_simstate_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_simstate_get_field_lng(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SIMSTATE  164

#define mavlink_simstate_t  fmav_simstate_t

#define MAVLINK_MSG_ID_SIMSTATE_LEN  44
#define MAVLINK_MSG_ID_SIMSTATE_MIN_LEN  44
#define MAVLINK_MSG_ID_164_LEN  44
#define MAVLINK_MSG_ID_164_MIN_LEN  44

#define MAVLINK_MSG_ID_SIMSTATE_CRC  154
#define MAVLINK_MSG_ID_164_CRC  154




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_simstate_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_simstate_pack(
        _msg, sysid, compid,
        roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_simstate_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_simstate_t* _payload)
{
    return mavlink_msg_simstate_pack(
        sysid,
        compid,
        _msg,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lng);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_simstate_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
    return fmav_msg_simstate_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_simstate_decode(const mavlink_message_t* msg, mavlink_simstate_t* payload)
{
    fmav_msg_simstate_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SIMSTATE_H

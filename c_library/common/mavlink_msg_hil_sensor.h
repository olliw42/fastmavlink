//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_SENSOR_H
#define FASTMAVLINK_MSG_HIL_SENSOR_H


//----------------------------------------
//-- Message HIL_SENSOR
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_sensor_t {
    uint64_t time_usec;
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
    float xmag;
    float ymag;
    float zmag;
    float abs_pressure;
    float diff_pressure;
    float pressure_alt;
    float temperature;
    uint32_t fields_updated;
    uint8_t id;
}) fmav_hil_sensor_t;


#define FASTMAVLINK_MSG_ID_HIL_SENSOR  107


#define FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MIN  64
#define FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX  65
#define FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN  65
#define FASTMAVLINK_MSG_HIL_SENSOR_CRCEXTRA  108

#define FASTMAVLINK_MSG_ID_107_LEN_MIN  64
#define FASTMAVLINK_MSG_ID_107_LEN_MAX  65
#define FASTMAVLINK_MSG_ID_107_LEN  65
#define FASTMAVLINK_MSG_ID_107_CRCEXTRA  108



#define FASTMAVLINK_MSG_HIL_SENSOR_FLAGS  0
#define FASTMAVLINK_MSG_HIL_SENSOR_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_SENSOR_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIL_SENSOR packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_sensor_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id,
    fmav_status_t* _status)
{
    fmav_hil_sensor_t* _payload = (fmav_hil_sensor_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->abs_pressure = abs_pressure;
    _payload->diff_pressure = diff_pressure;
    _payload->pressure_alt = pressure_alt;
    _payload->temperature = temperature;
    _payload->fields_updated = fields_updated;
    _payload->id = id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_SENSOR;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_SENSOR_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_sensor_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_sensor_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->abs_pressure, _payload->diff_pressure, _payload->pressure_alt, _payload->temperature, _payload->fields_updated, _payload->id,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_sensor_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id,
    fmav_status_t* _status)
{
    fmav_hil_sensor_t* _payload = (fmav_hil_sensor_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->abs_pressure = abs_pressure;
    _payload->diff_pressure = diff_pressure;
    _payload->pressure_alt = pressure_alt;
    _payload->temperature = temperature;
    _payload->fields_updated = fields_updated;
    _payload->id = id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_SENSOR;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_SENSOR >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_SENSOR >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_SENSOR_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_sensor_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_sensor_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_sensor_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->abs_pressure, _payload->diff_pressure, _payload->pressure_alt, _payload->temperature, _payload->fields_updated, _payload->id,
        _status);
}


//----------------------------------------
//-- Message HIL_SENSOR unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_sensor_decode(fmav_hil_sensor_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_SENSOR_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_SENSOR  107

#define mavlink_hil_sensor_t  fmav_hil_sensor_t

#define MAVLINK_MSG_ID_HIL_SENSOR_LEN  65
#define MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN  64
#define MAVLINK_MSG_ID_107_LEN  65
#define MAVLINK_MSG_ID_107_MIN_LEN  64

#define MAVLINK_MSG_ID_HIL_SENSOR_CRC  108
#define MAVLINK_MSG_ID_107_CRC  108




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_sensor_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_sensor_pack(
        msg, sysid, compid,
        time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, id,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_sensor_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
{
    return fmav_msg_hil_sensor_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated, id,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_sensor_decode(const mavlink_message_t* msg, mavlink_hil_sensor_t* payload)
{
    fmav_msg_hil_sensor_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_SENSOR_H

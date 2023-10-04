//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_H
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_H


//----------------------------------------
//-- Message ESC_TELEMETRY_5_TO_8
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_telemetry_5_to_8_t {
    uint16_t voltage[4];
    uint16_t current[4];
    uint16_t totalcurrent[4];
    uint16_t rpm[4];
    uint16_t count[4];
    uint8_t temperature[4];
}) fmav_esc_telemetry_5_to_8_t;


#define FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8  11031

#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX  44
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA  133

#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FLAGS  0
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FRAME_LEN_MAX  69

#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_VOLTAGE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_VOLTAGE_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_CURRENT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_CURRENT_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TOTALCURRENT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TOTALCURRENT_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_RPM_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_RPM_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_COUNT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_COUNT_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TEMPERATURE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TEMPERATURE_LEN  4 // length of array = number of bytes

#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_VOLTAGE_OFS  0
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_CURRENT_OFS  8
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TOTALCURRENT_OFS  16
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_RPM_OFS  24
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_COUNT_OFS  32
#define FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TEMPERATURE_OFS  40


//----------------------------------------
//-- Message ESC_TELEMETRY_5_TO_8 pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const uint8_t* temperature, const uint16_t* voltage, const uint16_t* current, const uint16_t* totalcurrent, const uint16_t* rpm, const uint16_t* count,
    fmav_status_t* _status)
{
    fmav_esc_telemetry_5_to_8_t* _payload = (fmav_esc_telemetry_5_to_8_t*)_msg->payload;


    memcpy(&(_payload->voltage), voltage, sizeof(uint16_t)*4);
    memcpy(&(_payload->current), current, sizeof(uint16_t)*4);
    memcpy(&(_payload->totalcurrent), totalcurrent, sizeof(uint16_t)*4);
    memcpy(&(_payload->rpm), rpm, sizeof(uint16_t)*4);
    memcpy(&(_payload->count), count, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(uint8_t)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_telemetry_5_to_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_telemetry_5_to_8_pack(
        _msg, sysid, compid,
        _payload->temperature, _payload->voltage, _payload->current, _payload->totalcurrent, _payload->rpm, _payload->count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const uint8_t* temperature, const uint16_t* voltage, const uint16_t* current, const uint16_t* totalcurrent, const uint16_t* rpm, const uint16_t* count,
    fmav_status_t* _status)
{
    fmav_esc_telemetry_5_to_8_t* _payload = (fmav_esc_telemetry_5_to_8_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->voltage), voltage, sizeof(uint16_t)*4);
    memcpy(&(_payload->current), current, sizeof(uint16_t)*4);
    memcpy(&(_payload->totalcurrent), totalcurrent, sizeof(uint16_t)*4);
    memcpy(&(_payload->rpm), rpm, sizeof(uint16_t)*4);
    memcpy(&(_payload->count), count, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(uint8_t)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8 >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8 >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_telemetry_5_to_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_telemetry_5_to_8_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->temperature, _payload->voltage, _payload->current, _payload->totalcurrent, _payload->rpm, _payload->count,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const uint8_t* temperature, const uint16_t* voltage, const uint16_t* current, const uint16_t* totalcurrent, const uint16_t* rpm, const uint16_t* count,
    fmav_status_t* _status)
{
    fmav_esc_telemetry_5_to_8_t _payload;


    memcpy(&(_payload.voltage), voltage, sizeof(uint16_t)*4);
    memcpy(&(_payload.current), current, sizeof(uint16_t)*4);
    memcpy(&(_payload.totalcurrent), totalcurrent, sizeof(uint16_t)*4);
    memcpy(&(_payload.rpm), rpm, sizeof(uint16_t)*4);
    memcpy(&(_payload.count), count, sizeof(uint16_t)*4);
    memcpy(&(_payload.temperature), temperature, sizeof(uint8_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_telemetry_5_to_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_TELEMETRY_5_TO_8 decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_telemetry_5_to_8_decode(fmav_esc_telemetry_5_to_8_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_PAYLOAD_LEN_MAX);
#endif
}





FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_telemetry_5_to_8_get_field_voltage_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_get_field_voltage(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_VOLTAGE_NUM) return 0;
    return ((uint16_t*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_telemetry_5_to_8_get_field_current_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_get_field_current(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_CURRENT_NUM) return 0;
    return ((uint16_t*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_telemetry_5_to_8_get_field_totalcurrent_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_get_field_totalcurrent(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TOTALCURRENT_NUM) return 0;
    return ((uint16_t*)&(msg->payload[16]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_telemetry_5_to_8_get_field_rpm_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[24]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_get_field_rpm(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_RPM_NUM) return 0;
    return ((uint16_t*)&(msg->payload[24]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_telemetry_5_to_8_get_field_count_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_telemetry_5_to_8_get_field_count(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_COUNT_NUM) return 0;
    return ((uint16_t*)&(msg->payload[32]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_esc_telemetry_5_to_8_get_field_temperature_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[40]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_telemetry_5_to_8_get_field_temperature(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TEMPERATURE_NUM) return 0;
    return ((uint8_t*)&(msg->payload[40]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8  11031

#define mavlink_esc_telemetry_5_to_8_t  fmav_esc_telemetry_5_to_8_t

#define MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_LEN  44
#define MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_MIN_LEN  44
#define MAVLINK_MSG_ID_11031_LEN  44
#define MAVLINK_MSG_ID_11031_MIN_LEN  44

#define MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_CRC  133
#define MAVLINK_MSG_ID_11031_CRC  133

#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_CURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TOTALCURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_COUNT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_5_TO_8_FIELD_TEMPERATURE_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_telemetry_5_to_8_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const uint8_t* temperature, const uint16_t* voltage, const uint16_t* current, const uint16_t* totalcurrent, const uint16_t* rpm, const uint16_t* count)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_telemetry_5_to_8_pack(
        _msg, sysid, compid,
        temperature, voltage, current, totalcurrent, rpm, count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_telemetry_5_to_8_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_esc_telemetry_5_to_8_t* _payload)
{
    return mavlink_msg_esc_telemetry_5_to_8_pack(
        sysid,
        compid,
        _msg,
        _payload->temperature, _payload->voltage, _payload->current, _payload->totalcurrent, _payload->rpm, _payload->count);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_telemetry_5_to_8_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const uint8_t* temperature, const uint16_t* voltage, const uint16_t* current, const uint16_t* totalcurrent, const uint16_t* rpm, const uint16_t* count)
{
    return fmav_msg_esc_telemetry_5_to_8_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        temperature, voltage, current, totalcurrent, rpm, count,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_telemetry_5_to_8_decode(const mavlink_message_t* msg, mavlink_esc_telemetry_5_to_8_t* payload)
{
    fmav_msg_esc_telemetry_5_to_8_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_TELEMETRY_5_TO_8_H

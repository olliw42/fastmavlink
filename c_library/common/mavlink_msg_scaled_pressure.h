//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SCALED_PRESSURE_H
#define FASTMAVLINK_MSG_SCALED_PRESSURE_H


//----------------------------------------
//-- Message SCALED_PRESSURE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_scaled_pressure_t {
    uint32_t time_boot_ms;
    float press_abs;
    float press_diff;
    int16_t temperature;
    int16_t temperature_press_diff;
}) fmav_scaled_pressure_t;


#define FASTMAVLINK_MSG_ID_SCALED_PRESSURE  29

#define FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA  115

#define FASTMAVLINK_MSG_SCALED_PRESSURE_FLAGS  0
#define FASTMAVLINK_MSG_SCALED_PRESSURE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SCALED_PRESSURE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SCALED_PRESSURE_FRAME_LEN_MAX  41



#define FASTMAVLINK_MSG_SCALED_PRESSURE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_SCALED_PRESSURE_FIELD_PRESS_ABS_OFS  4
#define FASTMAVLINK_MSG_SCALED_PRESSURE_FIELD_PRESS_DIFF_OFS  8
#define FASTMAVLINK_MSG_SCALED_PRESSURE_FIELD_TEMPERATURE_OFS  12
#define FASTMAVLINK_MSG_SCALED_PRESSURE_FIELD_TEMPERATURE_PRESS_DIFF_OFS  14


//----------------------------------------
//-- Message SCALED_PRESSURE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff,
    fmav_status_t* _status)
{
    fmav_scaled_pressure_t* _payload = (fmav_scaled_pressure_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->press_abs = press_abs;
    _payload->press_diff = press_diff;
    _payload->temperature = temperature;
    _payload->temperature_press_diff = temperature_press_diff;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SCALED_PRESSURE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_pressure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_pressure_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->press_abs, _payload->press_diff, _payload->temperature, _payload->temperature_press_diff,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff,
    fmav_status_t* _status)
{
    fmav_scaled_pressure_t* _payload = (fmav_scaled_pressure_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->press_abs = press_abs;
    _payload->press_diff = press_diff;
    _payload->temperature = temperature;
    _payload->temperature_press_diff = temperature_press_diff;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_pressure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_pressure_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->press_abs, _payload->press_diff, _payload->temperature, _payload->temperature_press_diff,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff,
    fmav_status_t* _status)
{
    fmav_scaled_pressure_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.press_abs = press_abs;
    _payload.press_diff = press_diff;
    _payload.temperature = temperature;
    _payload.temperature_press_diff = temperature_press_diff;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SCALED_PRESSURE,
        FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_pressure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SCALED_PRESSURE,
        FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_PRESSURE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SCALED_PRESSURE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_scaled_pressure_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_scaled_pressure_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_pressure_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_pressure_decode(fmav_scaled_pressure_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_PRESSURE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_scaled_pressure_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_scaled_pressure_get_field_press_abs(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_scaled_pressure_get_field_press_diff(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_pressure_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_pressure_get_field_temperature_press_diff(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SCALED_PRESSURE  29

#define mavlink_scaled_pressure_t  fmav_scaled_pressure_t

#define MAVLINK_MSG_ID_SCALED_PRESSURE_LEN  16
#define MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN  14
#define MAVLINK_MSG_ID_29_LEN  16
#define MAVLINK_MSG_ID_29_MIN_LEN  14

#define MAVLINK_MSG_ID_SCALED_PRESSURE_CRC  115
#define MAVLINK_MSG_ID_29_CRC  115




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_pressure_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_scaled_pressure_pack(
        msg, sysid, compid,
        time_boot_ms, press_abs, press_diff, temperature, temperature_press_diff,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_pressure_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
    return fmav_msg_scaled_pressure_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, press_abs, press_diff, temperature, temperature_press_diff,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_scaled_pressure_decode(const mavlink_message_t* msg, mavlink_scaled_pressure_t* payload)
{
    fmav_msg_scaled_pressure_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SCALED_PRESSURE_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SCALED_IMU3_H
#define FASTMAVLINK_MSG_SCALED_IMU3_H


//----------------------------------------
//-- Message SCALED_IMU3
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_scaled_imu3_t {
    uint32_t time_boot_ms;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
    int16_t xgyro;
    int16_t ygyro;
    int16_t zgyro;
    int16_t xmag;
    int16_t ymag;
    int16_t zmag;
    int16_t temperature;
}) fmav_scaled_imu3_t;


#define FASTMAVLINK_MSG_ID_SCALED_IMU3  129

#define FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA  46

#define FASTMAVLINK_MSG_SCALED_IMU3_FLAGS  0
#define FASTMAVLINK_MSG_SCALED_IMU3_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SCALED_IMU3_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SCALED_IMU3_FRAME_LEN_MAX  49



#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_XACC_OFS  4
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_YACC_OFS  6
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_ZACC_OFS  8
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_XGYRO_OFS  10
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_YGYRO_OFS  12
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_ZGYRO_OFS  14
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_XMAG_OFS  16
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_YMAG_OFS  18
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_ZMAG_OFS  20
#define FASTMAVLINK_MSG_SCALED_IMU3_FIELD_TEMPERATURE_OFS  22


//----------------------------------------
//-- Message SCALED_IMU3 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu3_t* _payload = (fmav_scaled_imu3_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->temperature = temperature;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SCALED_IMU3;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_imu3_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu3_t* _payload = (fmav_scaled_imu3_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->xmag = xmag;
    _payload->ymag = ymag;
    _payload->zmag = zmag;
    _payload->temperature = temperature;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SCALED_IMU3;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_IMU3 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_IMU3 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_imu3_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->temperature,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_scaled_imu3_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;
    _payload.xgyro = xgyro;
    _payload.ygyro = ygyro;
    _payload.zgyro = zgyro;
    _payload.xmag = xmag;
    _payload.ymag = ymag;
    _payload.zmag = zmag;
    _payload.temperature = temperature;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SCALED_IMU3,
        FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_imu3_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_imu3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SCALED_IMU3,
        FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_IMU3_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SCALED_IMU3 unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_scaled_imu3_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_scaled_imu3_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_imu3_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_imu3_decode(fmav_scaled_imu3_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SCALED_IMU3_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_scaled_imu3_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_xacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_yacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_zacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_xgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_ygyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_zgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_xmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_ymag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_zmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_scaled_imu3_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SCALED_IMU3  129

#define mavlink_scaled_imu3_t  fmav_scaled_imu3_t

#define MAVLINK_MSG_ID_SCALED_IMU3_LEN  24
#define MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN  22
#define MAVLINK_MSG_ID_129_LEN  24
#define MAVLINK_MSG_ID_129_MIN_LEN  22

#define MAVLINK_MSG_ID_SCALED_IMU3_CRC  46
#define MAVLINK_MSG_ID_129_CRC  46




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_imu3_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_scaled_imu3_pack(
        msg, sysid, compid,
        time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, temperature,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_imu3_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
    return fmav_msg_scaled_imu3_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_scaled_imu3_decode(const mavlink_message_t* msg, mavlink_scaled_imu3_t* payload)
{
    fmav_msg_scaled_imu3_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SCALED_IMU3_H

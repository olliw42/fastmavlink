//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RAW_IMU_H
#define FASTMAVLINK_MSG_RAW_IMU_H


//----------------------------------------
//-- Message RAW_IMU
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_raw_imu_t {
    uint64_t time_usec;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
    int16_t xgyro;
    int16_t ygyro;
    int16_t zgyro;
    int16_t xmag;
    int16_t ymag;
    int16_t zmag;
    uint8_t id;
    int16_t temperature;
}) fmav_raw_imu_t;


#define FASTMAVLINK_MSG_ID_RAW_IMU  27

#define FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX  29
#define FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA  144

#define FASTMAVLINK_MSG_RAW_IMU_FLAGS  0
#define FASTMAVLINK_MSG_RAW_IMU_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RAW_IMU_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RAW_IMU_FRAME_LEN_MAX  54



#define FASTMAVLINK_MSG_RAW_IMU_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_XACC_OFS  8
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_YACC_OFS  10
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_ZACC_OFS  12
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_XGYRO_OFS  14
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_YGYRO_OFS  16
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_ZGYRO_OFS  18
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_XMAG_OFS  20
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_YMAG_OFS  22
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_ZMAG_OFS  24
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_ID_OFS  26
#define FASTMAVLINK_MSG_RAW_IMU_FIELD_TEMPERATURE_OFS  27


//----------------------------------------
//-- Message RAW_IMU packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_raw_imu_t* _payload = (fmav_raw_imu_t*)msg->payload;

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
    _payload->id = id;
    _payload->temperature = temperature;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RAW_IMU;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_imu_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_imu_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->id, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_raw_imu_t* _payload = (fmav_raw_imu_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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
    _payload->id = id;
    _payload->temperature = temperature;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RAW_IMU;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_IMU >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_IMU >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_imu_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_imu_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->xmag, _payload->ymag, _payload->zmag, _payload->id, _payload->temperature,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_raw_imu_t _payload;

    _payload.time_usec = time_usec;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;
    _payload.xgyro = xgyro;
    _payload.ygyro = ygyro;
    _payload.zgyro = zgyro;
    _payload.xmag = xmag;
    _payload.ymag = ymag;
    _payload.zmag = zmag;
    _payload.id = id;
    _payload.temperature = temperature;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RAW_IMU,
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_imu_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_imu_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RAW_IMU,
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RAW_IMU unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_raw_imu_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_raw_imu_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_imu_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_imu_decode(fmav_raw_imu_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_raw_imu_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_xacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_yacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_zacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_xgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_ygyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_zgyro(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_xmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_ymag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_zmag(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_raw_imu_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_raw_imu_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RAW_IMU  27

#define mavlink_raw_imu_t  fmav_raw_imu_t

#define MAVLINK_MSG_ID_RAW_IMU_LEN  29
#define MAVLINK_MSG_ID_RAW_IMU_MIN_LEN  26
#define MAVLINK_MSG_ID_27_LEN  29
#define MAVLINK_MSG_ID_27_MIN_LEN  26

#define MAVLINK_MSG_ID_RAW_IMU_CRC  144
#define MAVLINK_MSG_ID_27_CRC  144




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_imu_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_raw_imu_pack(
        msg, sysid, compid,
        time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, id, temperature,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_imu_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature)
{
    return fmav_msg_raw_imu_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, id, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* payload)
{
    fmav_msg_raw_imu_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RAW_IMU_H

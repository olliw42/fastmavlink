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


#define FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MIN  26
#define FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX  29
#define FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN  29
#define FASTMAVLINK_MSG_RAW_IMU_CRCEXTRA  144

#define FASTMAVLINK_MSG_ID_27_LEN_MIN  26
#define FASTMAVLINK_MSG_ID_27_LEN_MAX  29
#define FASTMAVLINK_MSG_ID_27_LEN  29
#define FASTMAVLINK_MSG_ID_27_CRCEXTRA  144



#define FASTMAVLINK_MSG_RAW_IMU_FLAGS  0
#define FASTMAVLINK_MSG_RAW_IMU_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RAW_IMU_TARGET_COMPONENT_OFS  0


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
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message RAW_IMU unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_imu_decode(fmav_raw_imu_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RAW_IMU_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_NAV_FILTER_BIAS_H
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_H


//----------------------------------------
//-- Message NAV_FILTER_BIAS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_nav_filter_bias_t {
    uint64_t usec;
    float accel_0;
    float accel_1;
    float accel_2;
    float gyro_0;
    float gyro_1;
    float gyro_2;
}) fmav_nav_filter_bias_t;


#define FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS  220

#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA  34

#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FLAGS  0
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FRAME_LEN_MAX  57



#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_USEC_OFS  0
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_ACCEL_0_OFS  8
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_ACCEL_1_OFS  12
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_ACCEL_2_OFS  16
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_GYRO_0_OFS  20
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_GYRO_1_OFS  24
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FIELD_GYRO_2_OFS  28


//----------------------------------------
//-- Message NAV_FILTER_BIAS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2,
    fmav_status_t* _status)
{
    fmav_nav_filter_bias_t* _payload = (fmav_nav_filter_bias_t*)msg->payload;

    _payload->usec = usec;
    _payload->accel_0 = accel_0;
    _payload->accel_1 = accel_1;
    _payload->accel_2 = accel_2;
    _payload->gyro_0 = gyro_0;
    _payload->gyro_1 = gyro_1;
    _payload->gyro_2 = gyro_2;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_filter_bias_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_filter_bias_pack(
        msg, sysid, compid,
        _payload->usec, _payload->accel_0, _payload->accel_1, _payload->accel_2, _payload->gyro_0, _payload->gyro_1, _payload->gyro_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2,
    fmav_status_t* _status)
{
    fmav_nav_filter_bias_t* _payload = (fmav_nav_filter_bias_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->usec = usec;
    _payload->accel_0 = accel_0;
    _payload->accel_1 = accel_1;
    _payload->accel_2 = accel_2;
    _payload->gyro_0 = gyro_0;
    _payload->gyro_1 = gyro_1;
    _payload->gyro_2 = gyro_2;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_filter_bias_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_filter_bias_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->usec, _payload->accel_0, _payload->accel_1, _payload->accel_2, _payload->gyro_0, _payload->gyro_1, _payload->gyro_2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2,
    fmav_status_t* _status)
{
    fmav_nav_filter_bias_t _payload;

    _payload.usec = usec;
    _payload.accel_0 = accel_0;
    _payload.accel_1 = accel_1;
    _payload.accel_2 = accel_2;
    _payload.gyro_0 = gyro_0;
    _payload.gyro_1 = gyro_1;
    _payload.gyro_2 = gyro_2;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_filter_bias_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message NAV_FILTER_BIAS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_filter_bias_decode(fmav_nav_filter_bias_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX;

    // memset(payload, 0, FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX); not needed, must have been done before
    memcpy(payload, msg->payload, len);
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_nav_filter_bias_get_field_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_accel_0(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_accel_1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_accel_2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_gyro_0(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_gyro_1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_filter_bias_get_field_gyro_2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS  220

#define mavlink_nav_filter_bias_t  fmav_nav_filter_bias_t

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_LEN  32
#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_MIN_LEN  32
#define MAVLINK_MSG_ID_220_LEN  32
#define MAVLINK_MSG_ID_220_MIN_LEN  32

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS_CRC  34
#define MAVLINK_MSG_ID_220_CRC  34




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_filter_bias_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_nav_filter_bias_pack(
        msg, sysid, compid,
        usec, accel_0, accel_1, accel_2, gyro_0, gyro_1, gyro_2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_filter_bias_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
    return fmav_msg_nav_filter_bias_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        usec, accel_0, accel_1, accel_2, gyro_0, gyro_1, gyro_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_nav_filter_bias_decode(const mavlink_message_t* msg, mavlink_nav_filter_bias_t* payload)
{
    fmav_msg_nav_filter_bias_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_NAV_FILTER_BIAS_H

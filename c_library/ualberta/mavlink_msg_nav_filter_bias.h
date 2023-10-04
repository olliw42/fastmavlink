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

// fields are ordered, as they appear on the wire
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
//-- Message NAV_FILTER_BIAS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2,
    fmav_status_t* _status)
{
    fmav_nav_filter_bias_t* _payload = (fmav_nav_filter_bias_t*)_msg->payload;

    _payload->usec = usec;
    _payload->accel_0 = accel_0;
    _payload->accel_1 = accel_1;
    _payload->accel_2 = accel_2;
    _payload->gyro_0 = gyro_0;
    _payload->gyro_1 = gyro_1;
    _payload->gyro_2 = gyro_2;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_filter_bias_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_filter_bias_pack(
        _msg, sysid, compid,
        _payload->usec, _payload->accel_0, _payload->accel_1, _payload->accel_2, _payload->gyro_0, _payload->gyro_1, _payload->gyro_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2,
    fmav_status_t* _status)
{
    fmav_nav_filter_bias_t* _payload = (fmav_nav_filter_bias_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->usec = usec;
    _payload->accel_0 = accel_0;
    _payload->accel_1 = accel_1;
    _payload->accel_2 = accel_2;
    _payload->gyro_0 = gyro_0;
    _payload->gyro_1 = gyro_1;
    _payload->gyro_2 = gyro_2;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_FILTER_BIAS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_filter_bias_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_filter_bias_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_filter_bias_pack_to_frame_buf(
        _buf, sysid, compid,
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
//-- Message NAV_FILTER_BIAS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_filter_bias_decode(fmav_nav_filter_bias_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX);
#endif
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
    mavlink_message_t* _msg,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_nav_filter_bias_pack(
        _msg, sysid, compid,
        usec, accel_0, accel_1, accel_2, gyro_0, gyro_1, gyro_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_filter_bias_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_nav_filter_bias_t* _payload)
{
    return mavlink_msg_nav_filter_bias_pack(
        sysid,
        compid,
        _msg,
        _payload->usec, _payload->accel_0, _payload->accel_1, _payload->accel_2, _payload->gyro_0, _payload->gyro_1, _payload->gyro_2);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_filter_bias_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
    return fmav_msg_nav_filter_bias_pack_to_frame_buf(
        (uint8_t*)_buf,
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

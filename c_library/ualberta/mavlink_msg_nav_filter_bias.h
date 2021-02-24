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


#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN  32
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_CRCEXTRA  34

#define FASTMAVLINK_MSG_ID_220_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_220_LEN_MAX  32
#define FASTMAVLINK_MSG_ID_220_LEN  32
#define FASTMAVLINK_MSG_ID_220_CRCEXTRA  34



#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_FLAGS  0
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAV_FILTER_BIAS_TARGET_COMPONENT_OFS  0


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
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message NAV_FILTER_BIAS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_filter_bias_decode(fmav_nav_filter_bias_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_NAV_FILTER_BIAS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

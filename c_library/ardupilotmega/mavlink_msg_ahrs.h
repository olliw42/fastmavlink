//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AHRS_H
#define FASTMAVLINK_MSG_AHRS_H


//----------------------------------------
//-- Message AHRS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ahrs_t {
    float omegaIx;
    float omegaIy;
    float omegaIz;
    float accel_weight;
    float renorm_val;
    float error_rp;
    float error_yaw;
}) fmav_ahrs_t;


#define FASTMAVLINK_MSG_ID_AHRS  163


#define FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MIN  28
#define FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN  28
#define FASTMAVLINK_MSG_AHRS_CRCEXTRA  127

#define FASTMAVLINK_MSG_ID_163_LEN_MIN  28
#define FASTMAVLINK_MSG_ID_163_LEN_MAX  28
#define FASTMAVLINK_MSG_ID_163_LEN  28
#define FASTMAVLINK_MSG_ID_163_CRCEXTRA  127



#define FASTMAVLINK_MSG_AHRS_FLAGS  0
#define FASTMAVLINK_MSG_AHRS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AHRS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message AHRS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw,
    fmav_status_t* _status)
{
    fmav_ahrs_t* _payload = (fmav_ahrs_t*)msg->payload;

    _payload->omegaIx = omegaIx;
    _payload->omegaIy = omegaIy;
    _payload->omegaIz = omegaIz;
    _payload->accel_weight = accel_weight;
    _payload->renorm_val = renorm_val;
    _payload->error_rp = error_rp;
    _payload->error_yaw = error_yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AHRS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AHRS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs_pack(
        msg, sysid, compid,
        _payload->omegaIx, _payload->omegaIy, _payload->omegaIz, _payload->accel_weight, _payload->renorm_val, _payload->error_rp, _payload->error_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw,
    fmav_status_t* _status)
{
    fmav_ahrs_t* _payload = (fmav_ahrs_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->omegaIx = omegaIx;
    _payload->omegaIy = omegaIy;
    _payload->omegaIz = omegaIz;
    _payload->accel_weight = accel_weight;
    _payload->renorm_val = renorm_val;
    _payload->error_rp = error_rp;
    _payload->error_yaw = error_yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AHRS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->omegaIx, _payload->omegaIy, _payload->omegaIz, _payload->accel_weight, _payload->renorm_val, _payload->error_rp, _payload->error_yaw,
        _status);
}


//----------------------------------------
//-- Message AHRS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ahrs_decode(fmav_ahrs_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AHRS  163

#define mavlink_ahrs_t  fmav_ahrs_t

#define MAVLINK_MSG_ID_AHRS_LEN  28
#define MAVLINK_MSG_ID_AHRS_MIN_LEN  28
#define MAVLINK_MSG_ID_163_LEN  28
#define MAVLINK_MSG_ID_163_MIN_LEN  28

#define MAVLINK_MSG_ID_AHRS_CRC  127
#define MAVLINK_MSG_ID_163_CRC  127




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ahrs_pack(
        msg, sysid, compid,
        omegaIx, omegaIy, omegaIz, accel_weight, renorm_val, error_rp, error_yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
    return fmav_msg_ahrs_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        omegaIx, omegaIy, omegaIz, accel_weight, renorm_val, error_rp, error_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ahrs_decode(const mavlink_message_t* msg, mavlink_ahrs_t* payload)
{
    fmav_msg_ahrs_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AHRS_H

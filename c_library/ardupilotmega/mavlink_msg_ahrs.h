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

// fields are ordered, as they appear on the wire
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

#define FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_AHRS_CRCEXTRA  127

#define FASTMAVLINK_MSG_AHRS_FLAGS  0
#define FASTMAVLINK_MSG_AHRS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AHRS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AHRS_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_AHRS_FIELD_OMEGAIX_OFS  0
#define FASTMAVLINK_MSG_AHRS_FIELD_OMEGAIY_OFS  4
#define FASTMAVLINK_MSG_AHRS_FIELD_OMEGAIZ_OFS  8
#define FASTMAVLINK_MSG_AHRS_FIELD_ACCEL_WEIGHT_OFS  12
#define FASTMAVLINK_MSG_AHRS_FIELD_RENORM_VAL_OFS  16
#define FASTMAVLINK_MSG_AHRS_FIELD_ERROR_RP_OFS  20
#define FASTMAVLINK_MSG_AHRS_FIELD_ERROR_YAW_OFS  24


//----------------------------------------
//-- Message AHRS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw,
    fmav_status_t* _status)
{
    fmav_ahrs_t* _payload = (fmav_ahrs_t*)_msg->payload;

    _payload->omegaIx = omegaIx;
    _payload->omegaIy = omegaIy;
    _payload->omegaIz = omegaIz;
    _payload->accel_weight = accel_weight;
    _payload->renorm_val = renorm_val;
    _payload->error_rp = error_rp;
    _payload->error_yaw = error_yaw;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AHRS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AHRS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs_pack(
        _msg, sysid, compid,
        _payload->omegaIx, _payload->omegaIy, _payload->omegaIz, _payload->accel_weight, _payload->renorm_val, _payload->error_rp, _payload->error_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw,
    fmav_status_t* _status)
{
    fmav_ahrs_t* _payload = (fmav_ahrs_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->omegaIx = omegaIx;
    _payload->omegaIy = omegaIy;
    _payload->omegaIz = omegaIz;
    _payload->accel_weight = accel_weight;
    _payload->renorm_val = renorm_val;
    _payload->error_rp = error_rp;
    _payload->error_yaw = error_yaw;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AHRS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->omegaIx, _payload->omegaIy, _payload->omegaIz, _payload->accel_weight, _payload->renorm_val, _payload->error_rp, _payload->error_yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw,
    fmav_status_t* _status)
{
    fmav_ahrs_t _payload;

    _payload.omegaIx = omegaIx;
    _payload.omegaIy = omegaIy;
    _payload.omegaIz = omegaIz;
    _payload.accel_weight = accel_weight;
    _payload.renorm_val = renorm_val;
    _payload.error_rp = error_rp;
    _payload.error_yaw = error_yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AHRS,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AHRS,
        FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AHRS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ahrs_decode(fmav_ahrs_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AHRS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_omegaIx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_omegaIy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_omegaIz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_accel_weight(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_renorm_val(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_error_rp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs_get_field_error_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
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
    mavlink_message_t* _msg,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ahrs_pack(
        _msg, sysid, compid,
        omegaIx, omegaIy, omegaIz, accel_weight, renorm_val, error_rp, error_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_ahrs_t* _payload)
{
    return mavlink_msg_ahrs_pack(
        sysid,
        compid,
        _msg,
        _payload->omegaIx, _payload->omegaIy, _payload->omegaIz, _payload->accel_weight, _payload->renorm_val, _payload->error_rp, _payload->error_yaw);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
    return fmav_msg_ahrs_pack_to_frame_buf(
        (uint8_t*)_buf,
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

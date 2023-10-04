//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EKF_EXT_H
#define FASTMAVLINK_MSG_EKF_EXT_H


//----------------------------------------
//-- Message EKF_EXT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ekf_ext_t {
    uint64_t timestamp;
    float Windspeed;
    float WindDir;
    float WindZ;
    float Airspeed;
    float beta;
    float alpha;
}) fmav_ekf_ext_t;


#define FASTMAVLINK_MSG_ID_EKF_EXT  8007

#define FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_EKF_EXT_CRCEXTRA  64

#define FASTMAVLINK_MSG_EKF_EXT_FLAGS  0
#define FASTMAVLINK_MSG_EKF_EXT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EKF_EXT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_EKF_EXT_FRAME_LEN_MAX  57



#define FASTMAVLINK_MSG_EKF_EXT_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_WINDSPEED_OFS  8
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_WINDDIR_OFS  12
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_WINDZ_OFS  16
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_AIRSPEED_OFS  20
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_BETA_OFS  24
#define FASTMAVLINK_MSG_EKF_EXT_FIELD_ALPHA_OFS  28


//----------------------------------------
//-- Message EKF_EXT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha,
    fmav_status_t* _status)
{
    fmav_ekf_ext_t* _payload = (fmav_ekf_ext_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->Windspeed = Windspeed;
    _payload->WindDir = WindDir;
    _payload->WindZ = WindZ;
    _payload->Airspeed = Airspeed;
    _payload->beta = beta;
    _payload->alpha = alpha;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_EKF_EXT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_EKF_EXT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_ext_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->Windspeed, _payload->WindDir, _payload->WindZ, _payload->Airspeed, _payload->beta, _payload->alpha,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha,
    fmav_status_t* _status)
{
    fmav_ekf_ext_t* _payload = (fmav_ekf_ext_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->Windspeed = Windspeed;
    _payload->WindDir = WindDir;
    _payload->WindZ = WindZ;
    _payload->Airspeed = Airspeed;
    _payload->beta = beta;
    _payload->alpha = alpha;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EKF_EXT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_EXT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_EXT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_EXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_ext_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->Windspeed, _payload->WindDir, _payload->WindZ, _payload->Airspeed, _payload->beta, _payload->alpha,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha,
    fmav_status_t* _status)
{
    fmav_ekf_ext_t _payload;

    _payload.timestamp = timestamp;
    _payload.Windspeed = Windspeed;
    _payload.WindDir = WindDir;
    _payload.WindZ = WindZ;
    _payload.Airspeed = Airspeed;
    _payload.beta = beta;
    _payload.alpha = alpha;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_EKF_EXT,
        FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_EXT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_ext_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_ext_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_EKF_EXT,
        FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_EXT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message EKF_EXT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ekf_ext_decode(fmav_ekf_ext_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_EXT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_ekf_ext_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_Windspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_WindDir(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_WindZ(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_Airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_beta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_ext_get_field_alpha(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EKF_EXT  8007

#define mavlink_ekf_ext_t  fmav_ekf_ext_t

#define MAVLINK_MSG_ID_EKF_EXT_LEN  32
#define MAVLINK_MSG_ID_EKF_EXT_MIN_LEN  32
#define MAVLINK_MSG_ID_8007_LEN  32
#define MAVLINK_MSG_ID_8007_MIN_LEN  32

#define MAVLINK_MSG_ID_EKF_EXT_CRC  64
#define MAVLINK_MSG_ID_8007_CRC  64




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_ext_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ekf_ext_pack(
        _msg, sysid, compid,
        timestamp, Windspeed, WindDir, WindZ, Airspeed, beta, alpha,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_ext_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_ekf_ext_t* _payload)
{
    return mavlink_msg_ekf_ext_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->Windspeed, _payload->WindDir, _payload->WindZ, _payload->Airspeed, _payload->beta, _payload->alpha);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_ext_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float Windspeed, float WindDir, float WindZ, float Airspeed, float beta, float alpha)
{
    return fmav_msg_ekf_ext_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, Windspeed, WindDir, WindZ, Airspeed, beta, alpha,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ekf_ext_decode(const mavlink_message_t* msg, mavlink_ekf_ext_t* payload)
{
    fmav_msg_ekf_ext_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EKF_EXT_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENS_MPPT_H
#define FASTMAVLINK_MSG_SENS_MPPT_H


//----------------------------------------
//-- Message SENS_MPPT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sens_mppt_t {
    uint64_t mppt_timestamp;
    float mppt1_volt;
    float mppt1_amp;
    float mppt2_volt;
    float mppt2_amp;
    float mppt3_volt;
    float mppt3_amp;
    uint16_t mppt1_pwm;
    uint16_t mppt2_pwm;
    uint16_t mppt3_pwm;
    uint8_t mppt1_status;
    uint8_t mppt2_status;
    uint8_t mppt3_status;
}) fmav_sens_mppt_t;


#define FASTMAVLINK_MSG_ID_SENS_MPPT  8003

#define FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_SENS_MPPT_CRCEXTRA  231

#define FASTMAVLINK_MSG_SENS_MPPT_FLAGS  0
#define FASTMAVLINK_MSG_SENS_MPPT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENS_MPPT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENS_MPPT_FRAME_LEN_MAX  66



#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT1_VOLT_OFS  8
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT1_AMP_OFS  12
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT2_VOLT_OFS  16
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT2_AMP_OFS  20
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT3_VOLT_OFS  24
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT3_AMP_OFS  28
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT1_PWM_OFS  32
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT2_PWM_OFS  34
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT3_PWM_OFS  36
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT1_STATUS_OFS  38
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT2_STATUS_OFS  39
#define FASTMAVLINK_MSG_SENS_MPPT_FIELD_MPPT3_STATUS_OFS  40


//----------------------------------------
//-- Message SENS_MPPT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status,
    fmav_status_t* _status)
{
    fmav_sens_mppt_t* _payload = (fmav_sens_mppt_t*)_msg->payload;

    _payload->mppt_timestamp = mppt_timestamp;
    _payload->mppt1_volt = mppt1_volt;
    _payload->mppt1_amp = mppt1_amp;
    _payload->mppt2_volt = mppt2_volt;
    _payload->mppt2_amp = mppt2_amp;
    _payload->mppt3_volt = mppt3_volt;
    _payload->mppt3_amp = mppt3_amp;
    _payload->mppt1_pwm = mppt1_pwm;
    _payload->mppt2_pwm = mppt2_pwm;
    _payload->mppt3_pwm = mppt3_pwm;
    _payload->mppt1_status = mppt1_status;
    _payload->mppt2_status = mppt2_status;
    _payload->mppt3_status = mppt3_status;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENS_MPPT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENS_MPPT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_mppt_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_mppt_pack(
        _msg, sysid, compid,
        _payload->mppt_timestamp, _payload->mppt1_volt, _payload->mppt1_amp, _payload->mppt1_pwm, _payload->mppt1_status, _payload->mppt2_volt, _payload->mppt2_amp, _payload->mppt2_pwm, _payload->mppt2_status, _payload->mppt3_volt, _payload->mppt3_amp, _payload->mppt3_pwm, _payload->mppt3_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status,
    fmav_status_t* _status)
{
    fmav_sens_mppt_t* _payload = (fmav_sens_mppt_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mppt_timestamp = mppt_timestamp;
    _payload->mppt1_volt = mppt1_volt;
    _payload->mppt1_amp = mppt1_amp;
    _payload->mppt2_volt = mppt2_volt;
    _payload->mppt2_amp = mppt2_amp;
    _payload->mppt3_volt = mppt3_volt;
    _payload->mppt3_amp = mppt3_amp;
    _payload->mppt1_pwm = mppt1_pwm;
    _payload->mppt2_pwm = mppt2_pwm;
    _payload->mppt3_pwm = mppt3_pwm;
    _payload->mppt1_status = mppt1_status;
    _payload->mppt2_status = mppt2_status;
    _payload->mppt3_status = mppt3_status;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENS_MPPT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_MPPT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_MPPT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_MPPT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_mppt_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_mppt_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->mppt_timestamp, _payload->mppt1_volt, _payload->mppt1_amp, _payload->mppt1_pwm, _payload->mppt1_status, _payload->mppt2_volt, _payload->mppt2_amp, _payload->mppt2_pwm, _payload->mppt2_status, _payload->mppt3_volt, _payload->mppt3_amp, _payload->mppt3_pwm, _payload->mppt3_status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status,
    fmav_status_t* _status)
{
    fmav_sens_mppt_t _payload;

    _payload.mppt_timestamp = mppt_timestamp;
    _payload.mppt1_volt = mppt1_volt;
    _payload.mppt1_amp = mppt1_amp;
    _payload.mppt2_volt = mppt2_volt;
    _payload.mppt2_amp = mppt2_amp;
    _payload.mppt3_volt = mppt3_volt;
    _payload.mppt3_amp = mppt3_amp;
    _payload.mppt1_pwm = mppt1_pwm;
    _payload.mppt2_pwm = mppt2_pwm;
    _payload.mppt3_pwm = mppt3_pwm;
    _payload.mppt1_status = mppt1_status;
    _payload.mppt2_status = mppt2_status;
    _payload.mppt3_status = mppt3_status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENS_MPPT,
        FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_MPPT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_mppt_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENS_MPPT,
        FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_MPPT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENS_MPPT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sens_mppt_decode(fmav_sens_mppt_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_MPPT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_sens_mppt_get_field_mppt_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt1_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt1_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt2_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt2_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt3_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_mppt_get_field_mppt3_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_get_field_mppt1_pwm(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_get_field_mppt2_pwm(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_mppt_get_field_mppt3_pwm(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_mppt_get_field_mppt1_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_mppt_get_field_mppt2_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[39]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_mppt_get_field_mppt3_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENS_MPPT  8003

#define mavlink_sens_mppt_t  fmav_sens_mppt_t

#define MAVLINK_MSG_ID_SENS_MPPT_LEN  41
#define MAVLINK_MSG_ID_SENS_MPPT_MIN_LEN  41
#define MAVLINK_MSG_ID_8003_LEN  41
#define MAVLINK_MSG_ID_8003_MIN_LEN  41

#define MAVLINK_MSG_ID_SENS_MPPT_CRC  231
#define MAVLINK_MSG_ID_8003_CRC  231




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_mppt_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sens_mppt_pack(
        _msg, sysid, compid,
        mppt_timestamp, mppt1_volt, mppt1_amp, mppt1_pwm, mppt1_status, mppt2_volt, mppt2_amp, mppt2_pwm, mppt2_status, mppt3_volt, mppt3_amp, mppt3_pwm, mppt3_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_mppt_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sens_mppt_t* _payload)
{
    return mavlink_msg_sens_mppt_pack(
        sysid,
        compid,
        _msg,
        _payload->mppt_timestamp, _payload->mppt1_volt, _payload->mppt1_amp, _payload->mppt1_pwm, _payload->mppt1_status, _payload->mppt2_volt, _payload->mppt2_amp, _payload->mppt2_pwm, _payload->mppt2_status, _payload->mppt3_volt, _payload->mppt3_amp, _payload->mppt3_pwm, _payload->mppt3_status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_mppt_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t mppt_timestamp, float mppt1_volt, float mppt1_amp, uint16_t mppt1_pwm, uint8_t mppt1_status, float mppt2_volt, float mppt2_amp, uint16_t mppt2_pwm, uint8_t mppt2_status, float mppt3_volt, float mppt3_amp, uint16_t mppt3_pwm, uint8_t mppt3_status)
{
    return fmav_msg_sens_mppt_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        mppt_timestamp, mppt1_volt, mppt1_amp, mppt1_pwm, mppt1_status, mppt2_volt, mppt2_amp, mppt2_pwm, mppt2_status, mppt3_volt, mppt3_amp, mppt3_pwm, mppt3_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sens_mppt_decode(const mavlink_message_t* msg, mavlink_sens_mppt_t* payload)
{
    fmav_msg_sens_mppt_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENS_MPPT_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENS_POWER_H
#define FASTMAVLINK_MSG_SENS_POWER_H


//----------------------------------------
//-- Message SENS_POWER
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sens_power_t {
    float adc121_vspb_volt;
    float adc121_cspb_amp;
    float adc121_cs1_amp;
    float adc121_cs2_amp;
}) fmav_sens_power_t;


#define FASTMAVLINK_MSG_ID_SENS_POWER  8002

#define FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_SENS_POWER_CRCEXTRA  218

#define FASTMAVLINK_MSG_SENS_POWER_FLAGS  0
#define FASTMAVLINK_MSG_SENS_POWER_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENS_POWER_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENS_POWER_FRAME_LEN_MAX  41



#define FASTMAVLINK_MSG_SENS_POWER_FIELD_ADC121_VSPB_VOLT_OFS  0
#define FASTMAVLINK_MSG_SENS_POWER_FIELD_ADC121_CSPB_AMP_OFS  4
#define FASTMAVLINK_MSG_SENS_POWER_FIELD_ADC121_CS1_AMP_OFS  8
#define FASTMAVLINK_MSG_SENS_POWER_FIELD_ADC121_CS2_AMP_OFS  12


//----------------------------------------
//-- Message SENS_POWER pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_t* _payload = (fmav_sens_power_t*)_msg->payload;

    _payload->adc121_vspb_volt = adc121_vspb_volt;
    _payload->adc121_cspb_amp = adc121_cspb_amp;
    _payload->adc121_cs1_amp = adc121_cs1_amp;
    _payload->adc121_cs2_amp = adc121_cs2_amp;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENS_POWER;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENS_POWER_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_power_pack(
        _msg, sysid, compid,
        _payload->adc121_vspb_volt, _payload->adc121_cspb_amp, _payload->adc121_cs1_amp, _payload->adc121_cs2_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_t* _payload = (fmav_sens_power_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->adc121_vspb_volt = adc121_vspb_volt;
    _payload->adc121_cspb_amp = adc121_cspb_amp;
    _payload->adc121_cs1_amp = adc121_cs1_amp;
    _payload->adc121_cs2_amp = adc121_cs2_amp;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENS_POWER;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_POWER >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_POWER >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_power_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->adc121_vspb_volt, _payload->adc121_cspb_amp, _payload->adc121_cs1_amp, _payload->adc121_cs2_amp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_t _payload;

    _payload.adc121_vspb_volt = adc121_vspb_volt;
    _payload.adc121_cspb_amp = adc121_cspb_amp;
    _payload.adc121_cs1_amp = adc121_cs1_amp;
    _payload.adc121_cs2_amp = adc121_cs2_amp;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENS_POWER,
        FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENS_POWER,
        FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENS_POWER decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sens_power_decode(fmav_sens_power_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_POWER_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_get_field_adc121_vspb_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_get_field_adc121_cspb_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_get_field_adc121_cs1_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_get_field_adc121_cs2_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENS_POWER  8002

#define mavlink_sens_power_t  fmav_sens_power_t

#define MAVLINK_MSG_ID_SENS_POWER_LEN  16
#define MAVLINK_MSG_ID_SENS_POWER_MIN_LEN  16
#define MAVLINK_MSG_ID_8002_LEN  16
#define MAVLINK_MSG_ID_8002_MIN_LEN  16

#define MAVLINK_MSG_ID_SENS_POWER_CRC  218
#define MAVLINK_MSG_ID_8002_CRC  218




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sens_power_pack(
        _msg, sysid, compid,
        adc121_vspb_volt, adc121_cspb_amp, adc121_cs1_amp, adc121_cs2_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sens_power_t* _payload)
{
    return mavlink_msg_sens_power_pack(
        sysid,
        compid,
        _msg,
        _payload->adc121_vspb_volt, _payload->adc121_cspb_amp, _payload->adc121_cs1_amp, _payload->adc121_cs2_amp);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float adc121_vspb_volt, float adc121_cspb_amp, float adc121_cs1_amp, float adc121_cs2_amp)
{
    return fmav_msg_sens_power_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        adc121_vspb_volt, adc121_cspb_amp, adc121_cs1_amp, adc121_cs2_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sens_power_decode(const mavlink_message_t* msg, mavlink_sens_power_t* payload)
{
    fmav_msg_sens_power_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENS_POWER_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AP_ADC_H
#define FASTMAVLINK_MSG_AP_ADC_H


//----------------------------------------
//-- Message AP_ADC
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ap_adc_t {
    uint16_t adc1;
    uint16_t adc2;
    uint16_t adc3;
    uint16_t adc4;
    uint16_t adc5;
    uint16_t adc6;
}) fmav_ap_adc_t;


#define FASTMAVLINK_MSG_ID_AP_ADC  153

#define FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_AP_ADC_CRCEXTRA  188

#define FASTMAVLINK_MSG_AP_ADC_FLAGS  0
#define FASTMAVLINK_MSG_AP_ADC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AP_ADC_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AP_ADC_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC1_OFS  0
#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC2_OFS  2
#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC3_OFS  4
#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC4_OFS  6
#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC5_OFS  8
#define FASTMAVLINK_MSG_AP_ADC_FIELD_ADC6_OFS  10


//----------------------------------------
//-- Message AP_ADC pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6,
    fmav_status_t* _status)
{
    fmav_ap_adc_t* _payload = (fmav_ap_adc_t*)_msg->payload;

    _payload->adc1 = adc1;
    _payload->adc2 = adc2;
    _payload->adc3 = adc3;
    _payload->adc4 = adc4;
    _payload->adc5 = adc5;
    _payload->adc6 = adc6;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AP_ADC;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AP_ADC_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ap_adc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ap_adc_pack(
        _msg, sysid, compid,
        _payload->adc1, _payload->adc2, _payload->adc3, _payload->adc4, _payload->adc5, _payload->adc6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6,
    fmav_status_t* _status)
{
    fmav_ap_adc_t* _payload = (fmav_ap_adc_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->adc1 = adc1;
    _payload->adc2 = adc2;
    _payload->adc3 = adc3;
    _payload->adc4 = adc4;
    _payload->adc5 = adc5;
    _payload->adc6 = adc6;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AP_ADC;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AP_ADC >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AP_ADC >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AP_ADC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ap_adc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ap_adc_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->adc1, _payload->adc2, _payload->adc3, _payload->adc4, _payload->adc5, _payload->adc6,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6,
    fmav_status_t* _status)
{
    fmav_ap_adc_t _payload;

    _payload.adc1 = adc1;
    _payload.adc2 = adc2;
    _payload.adc3 = adc3;
    _payload.adc4 = adc4;
    _payload.adc5 = adc5;
    _payload.adc6 = adc6;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AP_ADC,
        FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AP_ADC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ap_adc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AP_ADC,
        FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AP_ADC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AP_ADC decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ap_adc_decode(fmav_ap_adc_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc1(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc2(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc3(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc4(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc5(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_get_field_adc6(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AP_ADC  153

#define mavlink_ap_adc_t  fmav_ap_adc_t

#define MAVLINK_MSG_ID_AP_ADC_LEN  12
#define MAVLINK_MSG_ID_AP_ADC_MIN_LEN  12
#define MAVLINK_MSG_ID_153_LEN  12
#define MAVLINK_MSG_ID_153_MIN_LEN  12

#define MAVLINK_MSG_ID_AP_ADC_CRC  188
#define MAVLINK_MSG_ID_153_CRC  188




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ap_adc_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ap_adc_pack(
        _msg, sysid, compid,
        adc1, adc2, adc3, adc4, adc5, adc6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ap_adc_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_ap_adc_t* _payload)
{
    return mavlink_msg_ap_adc_pack(
        sysid,
        compid,
        _msg,
        _payload->adc1, _payload->adc2, _payload->adc3, _payload->adc4, _payload->adc5, _payload->adc6);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ap_adc_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
    return fmav_msg_ap_adc_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        adc1, adc2, adc3, adc4, adc5, adc6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ap_adc_decode(const mavlink_message_t* msg, mavlink_ap_adc_t* payload)
{
    fmav_msg_ap_adc_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AP_ADC_H

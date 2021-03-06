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

// fields are ordered, as they are on the wire
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
//-- Message AP_ADC packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6,
    fmav_status_t* _status)
{
    fmav_ap_adc_t* _payload = (fmav_ap_adc_t*)msg->payload;

    _payload->adc1 = adc1;
    _payload->adc2 = adc2;
    _payload->adc3 = adc3;
    _payload->adc4 = adc4;
    _payload->adc5 = adc5;
    _payload->adc6 = adc6;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AP_ADC;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AP_ADC_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ap_adc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ap_adc_pack(
        msg, sysid, compid,
        _payload->adc1, _payload->adc2, _payload->adc3, _payload->adc4, _payload->adc5, _payload->adc6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6,
    fmav_status_t* _status)
{
    fmav_ap_adc_t* _payload = (fmav_ap_adc_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->adc1 = adc1;
    _payload->adc2 = adc2;
    _payload->adc3 = adc3;
    _payload->adc4 = adc4;
    _payload->adc5 = adc5;
    _payload->adc6 = adc6;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AP_ADC;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AP_ADC >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AP_ADC >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AP_ADC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ap_adc_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ap_adc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ap_adc_pack_to_frame_buf(
        buf, sysid, compid,
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
//-- Message AP_ADC unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_ap_adc_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_ap_adc_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ap_adc_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ap_adc_decode(fmav_ap_adc_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AP_ADC_PAYLOAD_LEN_MAX);
    }
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
    mavlink_message_t* msg,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ap_adc_pack(
        msg, sysid, compid,
        adc1, adc2, adc3, adc4, adc5, adc6,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ap_adc_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
    return fmav_msg_ap_adc_pack_to_frame_buf(
        (uint8_t*)buf,
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

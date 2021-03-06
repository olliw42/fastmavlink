//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CELLULAR_CONFIG_H
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_H


//----------------------------------------
//-- Message CELLULAR_CONFIG
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_cellular_config_t {
    uint8_t enable_lte;
    uint8_t enable_pin;
    char pin[16];
    char new_pin[16];
    char apn[32];
    char puk[16];
    uint8_t roaming;
    uint8_t response;
}) fmav_cellular_config_t;


#define FASTMAVLINK_MSG_ID_CELLULAR_CONFIG  336

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX  84
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA  245

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FLAGS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FRAME_LEN_MAX  109

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ENABLE_LTE_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ENABLE_PIN_OFS  1
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_OFS  2
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_OFS  18
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_OFS  34
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_OFS  66
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_ROAMING_OFS  82
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_RESPONSE_OFS  83


//----------------------------------------
//-- Message CELLULAR_CONFIG packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t* _payload = (fmav_cellular_config_t*)msg->payload;

    _payload->enable_lte = enable_lte;
    _payload->enable_pin = enable_pin;
    _payload->roaming = roaming;
    _payload->response = response;
    memcpy(&(_payload->pin), pin, sizeof(char)*16);
    memcpy(&(_payload->new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload->apn), apn, sizeof(char)*32);
    memcpy(&(_payload->puk), puk, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CELLULAR_CONFIG;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_config_pack(
        msg, sysid, compid,
        _payload->enable_lte, _payload->enable_pin, _payload->pin, _payload->new_pin, _payload->apn, _payload->puk, _payload->roaming, _payload->response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t* _payload = (fmav_cellular_config_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->enable_lte = enable_lte;
    _payload->enable_pin = enable_pin;
    _payload->roaming = roaming;
    _payload->response = response;
    memcpy(&(_payload->pin), pin, sizeof(char)*16);
    memcpy(&(_payload->new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload->apn), apn, sizeof(char)*32);
    memcpy(&(_payload->puk), puk, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_CONFIG >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_config_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->enable_lte, _payload->enable_pin, _payload->pin, _payload->new_pin, _payload->apn, _payload->puk, _payload->roaming, _payload->response,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response,
    fmav_status_t* _status)
{
    fmav_cellular_config_t _payload;

    _payload.enable_lte = enable_lte;
    _payload.enable_pin = enable_pin;
    _payload.roaming = roaming;
    _payload.response = response;
    memcpy(&(_payload.pin), pin, sizeof(char)*16);
    memcpy(&(_payload.new_pin), new_pin, sizeof(char)*16);
    memcpy(&(_payload.apn), apn, sizeof(char)*32);
    memcpy(&(_payload.puk), puk, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_CONFIG,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_config_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_CONFIG,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CELLULAR_CONFIG unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_cellular_config_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_cellular_config_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_config_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_config_decode(fmav_cellular_config_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_enable_lte(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_enable_pin(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_roaming(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[82]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_cellular_config_get_field_response(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[83]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_pin_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_pin(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_NUM) return 0;
    return ((char*)&(msg->payload[2]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_new_pin_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[18]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_new_pin(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_NUM) return 0;
    return ((char*)&(msg->payload[18]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_apn_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[34]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_apn(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_NUM) return 0;
    return ((char*)&(msg->payload[34]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_cellular_config_get_field_puk_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[66]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_cellular_config_get_field_puk(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_NUM) return 0;
    return ((char*)&(msg->payload[66]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CELLULAR_CONFIG  336

#define mavlink_cellular_config_t  fmav_cellular_config_t

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_LEN  84
#define MAVLINK_MSG_ID_CELLULAR_CONFIG_MIN_LEN  84
#define MAVLINK_MSG_ID_336_LEN  84
#define MAVLINK_MSG_ID_336_MIN_LEN  84

#define MAVLINK_MSG_ID_CELLULAR_CONFIG_CRC  245
#define MAVLINK_MSG_ID_336_CRC  245

#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN 16
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN 32
#define MAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_config_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_cellular_config_pack(
        msg, sysid, compid,
        enable_lte, enable_pin, pin, new_pin, apn, puk, roaming, response,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_config_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t enable_lte, uint8_t enable_pin, const char* pin, const char* new_pin, const char* apn, const char* puk, uint8_t roaming, uint8_t response)
{
    return fmav_msg_cellular_config_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        enable_lte, enable_pin, pin, new_pin, apn, puk, roaming, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_cellular_config_decode(const mavlink_message_t* msg, mavlink_cellular_config_t* payload)
{
    fmav_msg_cellular_config_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CELLULAR_CONFIG_H

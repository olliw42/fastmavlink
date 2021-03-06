//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIFI_CONFIG_AP_H
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_H


//----------------------------------------
//-- Message WIFI_CONFIG_AP
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wifi_config_ap_t {
    char ssid[32];
    char password[64];
    int8_t mode;
    int8_t response;
}) fmav_wifi_config_ap_t;


#define FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP  299

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX  98
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA  19

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FLAGS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FRAME_LEN_MAX  123

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_NUM  64 // number of elements in array
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN  64 // length of array = number of bytes

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_OFS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_OFS  32
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_MODE_OFS  96
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_RESPONSE_OFS  97


//----------------------------------------
//-- Message WIFI_CONFIG_AP packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t* _payload = (fmav_wifi_config_ap_t*)msg->payload;

    _payload->mode = mode;
    _payload->response = response;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload->password), password, sizeof(char)*64);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_config_ap_pack(
        msg, sysid, compid,
        _payload->ssid, _payload->password, _payload->mode, _payload->response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t* _payload = (fmav_wifi_config_ap_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mode = mode;
    _payload->response = response;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload->password), password, sizeof(char)*64);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_config_ap_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ssid, _payload->password, _payload->mode, _payload->response,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response,
    fmav_status_t* _status)
{
    fmav_wifi_config_ap_t _payload;

    _payload.mode = mode;
    _payload.response = response;
    memcpy(&(_payload.ssid), ssid, sizeof(char)*32);
    memcpy(&(_payload.password), password, sizeof(char)*64);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_config_ap_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_config_ap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WIFI_CONFIG_AP,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WIFI_CONFIG_AP unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_wifi_config_ap_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_wifi_config_ap_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wifi_config_ap_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wifi_config_ap_decode(fmav_wifi_config_ap_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_wifi_config_ap_get_field_mode(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[96]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_wifi_config_ap_get_field_response(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[97]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_wifi_config_ap_get_field_ssid_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_wifi_config_ap_get_field_ssid(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_NUM) return 0;
    return ((char*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_wifi_config_ap_get_field_password_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_wifi_config_ap_get_field_password(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_NUM) return 0;
    return ((char*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP  299

#define mavlink_wifi_config_ap_t  fmav_wifi_config_ap_t

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_LEN  98
#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_MIN_LEN  96
#define MAVLINK_MSG_ID_299_LEN  98
#define MAVLINK_MSG_ID_299_MIN_LEN  96

#define MAVLINK_MSG_ID_WIFI_CONFIG_AP_CRC  19
#define MAVLINK_MSG_ID_299_CRC  19

#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN 32
#define MAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN 64


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_config_ap_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const char* ssid, const char* password, int8_t mode, int8_t response)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wifi_config_ap_pack(
        msg, sysid, compid,
        ssid, password, mode, response,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_config_ap_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, const char* password, int8_t mode, int8_t response)
{
    return fmav_msg_wifi_config_ap_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ssid, password, mode, response,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wifi_config_ap_decode(const mavlink_message_t* msg, mavlink_wifi_config_ap_t* payload)
{
    fmav_msg_wifi_config_ap_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIFI_CONFIG_AP_H

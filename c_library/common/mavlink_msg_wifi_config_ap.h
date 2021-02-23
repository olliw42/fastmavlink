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


#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MIN  96
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX  98
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN  98
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_CRCEXTRA  19

#define FASTMAVLINK_MSG_ID_299_LEN_MIN  96
#define FASTMAVLINK_MSG_ID_299_LEN_MAX  98
#define FASTMAVLINK_MSG_ID_299_LEN  98
#define FASTMAVLINK_MSG_ID_299_CRCEXTRA  19

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_SSID_LEN  32
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FIELD_PASSWORD_LEN  64

#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_FLAGS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIFI_CONFIG_AP_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message WIFI_CONFIG_AP packing routines
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
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message WIFI_CONFIG_AP unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wifi_config_ap_decode(fmav_wifi_config_ap_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_WIFI_CONFIG_AP_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

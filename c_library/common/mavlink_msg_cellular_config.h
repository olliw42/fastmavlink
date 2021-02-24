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


#define FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MIN  84
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX  84
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN  84
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_CRCEXTRA  245

#define FASTMAVLINK_MSG_ID_336_LEN_MIN  84
#define FASTMAVLINK_MSG_ID_336_LEN_MAX  84
#define FASTMAVLINK_MSG_ID_336_LEN  84
#define FASTMAVLINK_MSG_ID_336_CRCEXTRA  245

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PIN_LEN  16
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_NEW_PIN_LEN  16
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_APN_LEN  32
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FIELD_PUK_LEN  16

#define FASTMAVLINK_MSG_CELLULAR_CONFIG_FLAGS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_CONFIG_TARGET_COMPONENT_OFS  0


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
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message CELLULAR_CONFIG unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_config_decode(fmav_cellular_config_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CELLULAR_CONFIG_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

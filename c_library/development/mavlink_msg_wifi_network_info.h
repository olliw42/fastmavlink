//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIFI_NETWORK_INFO_H
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_H


//----------------------------------------
//-- Message WIFI_NETWORK_INFO
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wifi_network_info_t {
    uint16_t data_rate;
    char ssid[32];
    uint8_t channel_id;
    uint8_t signal_quality;
    uint8_t security;
}) fmav_wifi_network_info_t;


#define FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO  298

#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_CRCEXTRA  237

#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FLAGS  0
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FRAME_LEN_MAX  62

#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SSID_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SSID_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_DATA_RATE_OFS  0
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SSID_OFS  2
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_CHANNEL_ID_OFS  34
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SIGNAL_QUALITY_OFS  35
#define FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SECURITY_OFS  36


//----------------------------------------
//-- Message WIFI_NETWORK_INFO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, uint8_t channel_id, uint8_t signal_quality, uint16_t data_rate, uint8_t security,
    fmav_status_t* _status)
{
    fmav_wifi_network_info_t* _payload = (fmav_wifi_network_info_t*)_msg->payload;

    _payload->data_rate = data_rate;
    _payload->channel_id = channel_id;
    _payload->signal_quality = signal_quality;
    _payload->security = security;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_WIFI_NETWORK_INFO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_network_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_network_info_pack(
        _msg, sysid, compid,
        _payload->ssid, _payload->channel_id, _payload->signal_quality, _payload->data_rate, _payload->security,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, uint8_t channel_id, uint8_t signal_quality, uint16_t data_rate, uint8_t security,
    fmav_status_t* _status)
{
    fmav_wifi_network_info_t* _payload = (fmav_wifi_network_info_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->data_rate = data_rate;
    _payload->channel_id = channel_id;
    _payload->signal_quality = signal_quality;
    _payload->security = security;
    memcpy(&(_payload->ssid), ssid, sizeof(char)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_network_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wifi_network_info_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->ssid, _payload->channel_id, _payload->signal_quality, _payload->data_rate, _payload->security,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, uint8_t channel_id, uint8_t signal_quality, uint16_t data_rate, uint8_t security,
    fmav_status_t* _status)
{
    fmav_wifi_network_info_t _payload;

    _payload.data_rate = data_rate;
    _payload.channel_id = channel_id;
    _payload.signal_quality = signal_quality;
    _payload.security = security;
    memcpy(&(_payload.ssid), ssid, sizeof(char)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_wifi_network_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WIFI_NETWORK_INFO,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIFI_NETWORK_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WIFI_NETWORK_INFO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wifi_network_info_decode(fmav_wifi_network_info_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIFI_NETWORK_INFO_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wifi_network_info_get_field_data_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_wifi_network_info_get_field_channel_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_wifi_network_info_get_field_signal_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_wifi_network_info_get_field_security(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_wifi_network_info_get_field_ssid_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_wifi_network_info_get_field_ssid(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SSID_NUM) return 0;
    return ((char*)&(msg->payload[2]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIFI_NETWORK_INFO  298

#define mavlink_wifi_network_info_t  fmav_wifi_network_info_t

#define MAVLINK_MSG_ID_WIFI_NETWORK_INFO_LEN  37
#define MAVLINK_MSG_ID_WIFI_NETWORK_INFO_MIN_LEN  37
#define MAVLINK_MSG_ID_298_LEN  37
#define MAVLINK_MSG_ID_298_MIN_LEN  37

#define MAVLINK_MSG_ID_WIFI_NETWORK_INFO_CRC  237
#define MAVLINK_MSG_ID_298_CRC  237

#define MAVLINK_MSG_WIFI_NETWORK_INFO_FIELD_SSID_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_network_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* ssid, uint8_t channel_id, uint8_t signal_quality, uint16_t data_rate, uint8_t security)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wifi_network_info_pack(
        _msg, sysid, compid,
        ssid, channel_id, signal_quality, data_rate, security,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_network_info_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_wifi_network_info_t* _payload)
{
    return mavlink_msg_wifi_network_info_pack(
        sysid,
        compid,
        _msg,
        _payload->ssid, _payload->channel_id, _payload->signal_quality, _payload->data_rate, _payload->security);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wifi_network_info_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* ssid, uint8_t channel_id, uint8_t signal_quality, uint16_t data_rate, uint8_t security)
{
    return fmav_msg_wifi_network_info_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        ssid, channel_id, signal_quality, data_rate, security,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wifi_network_info_decode(const mavlink_message_t* msg, mavlink_wifi_network_info_t* payload)
{
    fmav_msg_wifi_network_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIFI_NETWORK_INFO_H

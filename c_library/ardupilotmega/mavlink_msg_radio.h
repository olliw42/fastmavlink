//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RADIO_H
#define FASTMAVLINK_MSG_RADIO_H


//----------------------------------------
//-- Message RADIO
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_radio_t {
    uint16_t rxerrors;
    uint16_t fixed;
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint8_t noise;
    uint8_t remnoise;
}) fmav_radio_t;


#define FASTMAVLINK_MSG_ID_RADIO  166

#define FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_RADIO_CRCEXTRA  21

#define FASTMAVLINK_MSG_RADIO_FLAGS  0
#define FASTMAVLINK_MSG_RADIO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RADIO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RADIO_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_RADIO_FIELD_RXERRORS_OFS  0
#define FASTMAVLINK_MSG_RADIO_FIELD_FIXED_OFS  2
#define FASTMAVLINK_MSG_RADIO_FIELD_RSSI_OFS  4
#define FASTMAVLINK_MSG_RADIO_FIELD_REMRSSI_OFS  5
#define FASTMAVLINK_MSG_RADIO_FIELD_TXBUF_OFS  6
#define FASTMAVLINK_MSG_RADIO_FIELD_NOISE_OFS  7
#define FASTMAVLINK_MSG_RADIO_FIELD_REMNOISE_OFS  8


//----------------------------------------
//-- Message RADIO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
    fmav_status_t* _status)
{
    fmav_radio_t* _payload = (fmav_radio_t*)_msg->payload;

    _payload->rxerrors = rxerrors;
    _payload->fixed = fixed;
    _payload->rssi = rssi;
    _payload->remrssi = remrssi;
    _payload->txbuf = txbuf;
    _payload->noise = noise;
    _payload->remnoise = remnoise;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RADIO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RADIO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_pack(
        _msg, sysid, compid,
        _payload->rssi, _payload->remrssi, _payload->txbuf, _payload->noise, _payload->remnoise, _payload->rxerrors, _payload->fixed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
    fmav_status_t* _status)
{
    fmav_radio_t* _payload = (fmav_radio_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rxerrors = rxerrors;
    _payload->fixed = fixed;
    _payload->rssi = rssi;
    _payload->remrssi = remrssi;
    _payload->txbuf = txbuf;
    _payload->noise = noise;
    _payload->remnoise = remnoise;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->rssi, _payload->remrssi, _payload->txbuf, _payload->noise, _payload->remnoise, _payload->rxerrors, _payload->fixed,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
    fmav_status_t* _status)
{
    fmav_radio_t _payload;

    _payload.rxerrors = rxerrors;
    _payload.fixed = fixed;
    _payload.rssi = rssi;
    _payload.remrssi = remrssi;
    _payload.txbuf = txbuf;
    _payload.noise = noise;
    _payload.remnoise = remnoise;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RADIO,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RADIO,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RADIO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_decode(fmav_radio_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_get_field_rxerrors(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_get_field_fixed(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_get_field_remrssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_get_field_txbuf(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_get_field_noise(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_get_field_remnoise(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RADIO  166

#define mavlink_radio_t  fmav_radio_t

#define MAVLINK_MSG_ID_RADIO_LEN  9
#define MAVLINK_MSG_ID_RADIO_MIN_LEN  9
#define MAVLINK_MSG_ID_166_LEN  9
#define MAVLINK_MSG_ID_166_MIN_LEN  9

#define MAVLINK_MSG_ID_RADIO_CRC  21
#define MAVLINK_MSG_ID_166_CRC  21




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_pack(
        _msg, sysid, compid,
        rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_radio_t* _payload)
{
    return mavlink_msg_radio_pack(
        sysid,
        compid,
        _msg,
        _payload->rssi, _payload->remrssi, _payload->txbuf, _payload->noise, _payload->remnoise, _payload->rxerrors, _payload->fixed);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
    return fmav_msg_radio_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_radio_decode(const mavlink_message_t* msg, mavlink_radio_t* payload)
{
    fmav_msg_radio_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RADIO_H

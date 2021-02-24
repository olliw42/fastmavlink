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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MIN  9
#define FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN  9
#define FASTMAVLINK_MSG_RADIO_CRCEXTRA  21

#define FASTMAVLINK_MSG_ID_166_LEN_MIN  9
#define FASTMAVLINK_MSG_ID_166_LEN_MAX  9
#define FASTMAVLINK_MSG_ID_166_LEN  9
#define FASTMAVLINK_MSG_ID_166_CRCEXTRA  21



#define FASTMAVLINK_MSG_RADIO_FLAGS  0
#define FASTMAVLINK_MSG_RADIO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RADIO_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RADIO packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
    fmav_status_t* _status)
{
    fmav_radio_t* _payload = (fmav_radio_t*)msg->payload;

    _payload->rxerrors = rxerrors;
    _payload->fixed = fixed;
    _payload->rssi = rssi;
    _payload->remrssi = remrssi;
    _payload->txbuf = txbuf;
    _payload->noise = noise;
    _payload->remnoise = remnoise;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RADIO;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RADIO_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_pack(
        msg, sysid, compid,
        _payload->rssi, _payload->remrssi, _payload->txbuf, _payload->noise, _payload->remnoise, _payload->rxerrors, _payload->fixed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed,
    fmav_status_t* _status)
{
    fmav_radio_t* _payload = (fmav_radio_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rxerrors = rxerrors;
    _payload->fixed = fixed;
    _payload->rssi = rssi;
    _payload->remrssi = remrssi;
    _payload->txbuf = txbuf;
    _payload->noise = noise;
    _payload->remnoise = remnoise;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->rssi, _payload->remrssi, _payload->txbuf, _payload->noise, _payload->remnoise, _payload->rxerrors, _payload->fixed,
        _status);
}


//----------------------------------------
//-- Message RADIO unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_decode(fmav_radio_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RADIO_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_pack(
        msg, sysid, compid,
        rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed)
{
    return fmav_msg_radio_pack_to_frame_buf(
        (uint8_t*)buf,
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

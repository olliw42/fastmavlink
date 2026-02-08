//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_H
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_H


//----------------------------------------
//-- Message MLRS_RADIO_LINK_FLOW_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mlrs_radio_link_flow_control_t {
    uint16_t tx_ser_rate;
    uint16_t rx_ser_rate;
    uint8_t tx_used_ser_bandwidth;
    uint8_t rx_used_ser_bandwidth;
    uint8_t txbuf;
}) fmav_mlrs_radio_link_flow_control_t;


#define FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL  60047

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_CRCEXTRA  55

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FLAGS  0
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FRAME_LEN_MAX  32



#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FIELD_TX_SER_RATE_OFS  0
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FIELD_RX_SER_RATE_OFS  2
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FIELD_TX_USED_SER_BANDWIDTH_OFS  4
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FIELD_RX_USED_SER_BANDWIDTH_OFS  5
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_FIELD_TXBUF_OFS  6


//----------------------------------------
//-- Message MLRS_RADIO_LINK_FLOW_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t tx_ser_rate, uint16_t rx_ser_rate, uint8_t tx_used_ser_bandwidth, uint8_t rx_used_ser_bandwidth, uint8_t txbuf,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_flow_control_t* _payload = (fmav_mlrs_radio_link_flow_control_t*)_msg->payload;

    _payload->tx_ser_rate = tx_ser_rate;
    _payload->rx_ser_rate = rx_ser_rate;
    _payload->tx_used_ser_bandwidth = tx_used_ser_bandwidth;
    _payload->rx_used_ser_bandwidth = rx_used_ser_bandwidth;
    _payload->txbuf = txbuf;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_flow_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mlrs_radio_link_flow_control_pack(
        _msg, sysid, compid,
        _payload->tx_ser_rate, _payload->rx_ser_rate, _payload->tx_used_ser_bandwidth, _payload->rx_used_ser_bandwidth, _payload->txbuf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t tx_ser_rate, uint16_t rx_ser_rate, uint8_t tx_used_ser_bandwidth, uint8_t rx_used_ser_bandwidth, uint8_t txbuf,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_flow_control_t* _payload = (fmav_mlrs_radio_link_flow_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->tx_ser_rate = tx_ser_rate;
    _payload->rx_ser_rate = rx_ser_rate;
    _payload->tx_used_ser_bandwidth = tx_used_ser_bandwidth;
    _payload->rx_used_ser_bandwidth = rx_used_ser_bandwidth;
    _payload->txbuf = txbuf;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_flow_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mlrs_radio_link_flow_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->tx_ser_rate, _payload->rx_ser_rate, _payload->tx_used_ser_bandwidth, _payload->rx_used_ser_bandwidth, _payload->txbuf,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t tx_ser_rate, uint16_t rx_ser_rate, uint8_t tx_used_ser_bandwidth, uint8_t rx_used_ser_bandwidth, uint8_t txbuf,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_flow_control_t _payload;

    _payload.tx_ser_rate = tx_ser_rate;
    _payload.rx_ser_rate = rx_ser_rate;
    _payload.tx_used_ser_bandwidth = tx_used_ser_bandwidth;
    _payload.rx_used_ser_bandwidth = rx_used_ser_bandwidth;
    _payload.txbuf = txbuf;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_flow_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MLRS_RADIO_LINK_FLOW_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mlrs_radio_link_flow_control_decode(fmav_mlrs_radio_link_flow_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_get_field_tx_ser_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_flow_control_get_field_rx_ser_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_flow_control_get_field_tx_used_ser_bandwidth(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_flow_control_get_field_rx_used_ser_bandwidth(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_flow_control_get_field_txbuf(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL  60047

#define mavlink_mlrs_radio_link_flow_control_t  fmav_mlrs_radio_link_flow_control_t

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL_LEN  7
#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL_MIN_LEN  7
#define MAVLINK_MSG_ID_60047_LEN  7
#define MAVLINK_MSG_ID_60047_MIN_LEN  7

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_FLOW_CONTROL_CRC  55
#define MAVLINK_MSG_ID_60047_CRC  55




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_flow_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t tx_ser_rate, uint16_t rx_ser_rate, uint8_t tx_used_ser_bandwidth, uint8_t rx_used_ser_bandwidth, uint8_t txbuf)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mlrs_radio_link_flow_control_pack(
        _msg, sysid, compid,
        tx_ser_rate, rx_ser_rate, tx_used_ser_bandwidth, rx_used_ser_bandwidth, txbuf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_flow_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mlrs_radio_link_flow_control_t* _payload)
{
    return mavlink_msg_mlrs_radio_link_flow_control_pack(
        sysid,
        compid,
        _msg,
        _payload->tx_ser_rate, _payload->rx_ser_rate, _payload->tx_used_ser_bandwidth, _payload->rx_used_ser_bandwidth, _payload->txbuf);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_flow_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t tx_ser_rate, uint16_t rx_ser_rate, uint8_t tx_used_ser_bandwidth, uint8_t rx_used_ser_bandwidth, uint8_t txbuf)
{
    return fmav_msg_mlrs_radio_link_flow_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        tx_ser_rate, rx_ser_rate, tx_used_ser_bandwidth, rx_used_ser_bandwidth, txbuf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mlrs_radio_link_flow_control_decode(const mavlink_message_t* msg, mavlink_mlrs_radio_link_flow_control_t* payload)
{
    fmav_msg_mlrs_radio_link_flow_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MLRS_RADIO_LINK_FLOW_CONTROL_H

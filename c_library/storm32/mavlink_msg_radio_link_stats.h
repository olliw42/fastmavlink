//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RADIO_LINK_STATS_H
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_H


//----------------------------------------
//-- Message RADIO_LINK_STATS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_radio_link_stats_t {
    uint8_t flags;
    uint8_t rx_LQ;
    uint8_t rx_rssi1;
    int8_t rx_snr1;
    uint8_t rx_rssi2;
    int8_t rx_snr2;
    uint8_t rx_receive_antenna;
    uint8_t rx_transmit_antenna;
    uint8_t tx_LQ;
    uint8_t tx_rssi1;
    int8_t tx_snr1;
    uint8_t tx_rssi2;
    int8_t tx_snr2;
    uint8_t tx_receive_antenna;
    uint8_t tx_transmit_antenna;
}) fmav_radio_link_stats_t;


#define FASTMAVLINK_MSG_ID_RADIO_LINK_STATS  60046

#define FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX  15
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA  238

#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FLAGS  0
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FRAME_LEN_MAX  40



#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_FLAGS_OFS  0
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_LQ_OFS  1
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_RSSI1_OFS  2
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_SNR1_OFS  3
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_RSSI2_OFS  4
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_SNR2_OFS  5
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_RECEIVE_ANTENNA_OFS  6
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_RX_TRANSMIT_ANTENNA_OFS  7
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_LQ_OFS  8
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_RSSI1_OFS  9
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_SNR1_OFS  10
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_RSSI2_OFS  11
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_SNR2_OFS  12
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_RECEIVE_ANTENNA_OFS  13
#define FASTMAVLINK_MSG_RADIO_LINK_STATS_FIELD_TX_TRANSMIT_ANTENNA_OFS  14


//----------------------------------------
//-- Message RADIO_LINK_STATS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t flags, uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2, uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna, uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2, uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna,
    fmav_status_t* _status)
{
    fmav_radio_link_stats_t* _payload = (fmav_radio_link_stats_t*)_msg->payload;

    _payload->flags = flags;
    _payload->rx_LQ = rx_LQ;
    _payload->rx_rssi1 = rx_rssi1;
    _payload->rx_snr1 = rx_snr1;
    _payload->rx_rssi2 = rx_rssi2;
    _payload->rx_snr2 = rx_snr2;
    _payload->rx_receive_antenna = rx_receive_antenna;
    _payload->rx_transmit_antenna = rx_transmit_antenna;
    _payload->tx_LQ = tx_LQ;
    _payload->tx_rssi1 = tx_rssi1;
    _payload->tx_snr1 = tx_snr1;
    _payload->tx_rssi2 = tx_rssi2;
    _payload->tx_snr2 = tx_snr2;
    _payload->tx_receive_antenna = tx_receive_antenna;
    _payload->tx_transmit_antenna = tx_transmit_antenna;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RADIO_LINK_STATS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_stats_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_link_stats_pack(
        _msg, sysid, compid,
        _payload->flags, _payload->rx_LQ, _payload->rx_rssi1, _payload->rx_snr1, _payload->rx_rssi2, _payload->rx_snr2, _payload->rx_receive_antenna, _payload->rx_transmit_antenna, _payload->tx_LQ, _payload->tx_rssi1, _payload->tx_snr1, _payload->tx_rssi2, _payload->tx_snr2, _payload->tx_receive_antenna, _payload->tx_transmit_antenna,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t flags, uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2, uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna, uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2, uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna,
    fmav_status_t* _status)
{
    fmav_radio_link_stats_t* _payload = (fmav_radio_link_stats_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->flags = flags;
    _payload->rx_LQ = rx_LQ;
    _payload->rx_rssi1 = rx_rssi1;
    _payload->rx_snr1 = rx_snr1;
    _payload->rx_rssi2 = rx_rssi2;
    _payload->rx_snr2 = rx_snr2;
    _payload->rx_receive_antenna = rx_receive_antenna;
    _payload->rx_transmit_antenna = rx_transmit_antenna;
    _payload->tx_LQ = tx_LQ;
    _payload->tx_rssi1 = tx_rssi1;
    _payload->tx_snr1 = tx_snr1;
    _payload->tx_rssi2 = tx_rssi2;
    _payload->tx_snr2 = tx_snr2;
    _payload->tx_receive_antenna = tx_receive_antenna;
    _payload->tx_transmit_antenna = tx_transmit_antenna;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO_LINK_STATS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_LINK_STATS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_LINK_STATS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_stats_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_link_stats_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->flags, _payload->rx_LQ, _payload->rx_rssi1, _payload->rx_snr1, _payload->rx_rssi2, _payload->rx_snr2, _payload->rx_receive_antenna, _payload->rx_transmit_antenna, _payload->tx_LQ, _payload->tx_rssi1, _payload->tx_snr1, _payload->tx_rssi2, _payload->tx_snr2, _payload->tx_receive_antenna, _payload->tx_transmit_antenna,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t flags, uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2, uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna, uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2, uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna,
    fmav_status_t* _status)
{
    fmav_radio_link_stats_t _payload;

    _payload.flags = flags;
    _payload.rx_LQ = rx_LQ;
    _payload.rx_rssi1 = rx_rssi1;
    _payload.rx_snr1 = rx_snr1;
    _payload.rx_rssi2 = rx_rssi2;
    _payload.rx_snr2 = rx_snr2;
    _payload.rx_receive_antenna = rx_receive_antenna;
    _payload.rx_transmit_antenna = rx_transmit_antenna;
    _payload.tx_LQ = tx_LQ;
    _payload.tx_rssi1 = tx_rssi1;
    _payload.tx_snr1 = tx_snr1;
    _payload.tx_rssi2 = tx_rssi2;
    _payload.tx_snr2 = tx_snr2;
    _payload.tx_receive_antenna = tx_receive_antenna;
    _payload.tx_transmit_antenna = tx_transmit_antenna;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RADIO_LINK_STATS,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_link_stats_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_link_stats_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RADIO_LINK_STATS,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_LINK_STATS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RADIO_LINK_STATS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_link_stats_decode(fmav_radio_link_stats_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_LINK_STATS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_rx_LQ(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_rx_rssi1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_radio_link_stats_get_field_rx_snr1(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_rx_rssi2(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_radio_link_stats_get_field_rx_snr2(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_rx_receive_antenna(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_rx_transmit_antenna(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_tx_LQ(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_tx_rssi1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_radio_link_stats_get_field_tx_snr1(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_tx_rssi2(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_radio_link_stats_get_field_tx_snr2(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_tx_receive_antenna(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_radio_link_stats_get_field_tx_transmit_antenna(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RADIO_LINK_STATS  60046

#define mavlink_radio_link_stats_t  fmav_radio_link_stats_t

#define MAVLINK_MSG_ID_RADIO_LINK_STATS_LEN  15
#define MAVLINK_MSG_ID_RADIO_LINK_STATS_MIN_LEN  15
#define MAVLINK_MSG_ID_60046_LEN  15
#define MAVLINK_MSG_ID_60046_MIN_LEN  15

#define MAVLINK_MSG_ID_RADIO_LINK_STATS_CRC  238
#define MAVLINK_MSG_ID_60046_CRC  238




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_stats_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t flags, uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2, uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna, uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2, uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_link_stats_pack(
        _msg, sysid, compid,
        flags, rx_LQ, rx_rssi1, rx_snr1, rx_rssi2, rx_snr2, rx_receive_antenna, rx_transmit_antenna, tx_LQ, tx_rssi1, tx_snr1, tx_rssi2, tx_snr2, tx_receive_antenna, tx_transmit_antenna,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_stats_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_radio_link_stats_t* _payload)
{
    return mavlink_msg_radio_link_stats_pack(
        sysid,
        compid,
        _msg,
        _payload->flags, _payload->rx_LQ, _payload->rx_rssi1, _payload->rx_snr1, _payload->rx_rssi2, _payload->rx_snr2, _payload->rx_receive_antenna, _payload->rx_transmit_antenna, _payload->tx_LQ, _payload->tx_rssi1, _payload->tx_snr1, _payload->tx_rssi2, _payload->tx_snr2, _payload->tx_receive_antenna, _payload->tx_transmit_antenna);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_link_stats_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t flags, uint8_t rx_LQ, uint8_t rx_rssi1, int8_t rx_snr1, uint8_t rx_rssi2, int8_t rx_snr2, uint8_t rx_receive_antenna, uint8_t rx_transmit_antenna, uint8_t tx_LQ, uint8_t tx_rssi1, int8_t tx_snr1, uint8_t tx_rssi2, int8_t tx_snr2, uint8_t tx_receive_antenna, uint8_t tx_transmit_antenna)
{
    return fmav_msg_radio_link_stats_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        flags, rx_LQ, rx_rssi1, rx_snr1, rx_rssi2, rx_snr2, rx_receive_antenna, rx_transmit_antenna, tx_LQ, tx_rssi1, tx_snr1, tx_rssi2, tx_snr2, tx_receive_antenna, tx_transmit_antenna,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_radio_link_stats_decode(const mavlink_message_t* msg, mavlink_radio_link_stats_t* payload)
{
    fmav_msg_radio_link_stats_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RADIO_LINK_STATS_H

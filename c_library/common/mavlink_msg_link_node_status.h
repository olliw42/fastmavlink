//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LINK_NODE_STATUS_H
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_H


//----------------------------------------
//-- Message LINK_NODE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_link_node_status_t {
    uint64_t timestamp;
    uint32_t tx_rate;
    uint32_t rx_rate;
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t messages_lost;
    uint16_t rx_parse_err;
    uint16_t tx_overflows;
    uint16_t rx_overflows;
    uint8_t tx_buf;
    uint8_t rx_buf;
}) fmav_link_node_status_t;


#define FASTMAVLINK_MSG_ID_LINK_NODE_STATUS  8

#define FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX  36
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA  117

#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FRAME_LEN_MAX  61



#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_TX_RATE_OFS  8
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_RX_RATE_OFS  12
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_MESSAGES_SENT_OFS  16
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_MESSAGES_RECEIVED_OFS  20
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_MESSAGES_LOST_OFS  24
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_RX_PARSE_ERR_OFS  28
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_TX_OVERFLOWS_OFS  30
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_RX_OVERFLOWS_OFS  32
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_TX_BUF_OFS  34
#define FASTMAVLINK_MSG_LINK_NODE_STATUS_FIELD_RX_BUF_OFS  35


//----------------------------------------
//-- Message LINK_NODE_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost,
    fmav_status_t* _status)
{
    fmav_link_node_status_t* _payload = (fmav_link_node_status_t*)msg->payload;

    _payload->timestamp = timestamp;
    _payload->tx_rate = tx_rate;
    _payload->rx_rate = rx_rate;
    _payload->messages_sent = messages_sent;
    _payload->messages_received = messages_received;
    _payload->messages_lost = messages_lost;
    _payload->rx_parse_err = rx_parse_err;
    _payload->tx_overflows = tx_overflows;
    _payload->rx_overflows = rx_overflows;
    _payload->tx_buf = tx_buf;
    _payload->rx_buf = rx_buf;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LINK_NODE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_link_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_link_node_status_pack(
        msg, sysid, compid,
        _payload->timestamp, _payload->tx_buf, _payload->rx_buf, _payload->tx_rate, _payload->rx_rate, _payload->rx_parse_err, _payload->tx_overflows, _payload->rx_overflows, _payload->messages_sent, _payload->messages_received, _payload->messages_lost,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost,
    fmav_status_t* _status)
{
    fmav_link_node_status_t* _payload = (fmav_link_node_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->tx_rate = tx_rate;
    _payload->rx_rate = rx_rate;
    _payload->messages_sent = messages_sent;
    _payload->messages_received = messages_received;
    _payload->messages_lost = messages_lost;
    _payload->rx_parse_err = rx_parse_err;
    _payload->tx_overflows = tx_overflows;
    _payload->rx_overflows = rx_overflows;
    _payload->tx_buf = tx_buf;
    _payload->rx_buf = rx_buf;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LINK_NODE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LINK_NODE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LINK_NODE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_link_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_link_node_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->timestamp, _payload->tx_buf, _payload->rx_buf, _payload->tx_rate, _payload->rx_rate, _payload->rx_parse_err, _payload->tx_overflows, _payload->rx_overflows, _payload->messages_sent, _payload->messages_received, _payload->messages_lost,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost,
    fmav_status_t* _status)
{
    fmav_link_node_status_t _payload;

    _payload.timestamp = timestamp;
    _payload.tx_rate = tx_rate;
    _payload.rx_rate = rx_rate;
    _payload.messages_sent = messages_sent;
    _payload.messages_received = messages_received;
    _payload.messages_lost = messages_lost;
    _payload.rx_parse_err = rx_parse_err;
    _payload.tx_overflows = tx_overflows;
    _payload.rx_overflows = rx_overflows;
    _payload.tx_buf = tx_buf;
    _payload.rx_buf = rx_buf;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LINK_NODE_STATUS,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_link_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LINK_NODE_STATUS,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LINK_NODE_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LINK_NODE_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_link_node_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_link_node_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_link_node_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_link_node_status_decode(fmav_link_node_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LINK_NODE_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_link_node_status_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_link_node_status_get_field_tx_rate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_link_node_status_get_field_rx_rate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_link_node_status_get_field_messages_sent(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_link_node_status_get_field_messages_received(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_link_node_status_get_field_messages_lost(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_get_field_rx_parse_err(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_get_field_tx_overflows(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_link_node_status_get_field_rx_overflows(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_link_node_status_get_field_tx_buf(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_link_node_status_get_field_rx_buf(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LINK_NODE_STATUS  8

#define mavlink_link_node_status_t  fmav_link_node_status_t

#define MAVLINK_MSG_ID_LINK_NODE_STATUS_LEN  36
#define MAVLINK_MSG_ID_LINK_NODE_STATUS_MIN_LEN  36
#define MAVLINK_MSG_ID_8_LEN  36
#define MAVLINK_MSG_ID_8_MIN_LEN  36

#define MAVLINK_MSG_ID_LINK_NODE_STATUS_CRC  117
#define MAVLINK_MSG_ID_8_CRC  117




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_link_node_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_link_node_status_pack(
        msg, sysid, compid,
        timestamp, tx_buf, rx_buf, tx_rate, rx_rate, rx_parse_err, tx_overflows, rx_overflows, messages_sent, messages_received, messages_lost,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_link_node_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t tx_buf, uint8_t rx_buf, uint32_t tx_rate, uint32_t rx_rate, uint16_t rx_parse_err, uint16_t tx_overflows, uint16_t rx_overflows, uint32_t messages_sent, uint32_t messages_received, uint32_t messages_lost)
{
    return fmav_msg_link_node_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        timestamp, tx_buf, rx_buf, tx_rate, rx_rate, rx_parse_err, tx_overflows, rx_overflows, messages_sent, messages_received, messages_lost,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_link_node_status_decode(const mavlink_message_t* msg, mavlink_link_node_status_t* payload)
{
    fmav_msg_link_node_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LINK_NODE_STATUS_H

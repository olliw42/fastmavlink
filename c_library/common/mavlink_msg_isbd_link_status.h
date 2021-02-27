//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ISBD_LINK_STATUS_H
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_H


//----------------------------------------
//-- Message ISBD_LINK_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_isbd_link_status_t {
    uint64_t timestamp;
    uint64_t last_heartbeat;
    uint16_t failed_sessions;
    uint16_t successful_sessions;
    uint8_t signal_quality;
    uint8_t ring_pending;
    uint8_t tx_session_pending;
    uint8_t rx_session_pending;
}) fmav_isbd_link_status_t;


#define FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS  335


#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MIN  24
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN  24
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA  225

#define FASTMAVLINK_MSG_ID_335_LEN_MIN  24
#define FASTMAVLINK_MSG_ID_335_LEN_MAX  24
#define FASTMAVLINK_MSG_ID_335_LEN  24
#define FASTMAVLINK_MSG_ID_335_CRCEXTRA  225



#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ISBD_LINK_STATUS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_335_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_335_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message ISBD_LINK_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t* _payload = (fmav_isbd_link_status_t*)msg->payload;

    _payload->timestamp = timestamp;
    _payload->last_heartbeat = last_heartbeat;
    _payload->failed_sessions = failed_sessions;
    _payload->successful_sessions = successful_sessions;
    _payload->signal_quality = signal_quality;
    _payload->ring_pending = ring_pending;
    _payload->tx_session_pending = tx_session_pending;
    _payload->rx_session_pending = rx_session_pending;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_isbd_link_status_pack(
        msg, sysid, compid,
        _payload->timestamp, _payload->last_heartbeat, _payload->failed_sessions, _payload->successful_sessions, _payload->signal_quality, _payload->ring_pending, _payload->tx_session_pending, _payload->rx_session_pending,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t* _payload = (fmav_isbd_link_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->last_heartbeat = last_heartbeat;
    _payload->failed_sessions = failed_sessions;
    _payload->successful_sessions = successful_sessions;
    _payload->signal_quality = signal_quality;
    _payload->ring_pending = ring_pending;
    _payload->tx_session_pending = tx_session_pending;
    _payload->rx_session_pending = rx_session_pending;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_isbd_link_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->timestamp, _payload->last_heartbeat, _payload->failed_sessions, _payload->successful_sessions, _payload->signal_quality, _payload->ring_pending, _payload->tx_session_pending, _payload->rx_session_pending,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending,
    fmav_status_t* _status)
{
    fmav_isbd_link_status_t _payload;

    _payload.timestamp = timestamp;
    _payload.last_heartbeat = last_heartbeat;
    _payload.failed_sessions = failed_sessions;
    _payload.successful_sessions = successful_sessions;
    _payload.signal_quality = signal_quality;
    _payload.ring_pending = ring_pending;
    _payload.tx_session_pending = tx_session_pending;
    _payload.rx_session_pending = rx_session_pending;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_isbd_link_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_isbd_link_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ISBD_LINK_STATUS,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ISBD_LINK_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ISBD_LINK_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_isbd_link_status_decode(fmav_isbd_link_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ISBD_LINK_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS  335

#define mavlink_isbd_link_status_t  fmav_isbd_link_status_t

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_LEN  24
#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_MIN_LEN  24
#define MAVLINK_MSG_ID_335_LEN  24
#define MAVLINK_MSG_ID_335_MIN_LEN  24

#define MAVLINK_MSG_ID_ISBD_LINK_STATUS_CRC  225
#define MAVLINK_MSG_ID_335_CRC  225




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_isbd_link_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_isbd_link_status_pack(
        msg, sysid, compid,
        timestamp, last_heartbeat, failed_sessions, successful_sessions, signal_quality, ring_pending, tx_session_pending, rx_session_pending,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_isbd_link_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t last_heartbeat, uint16_t failed_sessions, uint16_t successful_sessions, uint8_t signal_quality, uint8_t ring_pending, uint8_t tx_session_pending, uint8_t rx_session_pending)
{
    return fmav_msg_isbd_link_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        timestamp, last_heartbeat, failed_sessions, successful_sessions, signal_quality, ring_pending, tx_session_pending, rx_session_pending,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_isbd_link_status_decode(const mavlink_message_t* msg, mavlink_isbd_link_status_t* payload)
{
    fmav_msg_isbd_link_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ISBD_LINK_STATUS_H

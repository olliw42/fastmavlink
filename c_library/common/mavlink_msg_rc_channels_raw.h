//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RC_CHANNELS_RAW_H
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_H


//----------------------------------------
//-- Message RC_CHANNELS_RAW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rc_channels_raw_t {
    uint32_t time_boot_ms;
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint8_t port;
    uint8_t rssi;
}) fmav_rc_channels_raw_t;


#define FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW  35


#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MIN  22
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA  244

#define FASTMAVLINK_MSG_ID_35_LEN_MIN  22
#define FASTMAVLINK_MSG_ID_35_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_35_LEN  22
#define FASTMAVLINK_MSG_ID_35_CRCEXTRA  244



#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FLAGS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RC_CHANNELS_RAW packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_raw_t* _payload = (fmav_rc_channels_raw_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->port = port;
    _payload->rssi = rssi;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_raw_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->port, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_raw_t* _payload = (fmav_rc_channels_raw_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->port = port;
    _payload->rssi = rssi;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_raw_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->port, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->rssi,
        _status);
}


//----------------------------------------
//-- Message RC_CHANNELS_RAW unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_raw_decode(fmav_rc_channels_raw_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW  35

#define mavlink_rc_channels_raw_t  fmav_rc_channels_raw_t

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN  22
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_MIN_LEN  22
#define MAVLINK_MSG_ID_35_LEN  22
#define MAVLINK_MSG_ID_35_MIN_LEN  22

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC  244
#define MAVLINK_MSG_ID_35_CRC  244




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rc_channels_raw_pack(
        msg, sysid, compid,
        time_boot_ms, port, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, rssi,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_raw_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
    return fmav_msg_rc_channels_raw_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, port, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rc_channels_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_raw_t* payload)
{
    fmav_msg_rc_channels_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RC_CHANNELS_RAW_H

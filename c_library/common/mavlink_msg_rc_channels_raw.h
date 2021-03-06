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

#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA  244

#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FLAGS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FRAME_LEN_MAX  47



#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN1_RAW_OFS  4
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN2_RAW_OFS  6
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN3_RAW_OFS  8
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN4_RAW_OFS  10
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN5_RAW_OFS  12
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN6_RAW_OFS  14
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN7_RAW_OFS  16
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_CHAN8_RAW_OFS  18
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_PORT_OFS  20
#define FASTMAVLINK_MSG_RC_CHANNELS_RAW_FIELD_RSSI_OFS  21


//----------------------------------------
//-- Message RC_CHANNELS_RAW packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_rc_channels_raw_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.chan1_raw = chan1_raw;
    _payload.chan2_raw = chan2_raw;
    _payload.chan3_raw = chan3_raw;
    _payload.chan4_raw = chan4_raw;
    _payload.chan5_raw = chan5_raw;
    _payload.chan6_raw = chan6_raw;
    _payload.chan7_raw = chan7_raw;
    _payload.chan8_raw = chan8_raw;
    _payload.port = port;
    _payload.rssi = rssi;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RC_CHANNELS_RAW,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_RAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RC_CHANNELS_RAW unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_rc_channels_raw_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_rc_channels_raw_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_raw_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_raw_decode(fmav_rc_channels_raw_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RC_CHANNELS_RAW_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_rc_channels_raw_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan1_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan2_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan3_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan4_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan5_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan6_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan7_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_raw_get_field_chan8_raw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_raw_get_field_port(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rc_channels_raw_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
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

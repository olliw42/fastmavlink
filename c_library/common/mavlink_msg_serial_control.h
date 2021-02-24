//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SERIAL_CONTROL_H
#define FASTMAVLINK_MSG_SERIAL_CONTROL_H


//----------------------------------------
//-- Message SERIAL_CONTROL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_serial_control_t {
    uint32_t baudrate;
    uint16_t timeout;
    uint8_t device;
    uint8_t flags;
    uint8_t count;
    uint8_t data[70];
}) fmav_serial_control_t;


#define FASTMAVLINK_MSG_ID_SERIAL_CONTROL  126


#define FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MIN  79
#define FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX  79
#define FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN  79
#define FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA  220

#define FASTMAVLINK_MSG_ID_126_LEN_MIN  79
#define FASTMAVLINK_MSG_ID_126_LEN_MAX  79
#define FASTMAVLINK_MSG_ID_126_LEN  79
#define FASTMAVLINK_MSG_ID_126_CRCEXTRA  220

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN  70

#define FASTMAVLINK_MSG_SERIAL_CONTROL_FLAGS  0
#define FASTMAVLINK_MSG_SERIAL_CONTROL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SERIAL_CONTROL_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SERIAL_CONTROL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_serial_control_t* _payload = (fmav_serial_control_t*)msg->payload;

    _payload->baudrate = baudrate;
    _payload->timeout = timeout;
    _payload->device = device;
    _payload->flags = flags;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*70);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SERIAL_CONTROL;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_serial_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_serial_control_pack(
        msg, sysid, compid,
        _payload->device, _payload->flags, _payload->timeout, _payload->baudrate, _payload->count, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_serial_control_t* _payload = (fmav_serial_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->baudrate = baudrate;
    _payload->timeout = timeout;
    _payload->device = device;
    _payload->flags = flags;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*70);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SERIAL_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERIAL_CONTROL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_serial_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_serial_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_serial_control_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->device, _payload->flags, _payload->timeout, _payload->baudrate, _payload->count, _payload->data,
        _status);
}


//----------------------------------------
//-- Message SERIAL_CONTROL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_serial_control_decode(fmav_serial_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SERIAL_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SERIAL_CONTROL  126

#define mavlink_serial_control_t  fmav_serial_control_t

#define MAVLINK_MSG_ID_SERIAL_CONTROL_LEN  79
#define MAVLINK_MSG_ID_SERIAL_CONTROL_MIN_LEN  79
#define MAVLINK_MSG_ID_126_LEN  79
#define MAVLINK_MSG_ID_126_MIN_LEN  79

#define MAVLINK_MSG_ID_SERIAL_CONTROL_CRC  220
#define MAVLINK_MSG_ID_126_CRC  220

#define MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN 70


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_serial_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_serial_control_pack(
        msg, sysid, compid,
        device, flags, timeout, baudrate, count, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_serial_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, uint8_t count, const uint8_t* data)
{
    return fmav_msg_serial_control_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        device, flags, timeout, baudrate, count, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_serial_control_decode(const mavlink_message_t* msg, mavlink_serial_control_t* payload)
{
    fmav_msg_serial_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SERIAL_CONTROL_H

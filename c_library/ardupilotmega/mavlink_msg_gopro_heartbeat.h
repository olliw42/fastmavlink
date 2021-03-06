//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GOPRO_HEARTBEAT_H
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_H


//----------------------------------------
//-- Message GOPRO_HEARTBEAT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gopro_heartbeat_t {
    uint8_t status;
    uint8_t capture_mode;
    uint8_t flags;
}) fmav_gopro_heartbeat_t;


#define FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT  215

#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA  101

#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_FLAGS  0
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_FRAME_LEN_MAX  28



#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_FIELD_STATUS_OFS  0
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_FIELD_CAPTURE_MODE_OFS  1
#define FASTMAVLINK_MSG_GOPRO_HEARTBEAT_FIELD_FLAGS_OFS  2


//----------------------------------------
//-- Message GOPRO_HEARTBEAT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t capture_mode, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_gopro_heartbeat_t* _payload = (fmav_gopro_heartbeat_t*)msg->payload;

    _payload->status = status;
    _payload->capture_mode = capture_mode;
    _payload->flags = flags;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_heartbeat_pack(
        msg, sysid, compid,
        _payload->status, _payload->capture_mode, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t capture_mode, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_gopro_heartbeat_t* _payload = (fmav_gopro_heartbeat_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->status = status;
    _payload->capture_mode = capture_mode;
    _payload->flags = flags;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_heartbeat_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->status, _payload->capture_mode, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t capture_mode, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_gopro_heartbeat_t _payload;

    _payload.status = status;
    _payload.capture_mode = capture_mode;
    _payload.flags = flags;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_heartbeat_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GOPRO_HEARTBEAT,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_HEARTBEAT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GOPRO_HEARTBEAT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gopro_heartbeat_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gopro_heartbeat_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_heartbeat_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_heartbeat_decode(fmav_gopro_heartbeat_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GOPRO_HEARTBEAT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_heartbeat_get_field_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_heartbeat_get_field_capture_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gopro_heartbeat_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT  215

#define mavlink_gopro_heartbeat_t  fmav_gopro_heartbeat_t

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_LEN  3
#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN  3
#define MAVLINK_MSG_ID_215_LEN  3
#define MAVLINK_MSG_ID_215_MIN_LEN  3

#define MAVLINK_MSG_ID_GOPRO_HEARTBEAT_CRC  101
#define MAVLINK_MSG_ID_215_CRC  101




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_heartbeat_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t status, uint8_t capture_mode, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gopro_heartbeat_pack(
        msg, sysid, compid,
        status, capture_mode, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_heartbeat_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t capture_mode, uint8_t flags)
{
    return fmav_msg_gopro_heartbeat_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        status, capture_mode, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gopro_heartbeat_decode(const mavlink_message_t* msg, mavlink_gopro_heartbeat_t* payload)
{
    fmav_msg_gopro_heartbeat_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GOPRO_HEARTBEAT_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_H
#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_H


//----------------------------------------
//-- Message ICAROUS_HEARTBEAT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_icarous_heartbeat_t {
    uint8_t status;
}) fmav_icarous_heartbeat_t;


#define FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT  42000

#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX  1
#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA  227

#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_FLAGS  0
#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_FRAME_LEN_MAX  26



#define FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_FIELD_STATUS_OFS  0


//----------------------------------------
//-- Message ICAROUS_HEARTBEAT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status,
    fmav_status_t* _status)
{
    fmav_icarous_heartbeat_t* _payload = (fmav_icarous_heartbeat_t*)msg->payload;

    _payload->status = status;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_heartbeat_pack(
        msg, sysid, compid,
        _payload->status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status,
    fmav_status_t* _status)
{
    fmav_icarous_heartbeat_t* _payload = (fmav_icarous_heartbeat_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->status = status;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_heartbeat_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t status,
    fmav_status_t* _status)
{
    fmav_icarous_heartbeat_t _payload;

    _payload.status = status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_heartbeat_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_heartbeat_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ICAROUS_HEARTBEAT,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ICAROUS_HEARTBEAT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_icarous_heartbeat_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_icarous_heartbeat_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_icarous_heartbeat_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_icarous_heartbeat_decode(fmav_icarous_heartbeat_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_heartbeat_get_field_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT  42000

#define mavlink_icarous_heartbeat_t  fmav_icarous_heartbeat_t

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_LEN  1
#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_MIN_LEN  1
#define MAVLINK_MSG_ID_42000_LEN  1
#define MAVLINK_MSG_ID_42000_MIN_LEN  1

#define MAVLINK_MSG_ID_ICAROUS_HEARTBEAT_CRC  227
#define MAVLINK_MSG_ID_42000_CRC  227




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_heartbeat_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_icarous_heartbeat_pack(
        msg, sysid, compid,
        status,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_heartbeat_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status)
{
    return fmav_msg_icarous_heartbeat_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_icarous_heartbeat_decode(const mavlink_message_t* msg, mavlink_icarous_heartbeat_t* payload)
{
    fmav_msg_icarous_heartbeat_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ICAROUS_HEARTBEAT_H

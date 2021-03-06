//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_TRIGGER_H
#define FASTMAVLINK_MSG_CAMERA_TRIGGER_H


//----------------------------------------
//-- Message CAMERA_TRIGGER
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_trigger_t {
    uint64_t time_usec;
    uint32_t seq;
}) fmav_camera_trigger_t;


#define FASTMAVLINK_MSG_ID_CAMERA_TRIGGER  112

#define FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA  174

#define FASTMAVLINK_MSG_CAMERA_TRIGGER_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_TRIGGER_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRIGGER_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_TRIGGER_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_CAMERA_TRIGGER_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRIGGER_FIELD_SEQ_OFS  8


//----------------------------------------
//-- Message CAMERA_TRIGGER packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq,
    fmav_status_t* _status)
{
    fmav_camera_trigger_t* _payload = (fmav_camera_trigger_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->seq = seq;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_TRIGGER;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_trigger_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_trigger_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq,
    fmav_status_t* _status)
{
    fmav_camera_trigger_t* _payload = (fmav_camera_trigger_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->seq = seq;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_TRIGGER;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRIGGER >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRIGGER >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_trigger_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_trigger_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->seq,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq,
    fmav_status_t* _status)
{
    fmav_camera_trigger_t _payload;

    _payload.time_usec = time_usec;
    _payload.seq = seq;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRIGGER,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_trigger_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_trigger_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_TRIGGER,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRIGGER_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_TRIGGER unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_camera_trigger_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_camera_trigger_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_trigger_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_trigger_decode(fmav_camera_trigger_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_TRIGGER_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_camera_trigger_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_camera_trigger_get_field_seq(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_TRIGGER  112

#define mavlink_camera_trigger_t  fmav_camera_trigger_t

#define MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN  12
#define MAVLINK_MSG_ID_CAMERA_TRIGGER_MIN_LEN  12
#define MAVLINK_MSG_ID_112_LEN  12
#define MAVLINK_MSG_ID_112_MIN_LEN  12

#define MAVLINK_MSG_ID_CAMERA_TRIGGER_CRC  174
#define MAVLINK_MSG_ID_112_CRC  174




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_trigger_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t seq)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_trigger_pack(
        msg, sysid, compid,
        time_usec, seq,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_trigger_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t seq)
{
    return fmav_msg_camera_trigger_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_trigger_decode(const mavlink_message_t* msg, mavlink_camera_trigger_t* payload)
{
    fmav_msg_camera_trigger_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_TRIGGER_H

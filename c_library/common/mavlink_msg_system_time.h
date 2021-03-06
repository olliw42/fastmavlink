//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SYSTEM_TIME_H
#define FASTMAVLINK_MSG_SYSTEM_TIME_H


//----------------------------------------
//-- Message SYSTEM_TIME
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_system_time_t {
    uint64_t time_unix_usec;
    uint32_t time_boot_ms;
}) fmav_system_time_t;


#define FASTMAVLINK_MSG_ID_SYSTEM_TIME  2

#define FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA  137

#define FASTMAVLINK_MSG_SYSTEM_TIME_FLAGS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SYSTEM_TIME_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_SYSTEM_TIME_FIELD_TIME_UNIX_USEC_OFS  0
#define FASTMAVLINK_MSG_SYSTEM_TIME_FIELD_TIME_BOOT_MS_OFS  8


//----------------------------------------
//-- Message SYSTEM_TIME packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t* _payload = (fmav_system_time_t*)msg->payload;

    _payload->time_unix_usec = time_unix_usec;
    _payload->time_boot_ms = time_boot_ms;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SYSTEM_TIME;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_system_time_pack(
        msg, sysid, compid,
        _payload->time_unix_usec, _payload->time_boot_ms,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t* _payload = (fmav_system_time_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_unix_usec = time_unix_usec;
    _payload->time_boot_ms = time_boot_ms;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SYSTEM_TIME >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_system_time_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_unix_usec, _payload->time_boot_ms,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms,
    fmav_status_t* _status)
{
    fmav_system_time_t _payload;

    _payload.time_unix_usec = time_unix_usec;
    _payload.time_boot_ms = time_boot_ms;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SYSTEM_TIME,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_system_time_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_system_time_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SYSTEM_TIME,
        FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYSTEM_TIME_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SYSTEM_TIME unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_system_time_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_system_time_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_system_time_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_system_time_decode(fmav_system_time_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYSTEM_TIME_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_system_time_get_field_time_unix_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_system_time_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SYSTEM_TIME  2

#define mavlink_system_time_t  fmav_system_time_t

#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN  12
#define MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN  12
#define MAVLINK_MSG_ID_2_LEN  12
#define MAVLINK_MSG_ID_2_MIN_LEN  12

#define MAVLINK_MSG_ID_SYSTEM_TIME_CRC  137
#define MAVLINK_MSG_ID_2_CRC  137




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_system_time_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_system_time_pack(
        msg, sysid, compid,
        time_unix_usec, time_boot_ms,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_system_time_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_unix_usec, uint32_t time_boot_ms)
{
    return fmav_msg_system_time_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_unix_usec, time_boot_ms,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_system_time_decode(const mavlink_message_t* msg, mavlink_system_time_t* payload)
{
    fmav_msg_system_time_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SYSTEM_TIME_H

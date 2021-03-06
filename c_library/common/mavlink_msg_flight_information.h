//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FLIGHT_INFORMATION_H
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_H


//----------------------------------------
//-- Message FLIGHT_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_flight_information_t {
    uint64_t arming_time_utc;
    uint64_t takeoff_time_utc;
    uint64_t flight_uuid;
    uint32_t time_boot_ms;
}) fmav_flight_information_t;


#define FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION  264

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA  49

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_ARMING_TIME_UTC_OFS  0
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_TAKEOFF_TIME_UTC_OFS  8
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_FLIGHT_UUID_OFS  16
#define FASTMAVLINK_MSG_FLIGHT_INFORMATION_FIELD_TIME_BOOT_MS_OFS  24


//----------------------------------------
//-- Message FLIGHT_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid,
    fmav_status_t* _status)
{
    fmav_flight_information_t* _payload = (fmav_flight_information_t*)msg->payload;

    _payload->arming_time_utc = arming_time_utc;
    _payload->takeoff_time_utc = takeoff_time_utc;
    _payload->flight_uuid = flight_uuid;
    _payload->time_boot_ms = time_boot_ms;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_flight_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->arming_time_utc, _payload->takeoff_time_utc, _payload->flight_uuid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid,
    fmav_status_t* _status)
{
    fmav_flight_information_t* _payload = (fmav_flight_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->arming_time_utc = arming_time_utc;
    _payload->takeoff_time_utc = takeoff_time_utc;
    _payload->flight_uuid = flight_uuid;
    _payload->time_boot_ms = time_boot_ms;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_flight_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->arming_time_utc, _payload->takeoff_time_utc, _payload->flight_uuid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid,
    fmav_status_t* _status)
{
    fmav_flight_information_t _payload;

    _payload.arming_time_utc = arming_time_utc;
    _payload.takeoff_time_utc = takeoff_time_utc;
    _payload.flight_uuid = flight_uuid;
    _payload.time_boot_ms = time_boot_ms;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_flight_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_flight_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FLIGHT_INFORMATION,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FLIGHT_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FLIGHT_INFORMATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_flight_information_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_flight_information_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_flight_information_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_flight_information_decode(fmav_flight_information_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FLIGHT_INFORMATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_arming_time_utc(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_takeoff_time_utc(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_flight_information_get_field_flight_uuid(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_flight_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION  264

#define mavlink_flight_information_t  fmav_flight_information_t

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN  28
#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN  28
#define MAVLINK_MSG_ID_264_LEN  28
#define MAVLINK_MSG_ID_264_MIN_LEN  28

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC  49
#define MAVLINK_MSG_ID_264_CRC  49




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_flight_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_flight_information_pack(
        msg, sysid, compid,
        time_boot_ms, arming_time_utc, takeoff_time_utc, flight_uuid,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_flight_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid)
{
    return fmav_msg_flight_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, arming_time_utc, takeoff_time_utc, flight_uuid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_flight_information_decode(const mavlink_message_t* msg, mavlink_flight_information_t* payload)
{
    fmav_msg_flight_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FLIGHT_INFORMATION_H

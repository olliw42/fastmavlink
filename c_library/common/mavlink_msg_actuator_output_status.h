//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_H
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_H


//----------------------------------------
//-- Message ACTUATOR_OUTPUT_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_actuator_output_status_t {
    uint64_t time_usec;
    uint32_t active;
    float actuator[32];
}) fmav_actuator_output_status_t;


#define FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS  375

#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX  140
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA  251

#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FRAME_LEN_MAX  165

#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_LEN  128 // length of array = number of bytes

#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTIVE_OFS  8
#define FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_OFS  12


//----------------------------------------
//-- Message ACTUATOR_OUTPUT_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t active, const float* actuator,
    fmav_status_t* _status)
{
    fmav_actuator_output_status_t* _payload = (fmav_actuator_output_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->active = active;
    memcpy(&(_payload->actuator), actuator, sizeof(float)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_output_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_output_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->active, _payload->actuator,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t active, const float* actuator,
    fmav_status_t* _status)
{
    fmav_actuator_output_status_t* _payload = (fmav_actuator_output_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->active = active;
    memcpy(&(_payload->actuator), actuator, sizeof(float)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_output_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_output_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->active, _payload->actuator,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t active, const float* actuator,
    fmav_status_t* _status)
{
    fmav_actuator_output_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.active = active;
    memcpy(&(_payload.actuator), actuator, sizeof(float)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_output_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_output_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ACTUATOR_OUTPUT_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_actuator_output_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_actuator_output_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_actuator_output_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_actuator_output_status_decode(fmav_actuator_output_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_actuator_output_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_actuator_output_status_get_field_active(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_actuator_output_status_get_field_actuator_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[12]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_actuator_output_status_get_field_actuator(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_NUM) return 0;
    return ((float*)&(msg->payload[12]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS  375

#define mavlink_actuator_output_status_t  fmav_actuator_output_status_t

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_LEN  140
#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_MIN_LEN  140
#define MAVLINK_MSG_ID_375_LEN  140
#define MAVLINK_MSG_ID_375_MIN_LEN  140

#define MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS_CRC  251
#define MAVLINK_MSG_ID_375_CRC  251

#define MAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_FIELD_ACTUATOR_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_output_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t active, const float* actuator)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_actuator_output_status_pack(
        msg, sysid, compid,
        time_usec, active, actuator,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_output_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t active, const float* actuator)
{
    return fmav_msg_actuator_output_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, active, actuator,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_actuator_output_status_decode(const mavlink_message_t* msg, mavlink_actuator_output_status_t* payload)
{
    fmav_msg_actuator_output_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ACTUATOR_OUTPUT_STATUS_H

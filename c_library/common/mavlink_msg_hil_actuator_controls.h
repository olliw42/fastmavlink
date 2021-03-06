//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_actuator_controls_t {
    uint64_t time_usec;
    uint64_t flags;
    float controls[16];
    uint8_t mode;
}) fmav_hil_actuator_controls_t;


#define FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX  81
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA  47

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FRAME_LEN_MAX  106

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN  64 // length of array = number of bytes

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_FLAGS_OFS  8
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_OFS  16
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_MODE_OFS  80


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t _payload;

    _payload.time_usec = time_usec;
    _payload.flags = flags;
    _payload.mode = mode;
    memcpy(&(_payload.controls), controls, sizeof(float)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hil_actuator_controls_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hil_actuator_controls_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_actuator_controls_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_actuator_controls_decode(fmav_hil_actuator_controls_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_actuator_controls_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_actuator_controls_get_field_flags(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_actuator_controls_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[80]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_hil_actuator_controls_get_field_controls_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_actuator_controls_get_field_controls(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_NUM) return 0;
    return ((float*)&(msg->payload[16]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93

#define mavlink_hil_actuator_controls_t  fmav_hil_actuator_controls_t

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN  81
#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN  81
#define MAVLINK_MSG_ID_93_LEN  81
#define MAVLINK_MSG_ID_93_MIN_LEN  81

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC  47
#define MAVLINK_MSG_ID_93_CRC  47

#define MAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_actuator_controls_pack(
        msg, sysid, compid,
        time_usec, controls, mode, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, controls, mode, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* payload)
{
    fmav_msg_hil_actuator_controls_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H

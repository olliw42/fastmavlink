//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VISION_POSITION_DELTA_H
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_H


//----------------------------------------
//-- Message VISION_POSITION_DELTA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vision_position_delta_t {
    uint64_t time_usec;
    uint64_t time_delta_usec;
    float angle_delta[3];
    float position_delta[3];
    float confidence;
}) fmav_vision_position_delta_t;


#define FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA  11011

#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX  44
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA  106

#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FLAGS  0
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FRAME_LEN_MAX  69

#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_LEN  12 // length of array = number of bytes

#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_TIME_DELTA_USEC_OFS  8
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_OFS  16
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_OFS  28
#define FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_CONFIDENCE_OFS  40


//----------------------------------------
//-- Message VISION_POSITION_DELTA packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint64_t time_delta_usec, const float* angle_delta, const float* position_delta, float confidence,
    fmav_status_t* _status)
{
    fmav_vision_position_delta_t* _payload = (fmav_vision_position_delta_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->time_delta_usec = time_delta_usec;
    _payload->confidence = confidence;
    memcpy(&(_payload->angle_delta), angle_delta, sizeof(float)*3);
    memcpy(&(_payload->position_delta), position_delta, sizeof(float)*3);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_delta_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vision_position_delta_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->time_delta_usec, _payload->angle_delta, _payload->position_delta, _payload->confidence,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint64_t time_delta_usec, const float* angle_delta, const float* position_delta, float confidence,
    fmav_status_t* _status)
{
    fmav_vision_position_delta_t* _payload = (fmav_vision_position_delta_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->time_delta_usec = time_delta_usec;
    _payload->confidence = confidence;
    memcpy(&(_payload->angle_delta), angle_delta, sizeof(float)*3);
    memcpy(&(_payload->position_delta), position_delta, sizeof(float)*3);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_delta_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vision_position_delta_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->time_delta_usec, _payload->angle_delta, _payload->position_delta, _payload->confidence,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint64_t time_delta_usec, const float* angle_delta, const float* position_delta, float confidence,
    fmav_status_t* _status)
{
    fmav_vision_position_delta_t _payload;

    _payload.time_usec = time_usec;
    _payload.time_delta_usec = time_delta_usec;
    _payload.confidence = confidence;
    memcpy(&(_payload.angle_delta), angle_delta, sizeof(float)*3);
    memcpy(&(_payload.position_delta), position_delta, sizeof(float)*3);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_delta_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_delta_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VISION_POSITION_DELTA,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_DELTA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VISION_POSITION_DELTA unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_vision_position_delta_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_vision_position_delta_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vision_position_delta_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vision_position_delta_decode(fmav_vision_position_delta_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VISION_POSITION_DELTA_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_vision_position_delta_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_vision_position_delta_get_field_time_delta_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vision_position_delta_get_field_confidence(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_vision_position_delta_get_field_angle_delta_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vision_position_delta_get_field_angle_delta(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_NUM) return 0;
    return ((float*)&(msg->payload[16]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_vision_position_delta_get_field_position_delta_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vision_position_delta_get_field_position_delta(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA  11011

#define mavlink_vision_position_delta_t  fmav_vision_position_delta_t

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_LEN  44
#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN  44
#define MAVLINK_MSG_ID_11011_LEN  44
#define MAVLINK_MSG_ID_11011_MIN_LEN  44

#define MAVLINK_MSG_ID_VISION_POSITION_DELTA_CRC  106
#define MAVLINK_MSG_ID_11011_CRC  106

#define MAVLINK_MSG_VISION_POSITION_DELTA_FIELD_ANGLE_DELTA_LEN 3
#define MAVLINK_MSG_VISION_POSITION_DELTA_FIELD_POSITION_DELTA_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vision_position_delta_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint64_t time_delta_usec, const float* angle_delta, const float* position_delta, float confidence)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vision_position_delta_pack(
        msg, sysid, compid,
        time_usec, time_delta_usec, angle_delta, position_delta, confidence,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vision_position_delta_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint64_t time_delta_usec, const float* angle_delta, const float* position_delta, float confidence)
{
    return fmav_msg_vision_position_delta_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, time_delta_usec, angle_delta, position_delta, confidence,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vision_position_delta_decode(const mavlink_message_t* msg, mavlink_vision_position_delta_t* payload)
{
    fmav_msg_vision_position_delta_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VISION_POSITION_DELTA_H

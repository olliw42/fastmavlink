//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_H
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_H


//----------------------------------------
//-- Message GLOBAL_VISION_POSITION_ESTIMATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_global_vision_position_estimate_t {
    uint64_t usec;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float covariance[21];
    uint8_t reset_counter;
}) fmav_global_vision_position_estimate_t;


#define FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE  101

#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX  117
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA  102

#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FLAGS  0
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FRAME_LEN_MAX  142

#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_NUM  21 // number of elements in array
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN  84 // length of array = number of bytes

#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_USEC_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_X_OFS  8
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_Y_OFS  12
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_Z_OFS  16
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_ROLL_OFS  20
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_PITCH_OFS  24
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_YAW_OFS  28
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_OFS  32
#define FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_RESET_COUNTER_OFS  116


//----------------------------------------
//-- Message GLOBAL_VISION_POSITION_ESTIMATE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_global_vision_position_estimate_t* _payload = (fmav_global_vision_position_estimate_t*)msg->payload;

    _payload->usec = usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->reset_counter = reset_counter;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_vision_position_estimate_pack(
        msg, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance, _payload->reset_counter,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_global_vision_position_estimate_t* _payload = (fmav_global_vision_position_estimate_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->usec = usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->reset_counter = reset_counter;
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_vision_position_estimate_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance, _payload->reset_counter,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_global_vision_position_estimate_t _payload;

    _payload.usec = usec;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.reset_counter = reset_counter;
    memcpy(&(_payload.covariance), covariance, sizeof(float)*21);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_vision_position_estimate_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GLOBAL_VISION_POSITION_ESTIMATE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_global_vision_position_estimate_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_global_vision_position_estimate_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_vision_position_estimate_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_vision_position_estimate_decode(fmav_global_vision_position_estimate_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_global_vision_position_estimate_get_field_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_global_vision_position_estimate_get_field_reset_counter(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[116]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_global_vision_position_estimate_get_field_covariance_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_global_vision_position_estimate_get_field_covariance(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_NUM) return 0;
    return ((float*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE  101

#define mavlink_global_vision_position_estimate_t  fmav_global_vision_position_estimate_t

#define MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_LEN  117
#define MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_MIN_LEN  32
#define MAVLINK_MSG_ID_101_LEN  117
#define MAVLINK_MSG_ID_101_MIN_LEN  32

#define MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE_CRC  102
#define MAVLINK_MSG_ID_101_CRC  102

#define MAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_vision_position_estimate_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_global_vision_position_estimate_pack(
        msg, sysid, compid,
        usec, x, y, z, roll, pitch, yaw, covariance, reset_counter,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_vision_position_estimate_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter)
{
    return fmav_msg_global_vision_position_estimate_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        usec, x, y, z, roll, pitch, yaw, covariance, reset_counter,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_global_vision_position_estimate_decode(const mavlink_message_t* msg, mavlink_global_vision_position_estimate_t* payload)
{
    fmav_msg_global_vision_position_estimate_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GLOBAL_VISION_POSITION_ESTIMATE_H

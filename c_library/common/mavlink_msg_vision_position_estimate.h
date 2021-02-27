//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_H
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_H


//----------------------------------------
//-- Message VISION_POSITION_ESTIMATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vision_position_estimate_t {
    uint64_t usec;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    float covariance[21];
    uint8_t reset_counter;
}) fmav_vision_position_estimate_t;


#define FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE  102


#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX  117
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN  117
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA  158

#define FASTMAVLINK_MSG_ID_102_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_102_LEN_MAX  117
#define FASTMAVLINK_MSG_ID_102_LEN  117
#define FASTMAVLINK_MSG_ID_102_CRCEXTRA  158

#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN  21

#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_FLAGS  0
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_102_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_102_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message VISION_POSITION_ESTIMATE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_vision_position_estimate_t* _payload = (fmav_vision_position_estimate_t*)msg->payload;

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
    msg->msgid = FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vision_position_estimate_pack(
        msg, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance, _payload->reset_counter,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_vision_position_estimate_t* _payload = (fmav_vision_position_estimate_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vision_position_estimate_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->usec, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw, _payload->covariance, _payload->reset_counter,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter,
    fmav_status_t* _status)
{
    fmav_vision_position_estimate_t _payload;

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
        FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vision_position_estimate_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_vision_position_estimate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VISION_POSITION_ESTIMATE,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VISION_POSITION_ESTIMATE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vision_position_estimate_decode(fmav_vision_position_estimate_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE  102

#define mavlink_vision_position_estimate_t  fmav_vision_position_estimate_t

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN  117
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_MIN_LEN  32
#define MAVLINK_MSG_ID_102_LEN  117
#define MAVLINK_MSG_ID_102_MIN_LEN  32

#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC  158
#define MAVLINK_MSG_ID_102_CRC  158

#define MAVLINK_MSG_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vision_position_estimate_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vision_position_estimate_pack(
        msg, sysid, compid,
        usec, x, y, z, roll, pitch, yaw, covariance, reset_counter,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vision_position_estimate_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw, const float* covariance, uint8_t reset_counter)
{
    return fmav_msg_vision_position_estimate_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        usec, x, y, z, roll, pitch, yaw, covariance, reset_counter,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vision_position_estimate_decode(const mavlink_message_t* msg, mavlink_vision_position_estimate_t* payload)
{
    fmav_msg_vision_position_estimate_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VISION_POSITION_ESTIMATE_H

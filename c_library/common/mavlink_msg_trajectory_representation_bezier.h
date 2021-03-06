//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_trajectory_representation_bezier_t {
    uint64_t time_usec;
    float pos_x[5];
    float pos_y[5];
    float pos_z[5];
    float delta[5];
    float pos_yaw[5];
    uint8_t valid_points;
}) fmav_trajectory_representation_bezier_t;


#define FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER  333

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX  109
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA  231

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FLAGS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FRAME_LEN_MAX  134

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_LEN  20 // length of array = number of bytes

#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_OFS  8
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_OFS  28
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_OFS  48
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_OFS  68
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_OFS  88
#define FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_VALID_POINTS_OFS  108


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t* _payload = (fmav_trajectory_representation_bezier_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->valid_points = valid_points;
    memcpy(&(_payload->pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload->pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload->pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload->delta), delta, sizeof(float)*5);
    memcpy(&(_payload->pos_yaw), pos_yaw, sizeof(float)*5);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_trajectory_representation_bezier_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->valid_points, _payload->pos_x, _payload->pos_y, _payload->pos_z, _payload->delta, _payload->pos_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t* _payload = (fmav_trajectory_representation_bezier_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->valid_points = valid_points;
    memcpy(&(_payload->pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload->pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload->pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload->delta), delta, sizeof(float)*5);
    memcpy(&(_payload->pos_yaw), pos_yaw, sizeof(float)*5);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->valid_points, _payload->pos_x, _payload->pos_y, _payload->pos_z, _payload->delta, _payload->pos_yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw,
    fmav_status_t* _status)
{
    fmav_trajectory_representation_bezier_t _payload;

    _payload.time_usec = time_usec;
    _payload.valid_points = valid_points;
    memcpy(&(_payload.pos_x), pos_x, sizeof(float)*5);
    memcpy(&(_payload.pos_y), pos_y, sizeof(float)*5);
    memcpy(&(_payload.pos_z), pos_z, sizeof(float)*5);
    memcpy(&(_payload.delta), delta, sizeof(float)*5);
    memcpy(&(_payload.pos_yaw), pos_yaw, sizeof(float)*5);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_trajectory_representation_bezier_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_trajectory_representation_bezier_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TRAJECTORY_REPRESENTATION_BEZIER unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_trajectory_representation_bezier_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_trajectory_representation_bezier_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_trajectory_representation_bezier_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_trajectory_representation_bezier_decode(fmav_trajectory_representation_bezier_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_trajectory_representation_bezier_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_trajectory_representation_bezier_get_field_valid_points(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[108]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_x_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_x(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_y_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_y(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_NUM) return 0;
    return ((float*)&(msg->payload[28]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_z_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[48]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_z(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_NUM) return 0;
    return ((float*)&(msg->payload[48]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_delta_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[68]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_delta(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_NUM) return 0;
    return ((float*)&(msg->payload[68]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_trajectory_representation_bezier_get_field_pos_yaw_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[88]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_trajectory_representation_bezier_get_field_pos_yaw(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_NUM) return 0;
    return ((float*)&(msg->payload[88]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER  333

#define mavlink_trajectory_representation_bezier_t  fmav_trajectory_representation_bezier_t

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_LEN  109
#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_MIN_LEN  109
#define MAVLINK_MSG_ID_333_LEN  109
#define MAVLINK_MSG_ID_333_MIN_LEN  109

#define MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER_CRC  231
#define MAVLINK_MSG_ID_333_CRC  231

#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_X_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Y_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_Z_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_DELTA_LEN 5
#define MAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_FIELD_POS_YAW_LEN 5


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_trajectory_representation_bezier_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_trajectory_representation_bezier_pack(
        msg, sysid, compid,
        time_usec, valid_points, pos_x, pos_y, pos_z, delta, pos_yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_trajectory_representation_bezier_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t valid_points, const float* pos_x, const float* pos_y, const float* pos_z, const float* delta, const float* pos_yaw)
{
    return fmav_msg_trajectory_representation_bezier_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, valid_points, pos_x, pos_y, pos_z, delta, pos_yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_trajectory_representation_bezier_decode(const mavlink_message_t* msg, mavlink_trajectory_representation_bezier_t* payload)
{
    fmav_msg_trajectory_representation_bezier_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TRAJECTORY_REPRESENTATION_BEZIER_H

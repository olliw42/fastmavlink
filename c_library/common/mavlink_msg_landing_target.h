//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LANDING_TARGET_H
#define FASTMAVLINK_MSG_LANDING_TARGET_H


//----------------------------------------
//-- Message LANDING_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_landing_target_t {
    uint64_t time_usec;
    float angle_x;
    float angle_y;
    float distance;
    float size_x;
    float size_y;
    uint8_t target_num;
    uint8_t frame;
    float x;
    float y;
    float z;
    float q[4];
    uint8_t type;
    uint8_t position_valid;
}) fmav_landing_target_t;


#define FASTMAVLINK_MSG_ID_LANDING_TARGET  149


#define FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MIN  30
#define FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX  60
#define FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN  60
#define FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA  200

#define FASTMAVLINK_MSG_ID_149_LEN_MIN  30
#define FASTMAVLINK_MSG_ID_149_LEN_MAX  60
#define FASTMAVLINK_MSG_ID_149_LEN  60
#define FASTMAVLINK_MSG_ID_149_CRCEXTRA  200

#define FASTMAVLINK_MSG_LANDING_TARGET_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_LANDING_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_LANDING_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LANDING_TARGET_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message LANDING_TARGET packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid,
    fmav_status_t* _status)
{
    fmav_landing_target_t* _payload = (fmav_landing_target_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->angle_x = angle_x;
    _payload->angle_y = angle_y;
    _payload->distance = distance;
    _payload->size_x = size_x;
    _payload->size_y = size_y;
    _payload->target_num = target_num;
    _payload->frame = frame;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->type = type;
    _payload->position_valid = position_valid;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LANDING_TARGET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_landing_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_landing_target_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->target_num, _payload->frame, _payload->angle_x, _payload->angle_y, _payload->distance, _payload->size_x, _payload->size_y, _payload->x, _payload->y, _payload->z, _payload->q, _payload->type, _payload->position_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid,
    fmav_status_t* _status)
{
    fmav_landing_target_t* _payload = (fmav_landing_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->angle_x = angle_x;
    _payload->angle_y = angle_y;
    _payload->distance = distance;
    _payload->size_x = size_x;
    _payload->size_y = size_y;
    _payload->target_num = target_num;
    _payload->frame = frame;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->type = type;
    _payload->position_valid = position_valid;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LANDING_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LANDING_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LANDING_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LANDING_TARGET_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_landing_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_landing_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_landing_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->target_num, _payload->frame, _payload->angle_x, _payload->angle_y, _payload->distance, _payload->size_x, _payload->size_y, _payload->x, _payload->y, _payload->z, _payload->q, _payload->type, _payload->position_valid,
        _status);
}


//----------------------------------------
//-- Message LANDING_TARGET unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_landing_target_decode(fmav_landing_target_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LANDING_TARGET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LANDING_TARGET  149

#define mavlink_landing_target_t  fmav_landing_target_t

#define MAVLINK_MSG_ID_LANDING_TARGET_LEN  60
#define MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN  30
#define MAVLINK_MSG_ID_149_LEN  60
#define MAVLINK_MSG_ID_149_MIN_LEN  30

#define MAVLINK_MSG_ID_LANDING_TARGET_CRC  200
#define MAVLINK_MSG_ID_149_CRC  200

#define MAVLINK_MSG_LANDING_TARGET_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_landing_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_landing_target_pack(
        msg, sysid, compid,
        time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y, x, y, z, q, type, position_valid,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_landing_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_num, uint8_t frame, float angle_x, float angle_y, float distance, float size_x, float size_y, float x, float y, float z, const float* q, uint8_t type, uint8_t position_valid)
{
    return fmav_msg_landing_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, target_num, frame, angle_x, angle_y, distance, size_x, size_y, x, y, z, q, type, position_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_landing_target_decode(const mavlink_message_t* msg, mavlink_landing_target_t* payload)
{
    fmav_msg_landing_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LANDING_TARGET_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATT_POS_MOCAP_H
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_H


//----------------------------------------
//-- Message ATT_POS_MOCAP
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_att_pos_mocap_t {
    uint64_t time_usec;
    float q[4];
    float x;
    float y;
    float z;
    float covariance[21];
}) fmav_att_pos_mocap_t;


#define FASTMAVLINK_MSG_ID_ATT_POS_MOCAP  138


#define FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MIN  36
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX  120
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN  120
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA  109

#define FASTMAVLINK_MSG_ID_138_LEN_MIN  36
#define FASTMAVLINK_MSG_ID_138_LEN_MAX  120
#define FASTMAVLINK_MSG_ID_138_LEN  120
#define FASTMAVLINK_MSG_ID_138_CRCEXTRA  109

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_LEN  4
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_LEN  21

#define FASTMAVLINK_MSG_ATT_POS_MOCAP_FLAGS  0
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATT_POS_MOCAP_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ATT_POS_MOCAP packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance,
    fmav_status_t* _status)
{
    fmav_att_pos_mocap_t* _payload = (fmav_att_pos_mocap_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ATT_POS_MOCAP;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_att_pos_mocap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_att_pos_mocap_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->q, _payload->x, _payload->y, _payload->z, _payload->covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance,
    fmav_status_t* _status)
{
    fmav_att_pos_mocap_t* _payload = (fmav_att_pos_mocap_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    memcpy(&(_payload->q), q, sizeof(float)*4);
    memcpy(&(_payload->covariance), covariance, sizeof(float)*21);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATT_POS_MOCAP >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATT_POS_MOCAP_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_att_pos_mocap_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_att_pos_mocap_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_att_pos_mocap_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->q, _payload->x, _payload->y, _payload->z, _payload->covariance,
        _status);
}


//----------------------------------------
//-- Message ATT_POS_MOCAP unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_att_pos_mocap_decode(fmav_att_pos_mocap_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ATT_POS_MOCAP_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATT_POS_MOCAP  138

#define mavlink_att_pos_mocap_t  fmav_att_pos_mocap_t

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN  120
#define MAVLINK_MSG_ID_ATT_POS_MOCAP_MIN_LEN  36
#define MAVLINK_MSG_ID_138_LEN  120
#define MAVLINK_MSG_ID_138_MIN_LEN  36

#define MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC  109
#define MAVLINK_MSG_ID_138_CRC  109

#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_Q_LEN 4
#define MAVLINK_MSG_ATT_POS_MOCAP_FIELD_COVARIANCE_LEN 21


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_att_pos_mocap_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_att_pos_mocap_pack(
        msg, sysid, compid,
        time_usec, q, x, y, z, covariance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_att_pos_mocap_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* q, float x, float y, float z, const float* covariance)
{
    return fmav_msg_att_pos_mocap_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, q, x, y, z, covariance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_att_pos_mocap_decode(const mavlink_message_t* msg, mavlink_att_pos_mocap_t* payload)
{
    fmav_msg_att_pos_mocap_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATT_POS_MOCAP_H

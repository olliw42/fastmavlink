//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_H
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_H


//----------------------------------------
//-- Message REMOTE_LOG_BLOCK_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_remote_log_block_status_t {
    uint32_t seqno;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t status;
}) fmav_remote_log_block_status_t;


#define FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS  185


#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MIN  7
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN  7
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_CRCEXTRA  186

#define FASTMAVLINK_MSG_ID_185_LEN_MIN  7
#define FASTMAVLINK_MSG_ID_185_LEN_MAX  7
#define FASTMAVLINK_MSG_ID_185_LEN  7
#define FASTMAVLINK_MSG_ID_185_CRCEXTRA  186



#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_FLAGS  3
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_TARGET_COMPONENT_OFS  5


//----------------------------------------
//-- Message REMOTE_LOG_BLOCK_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_block_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, uint8_t status,
    fmav_status_t* _status)
{
    fmav_remote_log_block_status_t* _payload = (fmav_remote_log_block_status_t*)msg->payload;

    _payload->seqno = seqno;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_block_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_remote_log_block_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_remote_log_block_status_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seqno, _payload->status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_block_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, uint8_t status,
    fmav_status_t* _status)
{
    fmav_remote_log_block_status_t* _payload = (fmav_remote_log_block_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->seqno = seqno;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_block_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_remote_log_block_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_remote_log_block_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seqno, _payload->status,
        _status);
}


//----------------------------------------
//-- Message REMOTE_LOG_BLOCK_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_remote_log_block_status_decode(fmav_remote_log_block_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS  185

#define mavlink_remote_log_block_status_t  fmav_remote_log_block_status_t

#define MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_LEN  7
#define MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_MIN_LEN  7
#define MAVLINK_MSG_ID_185_LEN  7
#define MAVLINK_MSG_ID_185_MIN_LEN  7

#define MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_CRC  186
#define MAVLINK_MSG_ID_185_CRC  186




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_remote_log_block_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, uint8_t status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_remote_log_block_status_pack(
        msg, sysid, compid,
        target_system, target_component, seqno, status,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_remote_log_block_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, uint8_t status)
{
    return fmav_msg_remote_log_block_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, seqno, status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_remote_log_block_status_decode(const mavlink_message_t* msg, mavlink_remote_log_block_status_t* payload)
{
    fmav_msg_remote_log_block_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_REMOTE_LOG_BLOCK_STATUS_H

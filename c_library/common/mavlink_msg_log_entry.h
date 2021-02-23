//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_ENTRY_H
#define FASTMAVLINK_MSG_LOG_ENTRY_H


//----------------------------------------
//-- Message LOG_ENTRY
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_entry_t {
    uint32_t time_utc;
    uint32_t size;
    uint16_t id;
    uint16_t num_logs;
    uint16_t last_log_num;
}) fmav_log_entry_t;


#define FASTMAVLINK_MSG_ID_LOG_ENTRY  118


#define FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN  14
#define FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA  56

#define FASTMAVLINK_MSG_ID_118_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_118_LEN_MAX  14
#define FASTMAVLINK_MSG_ID_118_LEN  14
#define FASTMAVLINK_MSG_ID_118_CRCEXTRA  56



#define FASTMAVLINK_MSG_LOG_ENTRY_FLAGS  0
#define FASTMAVLINK_MSG_LOG_ENTRY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_ENTRY_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message LOG_ENTRY packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size,
    fmav_status_t* _status)
{
    fmav_log_entry_t* _payload = (fmav_log_entry_t*)msg->payload;

    _payload->time_utc = time_utc;
    _payload->size = size;
    _payload->id = id;
    _payload->num_logs = num_logs;
    _payload->last_log_num = last_log_num;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOG_ENTRY;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_entry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_entry_pack(
        msg, sysid, compid,
        _payload->id, _payload->num_logs, _payload->last_log_num, _payload->time_utc, _payload->size,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size,
    fmav_status_t* _status)
{
    fmav_log_entry_t* _payload = (fmav_log_entry_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_utc = time_utc;
    _payload->size = size;
    _payload->id = id;
    _payload->num_logs = num_logs;
    _payload->last_log_num = last_log_num;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_ENTRY;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ENTRY >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_ENTRY >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_ENTRY_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_entry_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_entry_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_entry_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->id, _payload->num_logs, _payload->last_log_num, _payload->time_utc, _payload->size,
        _status);
}


//----------------------------------------
//-- Message LOG_ENTRY unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_entry_decode(fmav_log_entry_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOG_ENTRY_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_ENTRY  118

#define mavlink_log_entry_t  fmav_log_entry_t

#define MAVLINK_MSG_ID_LOG_ENTRY_LEN  14
#define MAVLINK_MSG_ID_LOG_ENTRY_MIN_LEN  14
#define MAVLINK_MSG_ID_118_LEN  14
#define MAVLINK_MSG_ID_118_MIN_LEN  14

#define MAVLINK_MSG_ID_LOG_ENTRY_CRC  56
#define MAVLINK_MSG_ID_118_CRC  56




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_entry_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_entry_pack(
        msg, sysid, compid,
        id, num_logs, last_log_num, time_utc, size,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_entry_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint16_t num_logs, uint16_t last_log_num, uint32_t time_utc, uint32_t size)
{
    return fmav_msg_log_entry_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        id, num_logs, last_log_num, time_utc, size,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_entry_decode(const mavlink_message_t* msg, mavlink_log_entry_t* payload)
{
    fmav_msg_log_entry_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_ENTRY_H

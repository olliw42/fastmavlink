//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_INFO_H
#define FASTMAVLINK_MSG_ESC_INFO_H


//----------------------------------------
//-- Message ESC_INFO
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_info_t {
    uint64_t time_usec;
    uint32_t error_count[4];
    uint16_t counter;
    uint16_t failure_flags[4];
    uint8_t index;
    uint8_t count;
    uint8_t connection_type;
    uint8_t info;
    uint8_t temperature[4];
}) fmav_esc_info_t;


#define FASTMAVLINK_MSG_ID_ESC_INFO  290


#define FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA  221

#define FASTMAVLINK_MSG_ID_290_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_290_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_290_LEN  42
#define FASTMAVLINK_MSG_ID_290_CRCEXTRA  221

#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_LEN  4
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_LEN  4
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_LEN  4

#define FASTMAVLINK_MSG_ESC_INFO_FLAGS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ESC_INFO packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const uint8_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t* _payload = (fmav_esc_info_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->counter = counter;
    _payload->index = index;
    _payload->count = count;
    _payload->connection_type = connection_type;
    _payload->info = info;
    memcpy(&(_payload->error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload->failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(uint8_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ESC_INFO;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_info_pack(
        msg, sysid, compid,
        _payload->index, _payload->time_usec, _payload->counter, _payload->count, _payload->connection_type, _payload->info, _payload->failure_flags, _payload->error_count, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const uint8_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t* _payload = (fmav_esc_info_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->counter = counter;
    _payload->index = index;
    _payload->count = count;
    _payload->connection_type = connection_type;
    _payload->info = info;
    memcpy(&(_payload->error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload->failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload->temperature), temperature, sizeof(uint8_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_INFO;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_INFO >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_INFO >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_info_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->index, _payload->time_usec, _payload->counter, _payload->count, _payload->connection_type, _payload->info, _payload->failure_flags, _payload->error_count, _payload->temperature,
        _status);
}


//----------------------------------------
//-- Message ESC_INFO unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_info_decode(fmav_esc_info_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_INFO  290

#define mavlink_esc_info_t  fmav_esc_info_t

#define MAVLINK_MSG_ID_ESC_INFO_LEN  42
#define MAVLINK_MSG_ID_ESC_INFO_MIN_LEN  42
#define MAVLINK_MSG_ID_290_LEN  42
#define MAVLINK_MSG_ID_290_MIN_LEN  42

#define MAVLINK_MSG_ID_ESC_INFO_CRC  221
#define MAVLINK_MSG_ID_290_CRC  221

#define MAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_LEN 4
#define MAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_LEN 4
#define MAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const uint8_t* temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_info_pack(
        msg, sysid, compid,
        index, time_usec, counter, count, connection_type, info, failure_flags, error_count, temperature,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_info_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const uint8_t* temperature)
{
    return fmav_msg_esc_info_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        index, time_usec, counter, count, connection_type, info, failure_flags, error_count, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_info_decode(const mavlink_message_t* msg, mavlink_esc_info_t* payload)
{
    fmav_msg_esc_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_INFO_H

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

#define FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA  221

#define FASTMAVLINK_MSG_ESC_INFO_FLAGS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESC_INFO_FRAME_LEN_MAX  67

#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_LEN  4 // length of array = number of bytes

#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_OFS  8
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_COUNTER_OFS  24
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_OFS  26
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_INDEX_OFS  34
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_COUNT_OFS  35
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_CONNECTION_TYPE_OFS  36
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_INFO_OFS  37
#define FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_OFS  38


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, uint16_t counter, uint8_t count, uint8_t connection_type, uint8_t info, const uint16_t* failure_flags, const uint32_t* error_count, const uint8_t* temperature,
    fmav_status_t* _status)
{
    fmav_esc_info_t _payload;

    _payload.time_usec = time_usec;
    _payload.counter = counter;
    _payload.index = index;
    _payload.count = count;
    _payload.connection_type = connection_type;
    _payload.info = info;
    memcpy(&(_payload.error_count), error_count, sizeof(uint32_t)*4);
    memcpy(&(_payload.failure_flags), failure_flags, sizeof(uint16_t)*4);
    memcpy(&(_payload.temperature), temperature, sizeof(uint8_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_INFO,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_INFO,
        FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_INFO unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_esc_info_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_esc_info_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_info_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_info_decode(fmav_esc_info_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESC_INFO_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_esc_info_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_get_field_counter(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_connection_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_info(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_esc_info_get_field_error_count_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_esc_info_get_field_error_count(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_ERROR_COUNT_NUM) return 0;
    return ((uint32_t*)&(msg->payload[8]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_esc_info_get_field_failure_flags_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[26]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_info_get_field_failure_flags(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_FAILURE_FLAGS_NUM) return 0;
    return ((uint16_t*)&(msg->payload[26]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_esc_info_get_field_temperature_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[38]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_esc_info_get_field_temperature(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ESC_INFO_FIELD_TEMPERATURE_NUM) return 0;
    return ((uint8_t*)&(msg->payload[38]))[index];
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

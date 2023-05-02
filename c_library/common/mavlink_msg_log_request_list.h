//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_REQUEST_LIST_H
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_H


//----------------------------------------
//-- Message LOG_REQUEST_LIST
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_request_list_t {
    uint16_t start;
    uint16_t end;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_log_request_list_t;


#define FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST  117

#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA  128

#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FLAGS  3
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FIELD_START_OFS  0
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FIELD_END_OFS  2
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_LOG_REQUEST_LIST_FIELD_TARGET_COMPONENT_OFS  5


//----------------------------------------
//-- Message LOG_REQUEST_LIST pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end,
    fmav_status_t* _status)
{
    fmav_log_request_list_t* _payload = (fmav_log_request_list_t*)_msg->payload;

    _payload->start = start;
    _payload->end = end;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_list_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->start, _payload->end,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end,
    fmav_status_t* _status)
{
    fmav_log_request_list_t* _payload = (fmav_log_request_list_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->start = start;
    _payload->end = end;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_list_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->start, _payload->end,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end,
    fmav_status_t* _status)
{
    fmav_log_request_list_t _payload;

    _payload.start = start;
    _payload.end = end;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOG_REQUEST_LIST decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_request_list_decode(fmav_log_request_list_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_get_field_start(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_get_field_end(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_request_list_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_request_list_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_REQUEST_LIST  117

#define mavlink_log_request_list_t  fmav_log_request_list_t

#define MAVLINK_MSG_ID_LOG_REQUEST_LIST_LEN  6
#define MAVLINK_MSG_ID_LOG_REQUEST_LIST_MIN_LEN  6
#define MAVLINK_MSG_ID_117_LEN  6
#define MAVLINK_MSG_ID_117_MIN_LEN  6

#define MAVLINK_MSG_ID_LOG_REQUEST_LIST_CRC  128
#define MAVLINK_MSG_ID_117_CRC  128




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_list_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_request_list_pack(
        _msg, sysid, compid,
        target_system, target_component, start, end,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_list_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_log_request_list_t* _payload)
{
    return mavlink_msg_log_request_list_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->start, _payload->end);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_list_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end)
{
    return fmav_msg_log_request_list_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, start, end,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_request_list_decode(const mavlink_message_t* msg, mavlink_log_request_list_t* payload)
{
    fmav_msg_log_request_list_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_REQUEST_LIST_H

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

// fields are ordered, as they are on the wire
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
//-- Message LOG_REQUEST_LIST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end,
    fmav_status_t* _status)
{
    fmav_log_request_list_t* _payload = (fmav_log_request_list_t*)msg->payload;

    _payload->start = start;
    _payload->end = end;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_list_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->start, _payload->end,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end,
    fmav_status_t* _status)
{
    fmav_log_request_list_t* _payload = (fmav_log_request_list_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->start = start;
    _payload->end = end;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_REQUEST_LIST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_REQUEST_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_request_list_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_request_list_pack_to_frame_buf(
        buf, sysid, compid,
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
//-- Message LOG_REQUEST_LIST unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_log_request_list_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_log_request_list_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_request_list_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_request_list_decode(fmav_log_request_list_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_REQUEST_LIST_PAYLOAD_LEN_MAX);
    }
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
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_request_list_pack(
        msg, sysid, compid,
        target_system, target_component, start, end,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_request_list_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t start, uint16_t end)
{
    return fmav_msg_log_request_list_pack_to_frame_buf(
        (uint8_t*)buf,
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

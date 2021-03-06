//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H


//----------------------------------------
//-- Message REQUEST_DATA_STREAM
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_request_data_stream_t {
    uint16_t req_message_rate;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t req_stream_id;
    uint8_t start_stop;
}) fmav_request_data_stream_t;


#define FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM  66

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA  148

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FLAGS  3
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_REQ_MESSAGE_RATE_OFS  0
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_REQ_STREAM_ID_OFS  4
#define FASTMAVLINK_MSG_REQUEST_DATA_STREAM_FIELD_START_STOP_OFS  5


//----------------------------------------
//-- Message REQUEST_DATA_STREAM packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t* _payload = (fmav_request_data_stream_t*)msg->payload;

    _payload->req_message_rate = req_message_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->req_stream_id = req_stream_id;
    _payload->start_stop = start_stop;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_request_data_stream_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->req_stream_id, _payload->req_message_rate, _payload->start_stop,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t* _payload = (fmav_request_data_stream_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->req_message_rate = req_message_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->req_stream_id = req_stream_id;
    _payload->start_stop = start_stop;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_request_data_stream_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->req_stream_id, _payload->req_message_rate, _payload->start_stop,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop,
    fmav_status_t* _status)
{
    fmav_request_data_stream_t _payload;

    _payload.req_message_rate = req_message_rate;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.req_stream_id = req_stream_id;
    _payload.start_stop = start_stop;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_request_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_REQUEST_DATA_STREAM,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REQUEST_DATA_STREAM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message REQUEST_DATA_STREAM unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_request_data_stream_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_request_data_stream_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_request_data_stream_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_request_data_stream_decode(fmav_request_data_stream_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_REQUEST_DATA_STREAM_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_request_data_stream_get_field_req_message_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_req_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_request_data_stream_get_field_start_stop(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM  66

#define mavlink_request_data_stream_t  fmav_request_data_stream_t

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN  6
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MIN_LEN  6
#define MAVLINK_MSG_ID_66_LEN  6
#define MAVLINK_MSG_ID_66_MIN_LEN  6

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC  148
#define MAVLINK_MSG_ID_66_CRC  148




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_request_data_stream_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_request_data_stream_pack(
        msg, sysid, compid,
        target_system, target_component, req_stream_id, req_message_rate, start_stop,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_request_data_stream_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    return fmav_msg_request_data_stream_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, req_stream_id, req_message_rate, start_stop,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_request_data_stream_decode(const mavlink_message_t* msg, mavlink_request_data_stream_t* payload)
{
    fmav_msg_request_data_stream_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_REQUEST_DATA_STREAM_H

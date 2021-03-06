//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_H
#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_H


//----------------------------------------
//-- Message PARAM_EXT_REQUEST_LIST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ext_request_list_t {
    uint8_t target_system;
    uint8_t target_component;
}) fmav_param_ext_request_list_t;


#define FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST  321

#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA  88

#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_FRAME_LEN_MAX  27



#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_FIELD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_FIELD_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message PARAM_EXT_REQUEST_LIST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_param_ext_request_list_t* _payload = (fmav_param_ext_request_list_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_request_list_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_param_ext_request_list_t* _payload = (fmav_param_ext_request_list_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_request_list_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component,
    fmav_status_t* _status)
{
    fmav_param_ext_request_list_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_request_list_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_request_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_EXT_REQUEST_LIST unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_param_ext_request_list_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_param_ext_request_list_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_request_list_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_request_list_decode(fmav_param_ext_request_list_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_request_list_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_ext_request_list_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST  321

#define mavlink_param_ext_request_list_t  fmav_param_ext_request_list_t

#define MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST_LEN  2
#define MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST_MIN_LEN  2
#define MAVLINK_MSG_ID_321_LEN  2
#define MAVLINK_MSG_ID_321_MIN_LEN  2

#define MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST_CRC  88
#define MAVLINK_MSG_ID_321_CRC  88




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_request_list_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ext_request_list_pack(
        msg, sysid, compid,
        target_system, target_component,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_request_list_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component)
{
    return fmav_msg_param_ext_request_list_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ext_request_list_decode(const mavlink_message_t* msg, mavlink_param_ext_request_list_t* payload)
{
    fmav_msg_param_ext_request_list_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_EXT_REQUEST_LIST_H

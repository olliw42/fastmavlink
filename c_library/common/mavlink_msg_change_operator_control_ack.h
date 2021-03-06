//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_change_operator_control_ack_t {
    uint8_t gcs_system_id;
    uint8_t control_request;
    uint8_t ack;
}) fmav_change_operator_control_ack_t;


#define FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK  6

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA  104

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FLAGS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FRAME_LEN_MAX  28



#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_GCS_SYSTEM_ID_OFS  0
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_CONTROL_REQUEST_OFS  1
#define FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_FIELD_ACK_OFS  2


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t* _payload = (fmav_change_operator_control_ack_t*)msg->payload;

    _payload->gcs_system_id = gcs_system_id;
    _payload->control_request = control_request;
    _payload->ack = ack;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_ack_pack(
        msg, sysid, compid,
        _payload->gcs_system_id, _payload->control_request, _payload->ack,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t* _payload = (fmav_change_operator_control_ack_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->gcs_system_id = gcs_system_id;
    _payload->control_request = control_request;
    _payload->ack = ack;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_change_operator_control_ack_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->gcs_system_id, _payload->control_request, _payload->ack,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack,
    fmav_status_t* _status)
{
    fmav_change_operator_control_ack_t _payload;

    _payload.gcs_system_id = gcs_system_id;
    _payload.control_request = control_request;
    _payload.ack = ack;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_change_operator_control_ack_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_change_operator_control_ack_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CHANGE_OPERATOR_CONTROL_ACK unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_change_operator_control_ack_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_change_operator_control_ack_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_change_operator_control_ack_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_change_operator_control_ack_decode(fmav_change_operator_control_ack_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_gcs_system_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_control_request(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_change_operator_control_ack_get_field_ack(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK  6

#define mavlink_change_operator_control_ack_t  fmav_change_operator_control_ack_t

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_LEN  3
#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_MIN_LEN  3
#define MAVLINK_MSG_ID_6_LEN  3
#define MAVLINK_MSG_ID_6_MIN_LEN  3

#define MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK_CRC  104
#define MAVLINK_MSG_ID_6_CRC  104




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_ack_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_change_operator_control_ack_pack(
        msg, sysid, compid,
        gcs_system_id, control_request, ack,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_change_operator_control_ack_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t gcs_system_id, uint8_t control_request, uint8_t ack)
{
    return fmav_msg_change_operator_control_ack_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        gcs_system_id, control_request, ack,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_change_operator_control_ack_decode(const mavlink_message_t* msg, mavlink_change_operator_control_ack_t* payload)
{
    fmav_msg_change_operator_control_ack_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CHANGE_OPERATOR_CONTROL_ACK_H

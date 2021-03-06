//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_QSHOT_STATUS_H
#define FASTMAVLINK_MSG_QSHOT_STATUS_H


//----------------------------------------
//-- Message QSHOT_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_qshot_status_t {
    uint16_t mode;
    uint16_t shot_state;
}) fmav_qshot_status_t;


#define FASTMAVLINK_MSG_ID_QSHOT_STATUS  60020

#define FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA  202

#define FASTMAVLINK_MSG_QSHOT_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_QSHOT_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_QSHOT_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_QSHOT_STATUS_FRAME_LEN_MAX  29



#define FASTMAVLINK_MSG_QSHOT_STATUS_FIELD_MODE_OFS  0
#define FASTMAVLINK_MSG_QSHOT_STATUS_FIELD_SHOT_STATE_OFS  2


//----------------------------------------
//-- Message QSHOT_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t mode, uint16_t shot_state,
    fmav_status_t* _status)
{
    fmav_qshot_status_t* _payload = (fmav_qshot_status_t*)msg->payload;

    _payload->mode = mode;
    _payload->shot_state = shot_state;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_QSHOT_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_qshot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_qshot_status_pack(
        msg, sysid, compid,
        _payload->mode, _payload->shot_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t mode, uint16_t shot_state,
    fmav_status_t* _status)
{
    fmav_qshot_status_t* _payload = (fmav_qshot_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mode = mode;
    _payload->shot_state = shot_state;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_QSHOT_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_QSHOT_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_QSHOT_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_qshot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_qshot_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->mode, _payload->shot_state,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t mode, uint16_t shot_state,
    fmav_status_t* _status)
{
    fmav_qshot_status_t _payload;

    _payload.mode = mode;
    _payload.shot_state = shot_state;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_QSHOT_STATUS,
        FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_qshot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_QSHOT_STATUS,
        FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_QSHOT_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message QSHOT_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_qshot_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_qshot_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_qshot_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_qshot_status_decode(fmav_qshot_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_QSHOT_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_get_field_mode(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_qshot_status_get_field_shot_state(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_QSHOT_STATUS  60020

#define mavlink_qshot_status_t  fmav_qshot_status_t

#define MAVLINK_MSG_ID_QSHOT_STATUS_LEN  4
#define MAVLINK_MSG_ID_QSHOT_STATUS_MIN_LEN  4
#define MAVLINK_MSG_ID_60020_LEN  4
#define MAVLINK_MSG_ID_60020_MIN_LEN  4

#define MAVLINK_MSG_ID_QSHOT_STATUS_CRC  202
#define MAVLINK_MSG_ID_60020_CRC  202




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_qshot_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t mode, uint16_t shot_state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_qshot_status_pack(
        msg, sysid, compid,
        mode, shot_state,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_qshot_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t mode, uint16_t shot_state)
{
    return fmav_msg_qshot_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        mode, shot_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_qshot_status_decode(const mavlink_message_t* msg, mavlink_qshot_status_t* payload)
{
    fmav_msg_qshot_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_QSHOT_STATUS_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMPASSMOT_STATUS_H
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_H


//----------------------------------------
//-- Message COMPASSMOT_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_compassmot_status_t {
    float current;
    float CompensationX;
    float CompensationY;
    float CompensationZ;
    uint16_t throttle;
    uint16_t interference;
}) fmav_compassmot_status_t;


#define FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS  177

#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA  240

#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_CURRENT_OFS  0
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_COMPENSATIONX_OFS  4
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_COMPENSATIONY_OFS  8
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_COMPENSATIONZ_OFS  12
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_THROTTLE_OFS  16
#define FASTMAVLINK_MSG_COMPASSMOT_STATUS_FIELD_INTERFERENCE_OFS  18


//----------------------------------------
//-- Message COMPASSMOT_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ,
    fmav_status_t* _status)
{
    fmav_compassmot_status_t* _payload = (fmav_compassmot_status_t*)msg->payload;

    _payload->current = current;
    _payload->CompensationX = CompensationX;
    _payload->CompensationY = CompensationY;
    _payload->CompensationZ = CompensationZ;
    _payload->throttle = throttle;
    _payload->interference = interference;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_compassmot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_compassmot_status_pack(
        msg, sysid, compid,
        _payload->throttle, _payload->current, _payload->interference, _payload->CompensationX, _payload->CompensationY, _payload->CompensationZ,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ,
    fmav_status_t* _status)
{
    fmav_compassmot_status_t* _payload = (fmav_compassmot_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->current = current;
    _payload->CompensationX = CompensationX;
    _payload->CompensationY = CompensationY;
    _payload->CompensationZ = CompensationZ;
    _payload->throttle = throttle;
    _payload->interference = interference;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_compassmot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_compassmot_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->throttle, _payload->current, _payload->interference, _payload->CompensationX, _payload->CompensationY, _payload->CompensationZ,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ,
    fmav_status_t* _status)
{
    fmav_compassmot_status_t _payload;

    _payload.current = current;
    _payload.CompensationX = CompensationX;
    _payload.CompensationY = CompensationY;
    _payload.CompensationZ = CompensationZ;
    _payload.throttle = throttle;
    _payload.interference = interference;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_compassmot_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMPASSMOT_STATUS,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPASSMOT_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMPASSMOT_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_compassmot_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_compassmot_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_compassmot_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_compassmot_status_decode(fmav_compassmot_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPASSMOT_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_compassmot_status_get_field_current(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_compassmot_status_get_field_CompensationX(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_compassmot_status_get_field_CompensationY(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_compassmot_status_get_field_CompensationZ(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_get_field_throttle(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_compassmot_status_get_field_interference(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS  177

#define mavlink_compassmot_status_t  fmav_compassmot_status_t

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_LEN  20
#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN  20
#define MAVLINK_MSG_ID_177_LEN  20
#define MAVLINK_MSG_ID_177_MIN_LEN  20

#define MAVLINK_MSG_ID_COMPASSMOT_STATUS_CRC  240
#define MAVLINK_MSG_ID_177_CRC  240




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_compassmot_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_compassmot_status_pack(
        msg, sysid, compid,
        throttle, current, interference, CompensationX, CompensationY, CompensationZ,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_compassmot_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t throttle, float current, uint16_t interference, float CompensationX, float CompensationY, float CompensationZ)
{
    return fmav_msg_compassmot_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        throttle, current, interference, CompensationX, CompensationY, CompensationZ,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_compassmot_status_decode(const mavlink_message_t* msg, mavlink_compassmot_status_t* payload)
{
    fmav_msg_compassmot_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMPASSMOT_STATUS_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_H
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_H


//----------------------------------------
//-- Message SAFETY_SET_ALLOWED_AREA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_safety_set_allowed_area_t {
    float p1x;
    float p1y;
    float p1z;
    float p2x;
    float p2y;
    float p2z;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
}) fmav_safety_set_allowed_area_t;


#define FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA  54

#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX  27
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA  15

#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FLAGS  3
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_TARGET_SYSTEM_OFS  24
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_TARGET_COMPONENT_OFS  25

#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FRAME_LEN_MAX  52



#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P1X_OFS  0
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P1Y_OFS  4
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P1Z_OFS  8
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P2X_OFS  12
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P2Y_OFS  16
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_P2Z_OFS  20
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_TARGET_SYSTEM_OFS  24
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_TARGET_COMPONENT_OFS  25
#define FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_FIELD_FRAME_OFS  26


//----------------------------------------
//-- Message SAFETY_SET_ALLOWED_AREA packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_set_allowed_area_t* _payload = (fmav_safety_set_allowed_area_t*)msg->payload;

    _payload->p1x = p1x;
    _payload->p1y = p1y;
    _payload->p1z = p1z;
    _payload->p2x = p2x;
    _payload->p2y = p2y;
    _payload->p2z = p2z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_set_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_safety_set_allowed_area_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->frame, _payload->p1x, _payload->p1y, _payload->p1z, _payload->p2x, _payload->p2y, _payload->p2z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_set_allowed_area_t* _payload = (fmav_safety_set_allowed_area_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->p1x = p1x;
    _payload->p1y = p1y;
    _payload->p1z = p1z;
    _payload->p2x = p2x;
    _payload->p2y = p2y;
    _payload->p2z = p2z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->frame = frame;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_set_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_safety_set_allowed_area_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->frame, _payload->p1x, _payload->p1y, _payload->p1z, _payload->p2x, _payload->p2y, _payload->p2z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z,
    fmav_status_t* _status)
{
    fmav_safety_set_allowed_area_t _payload;

    _payload.p1x = p1x;
    _payload.p1y = p1y;
    _payload.p1z = p1z;
    _payload.p2x = p2x;
    _payload.p2y = p2y;
    _payload.p2z = p2z;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.frame = frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_safety_set_allowed_area_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_safety_set_allowed_area_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SAFETY_SET_ALLOWED_AREA unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_safety_set_allowed_area_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_safety_set_allowed_area_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_safety_set_allowed_area_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_safety_set_allowed_area_decode(fmav_safety_set_allowed_area_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p1x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p1y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p1z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p2x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p2y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_safety_set_allowed_area_get_field_p2z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_safety_set_allowed_area_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_safety_set_allowed_area_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_safety_set_allowed_area_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA  54

#define mavlink_safety_set_allowed_area_t  fmav_safety_set_allowed_area_t

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN  27
#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN  27
#define MAVLINK_MSG_ID_54_LEN  27
#define MAVLINK_MSG_ID_54_MIN_LEN  27

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC  15
#define MAVLINK_MSG_ID_54_CRC  15




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_safety_set_allowed_area_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_safety_set_allowed_area_pack(
        msg, sysid, compid,
        target_system, target_component, frame, p1x, p1y, p1z, p2x, p2y, p2z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_safety_set_allowed_area_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
    return fmav_msg_safety_set_allowed_area_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, frame, p1x, p1y, p1z, p2x, p2y, p2z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_safety_set_allowed_area_decode(const mavlink_message_t* msg, mavlink_safety_set_allowed_area_t* payload)
{
    fmav_msg_safety_set_allowed_area_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SAFETY_SET_ALLOWED_AREA_H

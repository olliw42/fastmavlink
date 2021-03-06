//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_CONTROLS_H
#define FASTMAVLINK_MSG_HIL_CONTROLS_H


//----------------------------------------
//-- Message HIL_CONTROLS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_controls_t {
    uint64_t time_usec;
    float roll_ailerons;
    float pitch_elevator;
    float yaw_rudder;
    float throttle;
    float aux1;
    float aux2;
    float aux3;
    float aux4;
    uint8_t mode;
    uint8_t nav_mode;
}) fmav_hil_controls_t;


#define FASTMAVLINK_MSG_ID_HIL_CONTROLS  91

#define FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA  63

#define FASTMAVLINK_MSG_HIL_CONTROLS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_CONTROLS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_CONTROLS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_CONTROLS_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_ROLL_AILERONS_OFS  8
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_PITCH_ELEVATOR_OFS  12
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_YAW_RUDDER_OFS  16
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_THROTTLE_OFS  20
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_AUX1_OFS  24
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_AUX2_OFS  28
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_AUX3_OFS  32
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_AUX4_OFS  36
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_MODE_OFS  40
#define FASTMAVLINK_MSG_HIL_CONTROLS_FIELD_NAV_MODE_OFS  41


//----------------------------------------
//-- Message HIL_CONTROLS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, float aux1, float aux2, float aux3, float aux4, uint8_t mode, uint8_t nav_mode,
    fmav_status_t* _status)
{
    fmav_hil_controls_t* _payload = (fmav_hil_controls_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->roll_ailerons = roll_ailerons;
    _payload->pitch_elevator = pitch_elevator;
    _payload->yaw_rudder = yaw_rudder;
    _payload->throttle = throttle;
    _payload->aux1 = aux1;
    _payload->aux2 = aux2;
    _payload->aux3 = aux3;
    _payload->aux4 = aux4;
    _payload->mode = mode;
    _payload->nav_mode = nav_mode;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_CONTROLS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_controls_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->roll_ailerons, _payload->pitch_elevator, _payload->yaw_rudder, _payload->throttle, _payload->aux1, _payload->aux2, _payload->aux3, _payload->aux4, _payload->mode, _payload->nav_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, float aux1, float aux2, float aux3, float aux4, uint8_t mode, uint8_t nav_mode,
    fmav_status_t* _status)
{
    fmav_hil_controls_t* _payload = (fmav_hil_controls_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->roll_ailerons = roll_ailerons;
    _payload->pitch_elevator = pitch_elevator;
    _payload->yaw_rudder = yaw_rudder;
    _payload->throttle = throttle;
    _payload->aux1 = aux1;
    _payload->aux2 = aux2;
    _payload->aux3 = aux3;
    _payload->aux4 = aux4;
    _payload->mode = mode;
    _payload->nav_mode = nav_mode;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_CONTROLS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_CONTROLS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_CONTROLS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_controls_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->roll_ailerons, _payload->pitch_elevator, _payload->yaw_rudder, _payload->throttle, _payload->aux1, _payload->aux2, _payload->aux3, _payload->aux4, _payload->mode, _payload->nav_mode,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, float aux1, float aux2, float aux3, float aux4, uint8_t mode, uint8_t nav_mode,
    fmav_status_t* _status)
{
    fmav_hil_controls_t _payload;

    _payload.time_usec = time_usec;
    _payload.roll_ailerons = roll_ailerons;
    _payload.pitch_elevator = pitch_elevator;
    _payload.yaw_rudder = yaw_rudder;
    _payload.throttle = throttle;
    _payload.aux1 = aux1;
    _payload.aux2 = aux2;
    _payload.aux3 = aux3;
    _payload.aux4 = aux4;
    _payload.mode = mode;
    _payload.nav_mode = nav_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_CONTROLS,
        FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_controls_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_CONTROLS,
        FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_CONTROLS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_CONTROLS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hil_controls_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hil_controls_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_controls_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_controls_decode(fmav_hil_controls_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_CONTROLS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_controls_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_roll_ailerons(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_pitch_elevator(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_yaw_rudder(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_throttle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_aux1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_aux2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_aux3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_controls_get_field_aux4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_controls_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_controls_get_field_nav_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_CONTROLS  91

#define mavlink_hil_controls_t  fmav_hil_controls_t

#define MAVLINK_MSG_ID_HIL_CONTROLS_LEN  42
#define MAVLINK_MSG_ID_HIL_CONTROLS_MIN_LEN  42
#define MAVLINK_MSG_ID_91_LEN  42
#define MAVLINK_MSG_ID_91_MIN_LEN  42

#define MAVLINK_MSG_ID_HIL_CONTROLS_CRC  63
#define MAVLINK_MSG_ID_91_CRC  63




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_controls_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, float aux1, float aux2, float aux3, float aux4, uint8_t mode, uint8_t nav_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_controls_pack(
        msg, sysid, compid,
        time_usec, roll_ailerons, pitch_elevator, yaw_rudder, throttle, aux1, aux2, aux3, aux4, mode, nav_mode,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_controls_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, float aux1, float aux2, float aux3, float aux4, uint8_t mode, uint8_t nav_mode)
{
    return fmav_msg_hil_controls_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, roll_ailerons, pitch_elevator, yaw_rudder, throttle, aux1, aux2, aux3, aux4, mode, nav_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_controls_decode(const mavlink_message_t* msg, mavlink_hil_controls_t* payload)
{
    fmav_msg_hil_controls_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_CONTROLS_H

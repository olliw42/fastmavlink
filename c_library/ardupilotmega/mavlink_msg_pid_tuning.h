//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PID_TUNING_H
#define FASTMAVLINK_MSG_PID_TUNING_H


//----------------------------------------
//-- Message PID_TUNING
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_pid_tuning_t {
    float desired;
    float achieved;
    float FF;
    float P;
    float I;
    float D;
    uint8_t axis;
}) fmav_pid_tuning_t;


#define FASTMAVLINK_MSG_ID_PID_TUNING  194

#define FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX  25
#define FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA  98

#define FASTMAVLINK_MSG_PID_TUNING_FLAGS  0
#define FASTMAVLINK_MSG_PID_TUNING_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PID_TUNING_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_PID_TUNING_FRAME_LEN_MAX  50



#define FASTMAVLINK_MSG_PID_TUNING_FIELD_DESIRED_OFS  0
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_ACHIEVED_OFS  4
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_FF_OFS  8
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_P_OFS  12
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_I_OFS  16
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_D_OFS  20
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_AXIS_OFS  24


//----------------------------------------
//-- Message PID_TUNING packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D,
    fmav_status_t* _status)
{
    fmav_pid_tuning_t* _payload = (fmav_pid_tuning_t*)msg->payload;

    _payload->desired = desired;
    _payload->achieved = achieved;
    _payload->FF = FF;
    _payload->P = P;
    _payload->I = I;
    _payload->D = D;
    _payload->axis = axis;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PID_TUNING;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_pid_tuning_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_pid_tuning_pack(
        msg, sysid, compid,
        _payload->axis, _payload->desired, _payload->achieved, _payload->FF, _payload->P, _payload->I, _payload->D,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D,
    fmav_status_t* _status)
{
    fmav_pid_tuning_t* _payload = (fmav_pid_tuning_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->desired = desired;
    _payload->achieved = achieved;
    _payload->FF = FF;
    _payload->P = P;
    _payload->I = I;
    _payload->D = D;
    _payload->axis = axis;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PID_TUNING;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PID_TUNING >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PID_TUNING >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_pid_tuning_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_pid_tuning_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->axis, _payload->desired, _payload->achieved, _payload->FF, _payload->P, _payload->I, _payload->D,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D,
    fmav_status_t* _status)
{
    fmav_pid_tuning_t _payload;

    _payload.desired = desired;
    _payload.achieved = achieved;
    _payload.FF = FF;
    _payload.P = P;
    _payload.I = I;
    _payload.D = D;
    _payload.axis = axis;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PID_TUNING,
        FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_pid_tuning_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PID_TUNING,
        FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PID_TUNING unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_pid_tuning_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_pid_tuning_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_pid_tuning_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_pid_tuning_decode(fmav_pid_tuning_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_desired(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_achieved(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_FF(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_P(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_I(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_D(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_pid_tuning_get_field_axis(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PID_TUNING  194

#define mavlink_pid_tuning_t  fmav_pid_tuning_t

#define MAVLINK_MSG_ID_PID_TUNING_LEN  25
#define MAVLINK_MSG_ID_PID_TUNING_MIN_LEN  25
#define MAVLINK_MSG_ID_194_LEN  25
#define MAVLINK_MSG_ID_194_MIN_LEN  25

#define MAVLINK_MSG_ID_PID_TUNING_CRC  98
#define MAVLINK_MSG_ID_194_CRC  98




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_pid_tuning_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_pid_tuning_pack(
        msg, sysid, compid,
        axis, desired, achieved, FF, P, I, D,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_pid_tuning_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D)
{
    return fmav_msg_pid_tuning_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        axis, desired, achieved, FF, P, I, D,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_pid_tuning_decode(const mavlink_message_t* msg, mavlink_pid_tuning_t* payload)
{
    fmav_msg_pid_tuning_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PID_TUNING_H

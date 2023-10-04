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

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_pid_tuning_t {
    float desired;
    float achieved;
    float FF;
    float P;
    float I;
    float D;
    uint8_t axis;
    float SRate;
    float PDmod;
}) fmav_pid_tuning_t;


#define FASTMAVLINK_MSG_ID_PID_TUNING  194

#define FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA  98

#define FASTMAVLINK_MSG_PID_TUNING_FLAGS  0
#define FASTMAVLINK_MSG_PID_TUNING_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PID_TUNING_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_PID_TUNING_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_PID_TUNING_FIELD_DESIRED_OFS  0
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_ACHIEVED_OFS  4
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_FF_OFS  8
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_P_OFS  12
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_I_OFS  16
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_D_OFS  20
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_AXIS_OFS  24
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_SRATE_OFS  25
#define FASTMAVLINK_MSG_PID_TUNING_FIELD_PDMOD_OFS  29


//----------------------------------------
//-- Message PID_TUNING pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D, float SRate, float PDmod,
    fmav_status_t* _status)
{
    fmav_pid_tuning_t* _payload = (fmav_pid_tuning_t*)_msg->payload;

    _payload->desired = desired;
    _payload->achieved = achieved;
    _payload->FF = FF;
    _payload->P = P;
    _payload->I = I;
    _payload->D = D;
    _payload->axis = axis;
    _payload->SRate = SRate;
    _payload->PDmod = PDmod;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PID_TUNING;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_pid_tuning_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_pid_tuning_pack(
        _msg, sysid, compid,
        _payload->axis, _payload->desired, _payload->achieved, _payload->FF, _payload->P, _payload->I, _payload->D, _payload->SRate, _payload->PDmod,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D, float SRate, float PDmod,
    fmav_status_t* _status)
{
    fmav_pid_tuning_t* _payload = (fmav_pid_tuning_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->desired = desired;
    _payload->achieved = achieved;
    _payload->FF = FF;
    _payload->P = P;
    _payload->I = I;
    _payload->D = D;
    _payload->axis = axis;
    _payload->SRate = SRate;
    _payload->PDmod = PDmod;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PID_TUNING;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PID_TUNING >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PID_TUNING >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PID_TUNING_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_pid_tuning_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_pid_tuning_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->axis, _payload->desired, _payload->achieved, _payload->FF, _payload->P, _payload->I, _payload->D, _payload->SRate, _payload->PDmod,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_pid_tuning_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D, float SRate, float PDmod,
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
    _payload.SRate = SRate;
    _payload.PDmod = PDmod;


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
//-- Message PID_TUNING decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_pid_tuning_decode(fmav_pid_tuning_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PID_TUNING_PAYLOAD_LEN_MAX);
#endif
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


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_SRate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[25]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_pid_tuning_get_field_PDmod(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[29]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PID_TUNING  194

#define mavlink_pid_tuning_t  fmav_pid_tuning_t

#define MAVLINK_MSG_ID_PID_TUNING_LEN  33
#define MAVLINK_MSG_ID_PID_TUNING_MIN_LEN  25
#define MAVLINK_MSG_ID_194_LEN  33
#define MAVLINK_MSG_ID_194_MIN_LEN  25

#define MAVLINK_MSG_ID_PID_TUNING_CRC  98
#define MAVLINK_MSG_ID_194_CRC  98




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_pid_tuning_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D, float SRate, float PDmod)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_pid_tuning_pack(
        _msg, sysid, compid,
        axis, desired, achieved, FF, P, I, D, SRate, PDmod,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_pid_tuning_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_pid_tuning_t* _payload)
{
    return mavlink_msg_pid_tuning_pack(
        sysid,
        compid,
        _msg,
        _payload->axis, _payload->desired, _payload->achieved, _payload->FF, _payload->P, _payload->I, _payload->D, _payload->SRate, _payload->PDmod);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_pid_tuning_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t axis, float desired, float achieved, float FF, float P, float I, float D, float SRate, float PDmod)
{
    return fmav_msg_pid_tuning_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        axis, desired, achieved, FF, P, I, D, SRate, PDmod,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_pid_tuning_decode(const mavlink_message_t* msg, mavlink_pid_tuning_t* payload)
{
    fmav_msg_pid_tuning_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PID_TUNING_H

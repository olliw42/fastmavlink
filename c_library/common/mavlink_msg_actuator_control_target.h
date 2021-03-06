//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_actuator_control_target_t {
    uint64_t time_usec;
    float controls[8];
    uint8_t group_mlx;
}) fmav_actuator_control_target_t;


#define FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET  140

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA  181

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FRAME_LEN_MAX  66

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_OFS  8
#define FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_GROUP_MLX_OFS  40


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls,
    fmav_status_t* _status)
{
    fmav_actuator_control_target_t* _payload = (fmav_actuator_control_target_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->group_mlx = group_mlx;
    memcpy(&(_payload->controls), controls, sizeof(float)*8);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_control_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_control_target_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->group_mlx, _payload->controls,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls,
    fmav_status_t* _status)
{
    fmav_actuator_control_target_t* _payload = (fmav_actuator_control_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->group_mlx = group_mlx;
    memcpy(&(_payload->controls), controls, sizeof(float)*8);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_control_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_actuator_control_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->group_mlx, _payload->controls,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls,
    fmav_status_t* _status)
{
    fmav_actuator_control_target_t _payload;

    _payload.time_usec = time_usec;
    _payload.group_mlx = group_mlx;
    memcpy(&(_payload.controls), controls, sizeof(float)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_actuator_control_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_actuator_control_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ACTUATOR_CONTROL_TARGET unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_actuator_control_target_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_actuator_control_target_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_actuator_control_target_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_actuator_control_target_decode(fmav_actuator_control_target_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_actuator_control_target_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_actuator_control_target_get_field_group_mlx(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_actuator_control_target_get_field_controls_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_actuator_control_target_get_field_controls(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET  140

#define mavlink_actuator_control_target_t  fmav_actuator_control_target_t

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN  41
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_MIN_LEN  41
#define MAVLINK_MSG_ID_140_LEN  41
#define MAVLINK_MSG_ID_140_MIN_LEN  41

#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_CRC  181
#define MAVLINK_MSG_ID_140_CRC  181

#define MAVLINK_MSG_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_control_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t group_mlx, const float* controls)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_actuator_control_target_pack(
        msg, sysid, compid,
        time_usec, group_mlx, controls,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_actuator_control_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t group_mlx, const float* controls)
{
    return fmav_msg_actuator_control_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, group_mlx, controls,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_actuator_control_target_decode(const mavlink_message_t* msg, mavlink_actuator_control_target_t* payload)
{
    fmav_msg_actuator_control_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ACTUATOR_CONTROL_TARGET_H

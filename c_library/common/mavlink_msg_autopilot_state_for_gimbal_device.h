//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_state_for_gimbal_device_t {
    uint64_t time_boot_us;
    float q[4];
    uint32_t q_estimated_delay_us;
    float vx;
    float vy;
    float vz;
    uint32_t v_estimated_delay_us;
    float feed_forward_angular_velocity_z;
    uint16_t estimator_status;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t landed_state;
}) fmav_autopilot_state_for_gimbal_device_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE  286

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA  210

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FLAGS  3
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_COMPONENT_OFS  51

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FRAME_LEN_MAX  78

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TIME_BOOT_US_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_OFS  8
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_ESTIMATED_DELAY_US_OFS  24
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VX_OFS  28
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VY_OFS  32
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_VZ_OFS  36
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_V_ESTIMATED_DELAY_US_OFS  40
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_FEED_FORWARD_ANGULAR_VELOCITY_Z_OFS  44
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_ESTIMATOR_STATUS_OFS  48
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_TARGET_COMPONENT_OFS  51
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_LANDED_STATE_OFS  52


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t* _payload = (fmav_autopilot_state_for_gimbal_device_t*)msg->payload;

    _payload->time_boot_us = time_boot_us;
    _payload->q_estimated_delay_us = q_estimated_delay_us;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->v_estimated_delay_us = v_estimated_delay_us;
    _payload->feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload->estimator_status = estimator_status;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->landed_state = landed_state;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->q, _payload->q_estimated_delay_us, _payload->vx, _payload->vy, _payload->vz, _payload->v_estimated_delay_us, _payload->feed_forward_angular_velocity_z, _payload->estimator_status, _payload->landed_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t* _payload = (fmav_autopilot_state_for_gimbal_device_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_us = time_boot_us;
    _payload->q_estimated_delay_us = q_estimated_delay_us;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->v_estimated_delay_us = v_estimated_delay_us;
    _payload->feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload->estimator_status = estimator_status;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->landed_state = landed_state;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->time_boot_us, _payload->q, _payload->q_estimated_delay_us, _payload->vx, _payload->vy, _payload->vz, _payload->v_estimated_delay_us, _payload->feed_forward_angular_velocity_z, _payload->estimator_status, _payload->landed_state,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state,
    fmav_status_t* _status)
{
    fmav_autopilot_state_for_gimbal_device_t _payload;

    _payload.time_boot_us = time_boot_us;
    _payload.q_estimated_delay_us = q_estimated_delay_us;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.v_estimated_delay_us = v_estimated_delay_us;
    _payload.feed_forward_angular_velocity_z = feed_forward_angular_velocity_z;
    _payload.estimator_status = estimator_status;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.landed_state = landed_state;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_state_for_gimbal_device_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_autopilot_state_for_gimbal_device_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_autopilot_state_for_gimbal_device_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_state_for_gimbal_device_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_state_for_gimbal_device_decode(fmav_autopilot_state_for_gimbal_device_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_autopilot_state_for_gimbal_device_get_field_time_boot_us(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_state_for_gimbal_device_get_field_q_estimated_delay_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_autopilot_state_for_gimbal_device_get_field_v_estimated_delay_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_feed_forward_angular_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_state_for_gimbal_device_get_field_estimator_status(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_autopilot_state_for_gimbal_device_get_field_landed_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_autopilot_state_for_gimbal_device_get_field_q_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_autopilot_state_for_gimbal_device_get_field_q(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_NUM) return 0;
    return ((float*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE  286

#define mavlink_autopilot_state_for_gimbal_device_t  fmav_autopilot_state_for_gimbal_device_t

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_LEN  53
#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_MIN_LEN  53
#define MAVLINK_MSG_ID_286_LEN  53
#define MAVLINK_MSG_ID_286_MIN_LEN  53

#define MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRC  210
#define MAVLINK_MSG_ID_286_CRC  210

#define MAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_state_for_gimbal_device_pack(
        msg, sysid, compid,
        target_system, target_component, time_boot_us, q, q_estimated_delay_us, vx, vy, vz, v_estimated_delay_us, feed_forward_angular_velocity_z, estimator_status, landed_state,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_state_for_gimbal_device_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint64_t time_boot_us, const float* q, uint32_t q_estimated_delay_us, float vx, float vy, float vz, uint32_t v_estimated_delay_us, float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state)
{
    return fmav_msg_autopilot_state_for_gimbal_device_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, time_boot_us, q, q_estimated_delay_us, vx, vy, vz, v_estimated_delay_us, feed_forward_angular_velocity_z, estimator_status, landed_state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_state_for_gimbal_device_decode(const mavlink_message_t* msg, mavlink_autopilot_state_for_gimbal_device_t* payload)
{
    fmav_msg_autopilot_state_for_gimbal_device_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_H

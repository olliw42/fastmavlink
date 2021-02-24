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


#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MIN  53
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN  53
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_CRCEXTRA  210

#define FASTMAVLINK_MSG_ID_286_LEN_MIN  53
#define FASTMAVLINK_MSG_ID_286_LEN_MAX  53
#define FASTMAVLINK_MSG_ID_286_LEN  53
#define FASTMAVLINK_MSG_ID_286_CRCEXTRA  210

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FLAGS  3
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_TARGET_COMPONENT_OFS  51


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
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message AUTOPILOT_STATE_FOR_GIMBAL_DEVICE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_state_for_gimbal_device_decode(fmav_autopilot_state_for_gimbal_device_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

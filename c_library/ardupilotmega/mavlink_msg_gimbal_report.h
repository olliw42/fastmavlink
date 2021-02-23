//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_REPORT_H
#define FASTMAVLINK_MSG_GIMBAL_REPORT_H


//----------------------------------------
//-- Message GIMBAL_REPORT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_report_t {
    float delta_time;
    float delta_angle_x;
    float delta_angle_y;
    float delta_angle_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    float joint_roll;
    float joint_el;
    float joint_az;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_report_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_REPORT  200


#define FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA  134

#define FASTMAVLINK_MSG_ID_200_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_200_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_200_LEN  42
#define FASTMAVLINK_MSG_ID_200_CRCEXTRA  134



#define FASTMAVLINK_MSG_GIMBAL_REPORT_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_REPORT_TARGET_SYSTEM_OFS  40
#define FASTMAVLINK_MSG_GIMBAL_REPORT_TARGET_COMPONENT_OFS  41


//----------------------------------------
//-- Message GIMBAL_REPORT packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az,
    fmav_status_t* _status)
{
    fmav_gimbal_report_t* _payload = (fmav_gimbal_report_t*)msg->payload;

    _payload->delta_time = delta_time;
    _payload->delta_angle_x = delta_angle_x;
    _payload->delta_angle_y = delta_angle_y;
    _payload->delta_angle_z = delta_angle_z;
    _payload->delta_velocity_x = delta_velocity_x;
    _payload->delta_velocity_y = delta_velocity_y;
    _payload->delta_velocity_z = delta_velocity_z;
    _payload->joint_roll = joint_roll;
    _payload->joint_el = joint_el;
    _payload->joint_az = joint_az;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_REPORT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_report_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->delta_time, _payload->delta_angle_x, _payload->delta_angle_y, _payload->delta_angle_z, _payload->delta_velocity_x, _payload->delta_velocity_y, _payload->delta_velocity_z, _payload->joint_roll, _payload->joint_el, _payload->joint_az,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az,
    fmav_status_t* _status)
{
    fmav_gimbal_report_t* _payload = (fmav_gimbal_report_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->delta_time = delta_time;
    _payload->delta_angle_x = delta_angle_x;
    _payload->delta_angle_y = delta_angle_y;
    _payload->delta_angle_z = delta_angle_z;
    _payload->delta_velocity_x = delta_velocity_x;
    _payload->delta_velocity_y = delta_velocity_y;
    _payload->delta_velocity_z = delta_velocity_z;
    _payload->joint_roll = joint_roll;
    _payload->joint_el = joint_el;
    _payload->joint_az = joint_az;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_report_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->delta_time, _payload->delta_angle_x, _payload->delta_angle_y, _payload->delta_angle_z, _payload->delta_velocity_x, _payload->delta_velocity_y, _payload->delta_velocity_z, _payload->joint_roll, _payload->joint_el, _payload->joint_az,
        _status);
}


//----------------------------------------
//-- Message GIMBAL_REPORT unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_report_decode(fmav_gimbal_report_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_REPORT  200

#define mavlink_gimbal_report_t  fmav_gimbal_report_t

#define MAVLINK_MSG_ID_GIMBAL_REPORT_LEN  42
#define MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN  42
#define MAVLINK_MSG_ID_200_LEN  42
#define MAVLINK_MSG_ID_200_MIN_LEN  42

#define MAVLINK_MSG_ID_GIMBAL_REPORT_CRC  134
#define MAVLINK_MSG_ID_200_CRC  134




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_report_pack(
        msg, sysid, compid,
        target_system, target_component, delta_time, delta_angle_x, delta_angle_y, delta_angle_z, delta_velocity_x, delta_velocity_y, delta_velocity_z, joint_roll, joint_el, joint_az,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_report_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
    return fmav_msg_gimbal_report_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, delta_time, delta_angle_x, delta_angle_y, delta_angle_z, delta_velocity_x, delta_velocity_y, delta_velocity_z, joint_roll, joint_el, joint_az,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_report_decode(const mavlink_message_t* msg, mavlink_gimbal_report_t* payload)
{
    fmav_msg_gimbal_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_REPORT_H

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

// fields are ordered, as they appear on the wire
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

#define FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA  134

#define FASTMAVLINK_MSG_GIMBAL_REPORT_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_REPORT_TARGET_SYSTEM_OFS  40
#define FASTMAVLINK_MSG_GIMBAL_REPORT_TARGET_COMPONENT_OFS  41

#define FASTMAVLINK_MSG_GIMBAL_REPORT_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_TIME_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_ANGLE_X_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_ANGLE_Y_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_ANGLE_Z_OFS  12
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_VELOCITY_X_OFS  16
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_VELOCITY_Y_OFS  20
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_DELTA_VELOCITY_Z_OFS  24
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_JOINT_ROLL_OFS  28
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_JOINT_EL_OFS  32
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_JOINT_AZ_OFS  36
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_TARGET_SYSTEM_OFS  40
#define FASTMAVLINK_MSG_GIMBAL_REPORT_FIELD_TARGET_COMPONENT_OFS  41


//----------------------------------------
//-- Message GIMBAL_REPORT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az,
    fmav_status_t* _status)
{
    fmav_gimbal_report_t* _payload = (fmav_gimbal_report_t*)_msg->payload;

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


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_REPORT;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_report_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->delta_time, _payload->delta_angle_x, _payload->delta_angle_y, _payload->delta_angle_z, _payload->delta_velocity_x, _payload->delta_velocity_y, _payload->delta_velocity_z, _payload->joint_roll, _payload->joint_el, _payload->joint_az,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az,
    fmav_status_t* _status)
{
    fmav_gimbal_report_t* _payload = (fmav_gimbal_report_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_REPORT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_report_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->delta_time, _payload->delta_angle_x, _payload->delta_angle_y, _payload->delta_angle_z, _payload->delta_velocity_x, _payload->delta_velocity_y, _payload->delta_velocity_z, _payload->joint_roll, _payload->joint_el, _payload->joint_az,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az,
    fmav_status_t* _status)
{
    fmav_gimbal_report_t _payload;

    _payload.delta_time = delta_time;
    _payload.delta_angle_x = delta_angle_x;
    _payload.delta_angle_y = delta_angle_y;
    _payload.delta_angle_z = delta_angle_z;
    _payload.delta_velocity_x = delta_velocity_x;
    _payload.delta_velocity_y = delta_velocity_y;
    _payload.delta_velocity_z = delta_velocity_z;
    _payload.joint_roll = joint_roll;
    _payload.joint_el = joint_el;
    _payload.joint_az = joint_az;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_REPORT,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_REPORT,
        FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_REPORT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_report_decode(fmav_gimbal_report_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_REPORT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_time(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_angle_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_angle_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_angle_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_velocity_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_velocity_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_delta_velocity_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_joint_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_joint_el(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gimbal_report_get_field_joint_az(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_report_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_report_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint8_t));
    return r;
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
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_report_pack(
        _msg, sysid, compid,
        target_system, target_component, delta_time, delta_angle_x, delta_angle_y, delta_angle_z, delta_velocity_x, delta_velocity_y, delta_velocity_z, joint_roll, joint_el, joint_az,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_report_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_report_t* _payload)
{
    return mavlink_msg_gimbal_report_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->delta_time, _payload->delta_angle_x, _payload->delta_angle_y, _payload->delta_angle_z, _payload->delta_velocity_x, _payload->delta_velocity_y, _payload->delta_velocity_z, _payload->joint_roll, _payload->joint_el, _payload->joint_az);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_report_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, float delta_time, float delta_angle_x, float delta_angle_y, float delta_angle_z, float delta_velocity_x, float delta_velocity_y, float delta_velocity_z, float joint_roll, float joint_el, float joint_az)
{
    return fmav_msg_gimbal_report_pack_to_frame_buf(
        (uint8_t*)_buf,
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

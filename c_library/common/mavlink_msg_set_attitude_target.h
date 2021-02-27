//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_attitude_target_t {
    uint32_t time_boot_ms;
    float q[4];
    float body_roll_rate;
    float body_pitch_rate;
    float body_yaw_rate;
    float thrust;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type_mask;
}) fmav_set_attitude_target_t;


#define FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET  82


#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MIN  39
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX  39
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN  39
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA  49

#define FASTMAVLINK_MSG_ID_82_LEN_MIN  39
#define FASTMAVLINK_MSG_ID_82_LEN_MAX  39
#define FASTMAVLINK_MSG_ID_82_LEN  39
#define FASTMAVLINK_MSG_ID_82_CRCEXTRA  49

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FLAGS  3
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_TARGET_SYSTEM_OFS  36
#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_TARGET_COMPONENT_OFS  37

#define FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_82_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_82_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t* _payload = (fmav_set_attitude_target_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->body_roll_rate = body_roll_rate;
    _payload->body_pitch_rate = body_pitch_rate;
    _payload->body_yaw_rate = body_yaw_rate;
    _payload->thrust = thrust;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type_mask = type_mask;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_attitude_target_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->type_mask, _payload->q, _payload->body_roll_rate, _payload->body_pitch_rate, _payload->body_yaw_rate, _payload->thrust,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t* _payload = (fmav_set_attitude_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->body_roll_rate = body_roll_rate;
    _payload->body_pitch_rate = body_pitch_rate;
    _payload->body_yaw_rate = body_yaw_rate;
    _payload->thrust = thrust;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type_mask = type_mask;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_attitude_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->type_mask, _payload->q, _payload->body_roll_rate, _payload->body_pitch_rate, _payload->body_yaw_rate, _payload->thrust,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust,
    fmav_status_t* _status)
{
    fmav_set_attitude_target_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.body_roll_rate = body_roll_rate;
    _payload.body_pitch_rate = body_pitch_rate;
    _payload.body_yaw_rate = body_yaw_rate;
    _payload.thrust = thrust;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.type_mask = type_mask;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_attitude_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_attitude_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_ATTITUDE_TARGET unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_attitude_target_decode(fmav_set_attitude_target_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET  82

#define mavlink_set_attitude_target_t  fmav_set_attitude_target_t

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_LEN  39
#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_MIN_LEN  39
#define MAVLINK_MSG_ID_82_LEN  39
#define MAVLINK_MSG_ID_82_MIN_LEN  39

#define MAVLINK_MSG_ID_SET_ATTITUDE_TARGET_CRC  49
#define MAVLINK_MSG_ID_82_CRC  49

#define MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_attitude_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_attitude_target_pack(
        msg, sysid, compid,
        time_boot_ms, target_system, target_component, type_mask, q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_attitude_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t type_mask, const float* q, float body_roll_rate, float body_pitch_rate, float body_yaw_rate, float thrust)
{
    return fmav_msg_set_attitude_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, target_system, target_component, type_mask, q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_attitude_target_decode(const mavlink_message_t* msg, mavlink_set_attitude_target_t* payload)
{
    fmav_msg_set_attitude_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_ATTITUDE_TARGET_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MANUAL_SETPOINT_H
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_H


//----------------------------------------
//-- Message MANUAL_SETPOINT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_manual_setpoint_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float thrust;
    uint8_t mode_switch;
    uint8_t manual_override_switch;
}) fmav_manual_setpoint_t;


#define FASTMAVLINK_MSG_ID_MANUAL_SETPOINT  81


#define FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MIN  22
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_CRCEXTRA  106

#define FASTMAVLINK_MSG_ID_81_LEN_MIN  22
#define FASTMAVLINK_MSG_ID_81_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_81_LEN  22
#define FASTMAVLINK_MSG_ID_81_CRCEXTRA  106



#define FASTMAVLINK_MSG_MANUAL_SETPOINT_FLAGS  0
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MANUAL_SETPOINT_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message MANUAL_SETPOINT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_setpoint_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch,
    fmav_status_t* _status)
{
    fmav_manual_setpoint_t* _payload = (fmav_manual_setpoint_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->thrust = thrust;
    _payload->mode_switch = mode_switch;
    _payload->manual_override_switch = manual_override_switch;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MANUAL_SETPOINT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MANUAL_SETPOINT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_setpoint_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_manual_setpoint_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_manual_setpoint_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->thrust, _payload->mode_switch, _payload->manual_override_switch,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_setpoint_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch,
    fmav_status_t* _status)
{
    fmav_manual_setpoint_t* _payload = (fmav_manual_setpoint_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->thrust = thrust;
    _payload->mode_switch = mode_switch;
    _payload->manual_override_switch = manual_override_switch;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MANUAL_SETPOINT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MANUAL_SETPOINT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MANUAL_SETPOINT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MANUAL_SETPOINT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_manual_setpoint_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_manual_setpoint_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_manual_setpoint_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->thrust, _payload->mode_switch, _payload->manual_override_switch,
        _status);
}


//----------------------------------------
//-- Message MANUAL_SETPOINT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_manual_setpoint_decode(fmav_manual_setpoint_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MANUAL_SETPOINT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MANUAL_SETPOINT  81

#define mavlink_manual_setpoint_t  fmav_manual_setpoint_t

#define MAVLINK_MSG_ID_MANUAL_SETPOINT_LEN  22
#define MAVLINK_MSG_ID_MANUAL_SETPOINT_MIN_LEN  22
#define MAVLINK_MSG_ID_81_LEN  22
#define MAVLINK_MSG_ID_81_MIN_LEN  22

#define MAVLINK_MSG_ID_MANUAL_SETPOINT_CRC  106
#define MAVLINK_MSG_ID_81_CRC  106




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_manual_setpoint_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_manual_setpoint_pack(
        msg, sysid, compid,
        time_boot_ms, roll, pitch, yaw, thrust, mode_switch, manual_override_switch,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_manual_setpoint_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust, uint8_t mode_switch, uint8_t manual_override_switch)
{
    return fmav_msg_manual_setpoint_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, roll, pitch, yaw, thrust, mode_switch, manual_override_switch,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_manual_setpoint_decode(const mavlink_message_t* msg, mavlink_manual_setpoint_t* payload)
{
    fmav_msg_manual_setpoint_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MANUAL_SETPOINT_H

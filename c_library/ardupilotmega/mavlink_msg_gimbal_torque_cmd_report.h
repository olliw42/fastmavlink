//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_H
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_H


//----------------------------------------
//-- Message GIMBAL_TORQUE_CMD_REPORT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_torque_cmd_report_t {
    int16_t rl_torque_cmd;
    int16_t el_torque_cmd;
    int16_t az_torque_cmd;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_gimbal_torque_cmd_report_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT  214

#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA  69

#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FLAGS  3
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_TARGET_COMPONENT_OFS  7

#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FIELD_RL_TORQUE_CMD_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FIELD_EL_TORQUE_CMD_OFS  2
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FIELD_AZ_TORQUE_CMD_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FIELD_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_FIELD_TARGET_COMPONENT_OFS  7


//----------------------------------------
//-- Message GIMBAL_TORQUE_CMD_REPORT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd,
    fmav_status_t* _status)
{
    fmav_gimbal_torque_cmd_report_t* _payload = (fmav_gimbal_torque_cmd_report_t*)_msg->payload;

    _payload->rl_torque_cmd = rl_torque_cmd;
    _payload->el_torque_cmd = el_torque_cmd;
    _payload->az_torque_cmd = az_torque_cmd;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_torque_cmd_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_torque_cmd_report_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->rl_torque_cmd, _payload->el_torque_cmd, _payload->az_torque_cmd,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd,
    fmav_status_t* _status)
{
    fmav_gimbal_torque_cmd_report_t* _payload = (fmav_gimbal_torque_cmd_report_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rl_torque_cmd = rl_torque_cmd;
    _payload->el_torque_cmd = el_torque_cmd;
    _payload->az_torque_cmd = az_torque_cmd;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_torque_cmd_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_torque_cmd_report_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->rl_torque_cmd, _payload->el_torque_cmd, _payload->az_torque_cmd,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd,
    fmav_status_t* _status)
{
    fmav_gimbal_torque_cmd_report_t _payload;

    _payload.rl_torque_cmd = rl_torque_cmd;
    _payload.el_torque_cmd = el_torque_cmd;
    _payload.az_torque_cmd = az_torque_cmd;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_torque_cmd_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_torque_cmd_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_TORQUE_CMD_REPORT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_torque_cmd_report_decode(fmav_gimbal_torque_cmd_report_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_gimbal_torque_cmd_report_get_field_rl_torque_cmd(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_gimbal_torque_cmd_report_get_field_el_torque_cmd(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_gimbal_torque_cmd_report_get_field_az_torque_cmd(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_torque_cmd_report_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_torque_cmd_report_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT  214

#define mavlink_gimbal_torque_cmd_report_t  fmav_gimbal_torque_cmd_report_t

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN  8
#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_MIN_LEN  8
#define MAVLINK_MSG_ID_214_LEN  8
#define MAVLINK_MSG_ID_214_MIN_LEN  8

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC  69
#define MAVLINK_MSG_ID_214_CRC  69




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_torque_cmd_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_torque_cmd_report_pack(
        _msg, sysid, compid,
        target_system, target_component, rl_torque_cmd, el_torque_cmd, az_torque_cmd,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_torque_cmd_report_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_gimbal_torque_cmd_report_t* _payload)
{
    return mavlink_msg_gimbal_torque_cmd_report_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->rl_torque_cmd, _payload->el_torque_cmd, _payload->az_torque_cmd);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_torque_cmd_report_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd)
{
    return fmav_msg_gimbal_torque_cmd_report_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, rl_torque_cmd, el_torque_cmd, az_torque_cmd,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_torque_cmd_report_decode(const mavlink_message_t* msg, mavlink_gimbal_torque_cmd_report_t* payload)
{
    fmav_msg_gimbal_torque_cmd_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_TORQUE_CMD_REPORT_H

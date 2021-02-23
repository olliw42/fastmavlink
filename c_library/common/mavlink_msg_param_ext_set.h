//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_EXT_SET_H
#define FASTMAVLINK_MSG_PARAM_EXT_SET_H


//----------------------------------------
//-- Message PARAM_EXT_SET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_ext_set_t {
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    char param_value[128];
    uint8_t param_type;
}) fmav_param_ext_set_t;


#define FASTMAVLINK_MSG_ID_PARAM_EXT_SET  323


#define FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MIN  147
#define FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX  147
#define FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN  147
#define FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA  78

#define FASTMAVLINK_MSG_ID_323_LEN_MIN  147
#define FASTMAVLINK_MSG_ID_323_LEN_MAX  147
#define FASTMAVLINK_MSG_ID_323_LEN  147
#define FASTMAVLINK_MSG_ID_323_CRCEXTRA  78

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN  16
#define FASTMAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN  128

#define FASTMAVLINK_MSG_PARAM_EXT_SET_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_EXT_SET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_EXT_SET_TARGET_COMPONENT_OFS  1


//----------------------------------------
//-- Message PARAM_EXT_SET packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_ext_set_t* _payload = (fmav_param_ext_set_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_EXT_SET;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_set_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type,
    fmav_status_t* _status)
{
    fmav_param_ext_set_t* _payload = (fmav_param_ext_set_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->param_type = param_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);
    memcpy(&(_payload->param_value), param_value, sizeof(char)*128);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_EXT_SET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_EXT_SET_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_ext_set_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_ext_set_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_ext_set_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_value, _payload->param_type,
        _status);
}


//----------------------------------------
//-- Message PARAM_EXT_SET unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_ext_set_decode(fmav_param_ext_set_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_EXT_SET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_EXT_SET  323

#define mavlink_param_ext_set_t  fmav_param_ext_set_t

#define MAVLINK_MSG_ID_PARAM_EXT_SET_LEN  147
#define MAVLINK_MSG_ID_PARAM_EXT_SET_MIN_LEN  147
#define MAVLINK_MSG_ID_323_LEN  147
#define MAVLINK_MSG_ID_323_MIN_LEN  147

#define MAVLINK_MSG_ID_PARAM_EXT_SET_CRC  78
#define MAVLINK_MSG_ID_323_CRC  78

#define MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN 16
#define MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_set_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_ext_set_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_ext_set_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, const char* param_value, uint8_t param_type)
{
    return fmav_msg_param_ext_set_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_value, param_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_ext_set_decode(const mavlink_message_t* msg, mavlink_param_ext_set_t* payload)
{
    fmav_msg_param_ext_set_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_EXT_SET_H

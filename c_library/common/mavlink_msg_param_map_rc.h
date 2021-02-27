//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_MAP_RC_H
#define FASTMAVLINK_MSG_PARAM_MAP_RC_H


//----------------------------------------
//-- Message PARAM_MAP_RC
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_map_rc_t {
    float param_value0;
    float scale;
    float param_value_min;
    float param_value_max;
    int16_t param_index;
    uint8_t target_system;
    uint8_t target_component;
    char param_id[16];
    uint8_t parameter_rc_channel_index;
}) fmav_param_map_rc_t;


#define FASTMAVLINK_MSG_ID_PARAM_MAP_RC  50


#define FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MIN  37
#define FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN  37
#define FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA  78

#define FASTMAVLINK_MSG_ID_50_LEN_MIN  37
#define FASTMAVLINK_MSG_ID_50_LEN_MAX  37
#define FASTMAVLINK_MSG_ID_50_LEN  37
#define FASTMAVLINK_MSG_ID_50_CRCEXTRA  78

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_LEN  16

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FLAGS  3
#define FASTMAVLINK_MSG_PARAM_MAP_RC_TARGET_SYSTEM_OFS  18
#define FASTMAVLINK_MSG_PARAM_MAP_RC_TARGET_COMPONENT_OFS  19

#define FASTMAVLINK_MSG_PARAM_MAP_RC_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_50_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_50_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message PARAM_MAP_RC packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t* _payload = (fmav_param_map_rc_t*)msg->payload;

    _payload->param_value0 = param_value0;
    _payload->scale = scale;
    _payload->param_value_min = param_value_min;
    _payload->param_value_max = param_value_max;
    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PARAM_MAP_RC;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_map_rc_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index, _payload->parameter_rc_channel_index, _payload->param_value0, _payload->scale, _payload->param_value_min, _payload->param_value_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t* _payload = (fmav_param_map_rc_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_value0 = param_value0;
    _payload->scale = scale;
    _payload->param_value_min = param_value_min;
    _payload->param_value_max = param_value_max;
    _payload->param_index = param_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_MAP_RC >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_map_rc_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->param_id, _payload->param_index, _payload->parameter_rc_channel_index, _payload->param_value0, _payload->scale, _payload->param_value_min, _payload->param_value_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max,
    fmav_status_t* _status)
{
    fmav_param_map_rc_t _payload;

    _payload.param_value0 = param_value0;
    _payload.scale = scale;
    _payload.param_value_min = param_value_min;
    _payload.param_value_max = param_value_max;
    _payload.param_index = param_index;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.parameter_rc_channel_index = parameter_rc_channel_index;
    memcpy(&(_payload.param_id), param_id, sizeof(char)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_MAP_RC,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_map_rc_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_map_rc_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_MAP_RC,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_MAP_RC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_MAP_RC unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_map_rc_decode(fmav_param_map_rc_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PARAM_MAP_RC_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_MAP_RC  50

#define mavlink_param_map_rc_t  fmav_param_map_rc_t

#define MAVLINK_MSG_ID_PARAM_MAP_RC_LEN  37
#define MAVLINK_MSG_ID_PARAM_MAP_RC_MIN_LEN  37
#define MAVLINK_MSG_ID_50_LEN  37
#define MAVLINK_MSG_ID_50_MIN_LEN  37

#define MAVLINK_MSG_ID_PARAM_MAP_RC_CRC  78
#define MAVLINK_MSG_ID_50_CRC  78

#define MAVLINK_MSG_PARAM_MAP_RC_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_map_rc_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_map_rc_pack(
        msg, sysid, compid,
        target_system, target_component, param_id, param_index, parameter_rc_channel_index, param_value0, scale, param_value_min, param_value_max,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_map_rc_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const char* param_id, int16_t param_index, uint8_t parameter_rc_channel_index, float param_value0, float scale, float param_value_min, float param_value_max)
{
    return fmav_msg_param_map_rc_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, param_id, param_index, parameter_rc_channel_index, param_value0, scale, param_value_min, param_value_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_map_rc_decode(const mavlink_message_t* msg, mavlink_param_map_rc_t* payload)
{
    fmav_msg_param_map_rc_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_MAP_RC_H

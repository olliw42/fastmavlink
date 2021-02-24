//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OSD_PARAM_CONFIG_H
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_H


//----------------------------------------
//-- Message OSD_PARAM_CONFIG
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_osd_param_config_t {
    uint32_t request_id;
    float min_value;
    float max_value;
    float increment;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t osd_screen;
    uint8_t osd_index;
    char param_id[16];
    uint8_t config_type;
}) fmav_osd_param_config_t;


#define FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG  11033


#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MIN  37
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN  37
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_CRCEXTRA  195

#define FASTMAVLINK_MSG_ID_11033_LEN_MIN  37
#define FASTMAVLINK_MSG_ID_11033_LEN_MAX  37
#define FASTMAVLINK_MSG_ID_11033_LEN  37
#define FASTMAVLINK_MSG_ID_11033_CRCEXTRA  195

#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_FIELD_PARAM_ID_LEN  16

#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_FLAGS  3
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_TARGET_SYSTEM_OFS  16
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_TARGET_COMPONENT_OFS  17


//----------------------------------------
//-- Message OSD_PARAM_CONFIG packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char* param_id, uint8_t config_type, float min_value, float max_value, float increment,
    fmav_status_t* _status)
{
    fmav_osd_param_config_t* _payload = (fmav_osd_param_config_t*)msg->payload;

    _payload->request_id = request_id;
    _payload->min_value = min_value;
    _payload->max_value = max_value;
    _payload->increment = increment;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->osd_screen = osd_screen;
    _payload->osd_index = osd_index;
    _payload->config_type = config_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_osd_param_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_osd_param_config_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->osd_screen, _payload->osd_index, _payload->param_id, _payload->config_type, _payload->min_value, _payload->max_value, _payload->increment,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char* param_id, uint8_t config_type, float min_value, float max_value, float increment,
    fmav_status_t* _status)
{
    fmav_osd_param_config_t* _payload = (fmav_osd_param_config_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->request_id = request_id;
    _payload->min_value = min_value;
    _payload->max_value = max_value;
    _payload->increment = increment;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->osd_screen = osd_screen;
    _payload->osd_index = osd_index;
    _payload->config_type = config_type;
    memcpy(&(_payload->param_id), param_id, sizeof(char)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_osd_param_config_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_osd_param_config_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->osd_screen, _payload->osd_index, _payload->param_id, _payload->config_type, _payload->min_value, _payload->max_value, _payload->increment,
        _status);
}


//----------------------------------------
//-- Message OSD_PARAM_CONFIG unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_osd_param_config_decode(fmav_osd_param_config_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_OSD_PARAM_CONFIG_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG  11033

#define mavlink_osd_param_config_t  fmav_osd_param_config_t

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_LEN  37
#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN  37
#define MAVLINK_MSG_ID_11033_LEN  37
#define MAVLINK_MSG_ID_11033_MIN_LEN  37

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_CRC  195
#define MAVLINK_MSG_ID_11033_CRC  195

#define MAVLINK_MSG_OSD_PARAM_CONFIG_FIELD_PARAM_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_osd_param_config_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char* param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_osd_param_config_pack(
        msg, sysid, compid,
        target_system, target_component, request_id, osd_screen, osd_index, param_id, config_type, min_value, max_value, increment,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_osd_param_config_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t osd_screen, uint8_t osd_index, const char* param_id, uint8_t config_type, float min_value, float max_value, float increment)
{
    return fmav_msg_osd_param_config_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, request_id, osd_screen, osd_index, param_id, config_type, min_value, max_value, increment,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_osd_param_config_decode(const mavlink_message_t* msg, mavlink_osd_param_config_t* payload)
{
    fmav_msg_osd_param_config_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OSD_PARAM_CONFIG_H

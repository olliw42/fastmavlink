//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DIGICAM_CONTROL_H
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_H


//----------------------------------------
//-- Message DIGICAM_CONTROL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_digicam_control_t {
    float extra_value;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t session;
    uint8_t zoom_pos;
    int8_t zoom_step;
    uint8_t focus_lock;
    uint8_t shot;
    uint8_t command_id;
    uint8_t extra_param;
}) fmav_digicam_control_t;


#define FASTMAVLINK_MSG_ID_DIGICAM_CONTROL  155


#define FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MIN  13
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN  13
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_CRCEXTRA  22

#define FASTMAVLINK_MSG_ID_155_LEN_MIN  13
#define FASTMAVLINK_MSG_ID_155_LEN_MAX  13
#define FASTMAVLINK_MSG_ID_155_LEN  13
#define FASTMAVLINK_MSG_ID_155_CRCEXTRA  22



#define FASTMAVLINK_MSG_DIGICAM_CONTROL_FLAGS  3
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DIGICAM_CONTROL_TARGET_COMPONENT_OFS  5


//----------------------------------------
//-- Message DIGICAM_CONTROL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_control_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t session, uint8_t zoom_pos, int8_t zoom_step, uint8_t focus_lock, uint8_t shot, uint8_t command_id, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_control_t* _payload = (fmav_digicam_control_t*)msg->payload;

    _payload->extra_value = extra_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->session = session;
    _payload->zoom_pos = zoom_pos;
    _payload->zoom_step = zoom_step;
    _payload->focus_lock = focus_lock;
    _payload->shot = shot;
    _payload->command_id = command_id;
    _payload->extra_param = extra_param;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DIGICAM_CONTROL;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_DIGICAM_CONTROL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_control_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_control_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->session, _payload->zoom_pos, _payload->zoom_step, _payload->focus_lock, _payload->shot, _payload->command_id, _payload->extra_param, _payload->extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_control_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t session, uint8_t zoom_pos, int8_t zoom_step, uint8_t focus_lock, uint8_t shot, uint8_t command_id, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_control_t* _payload = (fmav_digicam_control_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->extra_value = extra_value;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->session = session;
    _payload->zoom_pos = zoom_pos;
    _payload->zoom_step = zoom_step;
    _payload->focus_lock = focus_lock;
    _payload->shot = shot;
    _payload->command_id = command_id;
    _payload->extra_param = extra_param;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DIGICAM_CONTROL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONTROL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DIGICAM_CONTROL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_control_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_control_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->session, _payload->zoom_pos, _payload->zoom_step, _payload->focus_lock, _payload->shot, _payload->command_id, _payload->extra_param, _payload->extra_value,
        _status);
}


//----------------------------------------
//-- Message DIGICAM_CONTROL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_digicam_control_decode(fmav_digicam_control_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DIGICAM_CONTROL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DIGICAM_CONTROL  155

#define mavlink_digicam_control_t  fmav_digicam_control_t

#define MAVLINK_MSG_ID_DIGICAM_CONTROL_LEN  13
#define MAVLINK_MSG_ID_DIGICAM_CONTROL_MIN_LEN  13
#define MAVLINK_MSG_ID_155_LEN  13
#define MAVLINK_MSG_ID_155_MIN_LEN  13

#define MAVLINK_MSG_ID_DIGICAM_CONTROL_CRC  22
#define MAVLINK_MSG_ID_155_CRC  22




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t session, uint8_t zoom_pos, int8_t zoom_step, uint8_t focus_lock, uint8_t shot, uint8_t command_id, uint8_t extra_param, float extra_value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_digicam_control_pack(
        msg, sysid, compid,
        target_system, target_component, session, zoom_pos, zoom_step, focus_lock, shot, command_id, extra_param, extra_value,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_control_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t session, uint8_t zoom_pos, int8_t zoom_step, uint8_t focus_lock, uint8_t shot, uint8_t command_id, uint8_t extra_param, float extra_value)
{
    return fmav_msg_digicam_control_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, session, zoom_pos, zoom_step, focus_lock, shot, command_id, extra_param, extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_digicam_control_decode(const mavlink_message_t* msg, mavlink_digicam_control_t* payload)
{
    fmav_msg_digicam_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DIGICAM_CONTROL_H

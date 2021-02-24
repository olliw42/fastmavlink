//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DIGICAM_CONFIGURE_H
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_H


//----------------------------------------
//-- Message DIGICAM_CONFIGURE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_digicam_configure_t {
    float extra_value;
    uint16_t shutter_speed;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t mode;
    uint8_t aperture;
    uint8_t iso;
    uint8_t exposure_type;
    uint8_t command_id;
    uint8_t engine_cut_off;
    uint8_t extra_param;
}) fmav_digicam_configure_t;


#define FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE  154


#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MIN  15
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX  15
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN  15
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA  84

#define FASTMAVLINK_MSG_ID_154_LEN_MIN  15
#define FASTMAVLINK_MSG_ID_154_LEN_MAX  15
#define FASTMAVLINK_MSG_ID_154_LEN  15
#define FASTMAVLINK_MSG_ID_154_CRCEXTRA  84



#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FLAGS  3
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_TARGET_COMPONENT_OFS  7


//----------------------------------------
//-- Message DIGICAM_CONFIGURE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_configure_t* _payload = (fmav_digicam_configure_t*)msg->payload;

    _payload->extra_value = extra_value;
    _payload->shutter_speed = shutter_speed;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mode = mode;
    _payload->aperture = aperture;
    _payload->iso = iso;
    _payload->exposure_type = exposure_type;
    _payload->command_id = command_id;
    _payload->engine_cut_off = engine_cut_off;
    _payload->extra_param = extra_param;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_configure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_configure_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mode, _payload->shutter_speed, _payload->aperture, _payload->iso, _payload->exposure_type, _payload->command_id, _payload->engine_cut_off, _payload->extra_param, _payload->extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_configure_t* _payload = (fmav_digicam_configure_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->extra_value = extra_value;
    _payload->shutter_speed = shutter_speed;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mode = mode;
    _payload->aperture = aperture;
    _payload->iso = iso;
    _payload->exposure_type = exposure_type;
    _payload->command_id = command_id;
    _payload->engine_cut_off = engine_cut_off;
    _payload->extra_param = extra_param;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_configure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_configure_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mode, _payload->shutter_speed, _payload->aperture, _payload->iso, _payload->exposure_type, _payload->command_id, _payload->engine_cut_off, _payload->extra_param, _payload->extra_value,
        _status);
}


//----------------------------------------
//-- Message DIGICAM_CONFIGURE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_digicam_configure_decode(fmav_digicam_configure_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE  154

#define mavlink_digicam_configure_t  fmav_digicam_configure_t

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN  15
#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN  15
#define MAVLINK_MSG_ID_154_LEN  15
#define MAVLINK_MSG_ID_154_MIN_LEN  15

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC  84
#define MAVLINK_MSG_ID_154_CRC  84




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_configure_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_digicam_configure_pack(
        msg, sysid, compid,
        target_system, target_component, mode, shutter_speed, aperture, iso, exposure_type, command_id, engine_cut_off, extra_param, extra_value,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_configure_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
    return fmav_msg_digicam_configure_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, mode, shutter_speed, aperture, iso, exposure_type, command_id, engine_cut_off, extra_param, extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_digicam_configure_decode(const mavlink_message_t* msg, mavlink_digicam_configure_t* payload)
{
    fmav_msg_digicam_configure_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DIGICAM_CONFIGURE_H

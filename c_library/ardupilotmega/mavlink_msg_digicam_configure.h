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

// fields are ordered, as they appear on the wire
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

#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX  15
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA  84

#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FLAGS  3
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_TARGET_COMPONENT_OFS  7

#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FRAME_LEN_MAX  40



#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_EXTRA_VALUE_OFS  0
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_SHUTTER_SPEED_OFS  4
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_TARGET_COMPONENT_OFS  7
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_MODE_OFS  8
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_APERTURE_OFS  9
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_ISO_OFS  10
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_EXPOSURE_TYPE_OFS  11
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_COMMAND_ID_OFS  12
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_ENGINE_CUT_OFF_OFS  13
#define FASTMAVLINK_MSG_DIGICAM_CONFIGURE_FIELD_EXTRA_PARAM_OFS  14


//----------------------------------------
//-- Message DIGICAM_CONFIGURE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_configure_t* _payload = (fmav_digicam_configure_t*)_msg->payload;

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


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_configure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_configure_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mode, _payload->shutter_speed, _payload->aperture, _payload->iso, _payload->exposure_type, _payload->command_id, _payload->engine_cut_off, _payload->extra_param, _payload->extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_configure_t* _payload = (fmav_digicam_configure_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_configure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_digicam_configure_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mode, _payload->shutter_speed, _payload->aperture, _payload->iso, _payload->exposure_type, _payload->command_id, _payload->engine_cut_off, _payload->extra_param, _payload->extra_value,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value,
    fmav_status_t* _status)
{
    fmav_digicam_configure_t _payload;

    _payload.extra_value = extra_value;
    _payload.shutter_speed = shutter_speed;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.mode = mode;
    _payload.aperture = aperture;
    _payload.iso = iso;
    _payload.exposure_type = exposure_type;
    _payload.command_id = command_id;
    _payload.engine_cut_off = engine_cut_off;
    _payload.extra_param = extra_param;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_digicam_configure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DIGICAM_CONFIGURE,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DIGICAM_CONFIGURE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DIGICAM_CONFIGURE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_digicam_configure_decode(fmav_digicam_configure_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DIGICAM_CONFIGURE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_digicam_configure_get_field_extra_value(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_digicam_configure_get_field_shutter_speed(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_aperture(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_iso(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_exposure_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_command_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_engine_cut_off(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_digicam_configure_get_field_extra_param(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
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
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_digicam_configure_pack(
        _msg, sysid, compid,
        target_system, target_component, mode, shutter_speed, aperture, iso, exposure_type, command_id, engine_cut_off, extra_param, extra_value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_configure_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_digicam_configure_t* _payload)
{
    return mavlink_msg_digicam_configure_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->mode, _payload->shutter_speed, _payload->aperture, _payload->iso, _payload->exposure_type, _payload->command_id, _payload->engine_cut_off, _payload->extra_param, _payload->extra_value);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_digicam_configure_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
    return fmav_msg_digicam_configure_pack_to_frame_buf(
        (uint8_t*)_buf,
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

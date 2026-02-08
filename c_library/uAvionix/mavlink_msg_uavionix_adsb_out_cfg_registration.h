//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG_REGISTRATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_out_cfg_registration_t {
    char registration[9];
}) fmav_uavionix_adsb_out_cfg_registration_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION  10004

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRCEXTRA  133

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FRAME_LEN_MAX  34

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_NUM  9 // number of elements in array
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_LEN  9 // length of array = number of bytes

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_OFS  0


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG_REGISTRATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const char* registration,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_registration_t* _payload = (fmav_uavionix_adsb_out_cfg_registration_t*)_msg->payload;


    memcpy(&(_payload->registration), registration, sizeof(char)*9);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_registration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_registration_pack(
        _msg, sysid, compid,
        _payload->registration,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const char* registration,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_registration_t* _payload = (fmav_uavionix_adsb_out_cfg_registration_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->registration), registration, sizeof(char)*9);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_registration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_registration_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->registration,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const char* registration,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_registration_t _payload;


    memcpy(&(_payload.registration), registration, sizeof(char)*9);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_registration_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_registration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG_REGISTRATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_cfg_registration_decode(fmav_uavionix_adsb_out_cfg_registration_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_PAYLOAD_LEN_MAX);
#endif
}





FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_uavionix_adsb_out_cfg_registration_get_field_registration_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_uavionix_adsb_out_cfg_registration_get_field_registration(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_NUM) return 0;
    return ((char*)&(msg->payload[0]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION  10004

#define mavlink_uavionix_adsb_out_cfg_registration_t  fmav_uavionix_adsb_out_cfg_registration_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_LEN  9
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_MIN_LEN  9
#define MAVLINK_MSG_ID_10004_LEN  9
#define MAVLINK_MSG_ID_10004_MIN_LEN  9

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_CRC  133
#define MAVLINK_MSG_ID_10004_CRC  133

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_FIELD_REGISTRATION_LEN 9


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const char* registration)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_cfg_registration_pack(
        _msg, sysid, compid,
        registration,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_out_cfg_registration_t* _payload)
{
    return mavlink_msg_uavionix_adsb_out_cfg_registration_pack(
        sysid,
        compid,
        _msg,
        _payload->registration);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_registration_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const char* registration)
{
    return fmav_msg_uavionix_adsb_out_cfg_registration_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        registration,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_out_cfg_registration_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_cfg_registration_t* payload)
{
    fmav_msg_uavionix_adsb_out_cfg_registration_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_REGISTRATION_H

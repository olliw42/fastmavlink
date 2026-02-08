//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CONTROL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_out_control_t {
    int32_t baroAltMSL;
    uint16_t squawk;
    uint8_t state;
    uint8_t emergencyStatus;
    char flight_id[8];
    uint8_t x_bit;
}) fmav_uavionix_adsb_out_control_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL  10007

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_CRCEXTRA  71

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FRAME_LEN_MAX  42

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_BAROALTMSL_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_SQUAWK_OFS  4
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_STATE_OFS  6
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_EMERGENCYSTATUS_OFS  7
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_OFS  8
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_X_BIT_OFS  16


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CONTROL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char* flight_id, uint8_t x_bit,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_control_t* _payload = (fmav_uavionix_adsb_out_control_t*)_msg->payload;

    _payload->baroAltMSL = baroAltMSL;
    _payload->squawk = squawk;
    _payload->state = state;
    _payload->emergencyStatus = emergencyStatus;
    _payload->x_bit = x_bit;
    memcpy(&(_payload->flight_id), flight_id, sizeof(char)*8);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_control_pack(
        _msg, sysid, compid,
        _payload->state, _payload->baroAltMSL, _payload->squawk, _payload->emergencyStatus, _payload->flight_id, _payload->x_bit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char* flight_id, uint8_t x_bit,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_control_t* _payload = (fmav_uavionix_adsb_out_control_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->baroAltMSL = baroAltMSL;
    _payload->squawk = squawk;
    _payload->state = state;
    _payload->emergencyStatus = emergencyStatus;
    _payload->x_bit = x_bit;
    memcpy(&(_payload->flight_id), flight_id, sizeof(char)*8);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_control_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->state, _payload->baroAltMSL, _payload->squawk, _payload->emergencyStatus, _payload->flight_id, _payload->x_bit,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char* flight_id, uint8_t x_bit,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_control_t _payload;

    _payload.baroAltMSL = baroAltMSL;
    _payload.squawk = squawk;
    _payload.state = state;
    _payload.emergencyStatus = emergencyStatus;
    _payload.x_bit = x_bit;
    memcpy(&(_payload.flight_id), flight_id, sizeof(char)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_control_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CONTROL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_control_decode(fmav_uavionix_adsb_out_control_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_uavionix_adsb_out_control_get_field_baroAltMSL(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_control_get_field_squawk(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_control_get_field_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_control_get_field_emergencyStatus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_control_get_field_x_bit(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_uavionix_adsb_out_control_get_field_flight_id_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_uavionix_adsb_out_control_get_field_flight_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_NUM) return 0;
    return ((char*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL  10007

#define mavlink_uavionix_adsb_out_control_t  fmav_uavionix_adsb_out_control_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN  17
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN  17
#define MAVLINK_MSG_ID_10007_LEN  17
#define MAVLINK_MSG_ID_10007_MIN_LEN  17

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC  71
#define MAVLINK_MSG_ID_10007_CRC  71

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_control_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char* flight_id, uint8_t x_bit)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_control_pack(
        _msg, sysid, compid,
        state, baroAltMSL, squawk, emergencyStatus, flight_id, x_bit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_control_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_out_control_t* _payload)
{
    return mavlink_msg_uavionix_adsb_out_control_pack(
        sysid,
        compid,
        _msg,
        _payload->state, _payload->baroAltMSL, _payload->squawk, _payload->emergencyStatus, _payload->flight_id, _payload->x_bit);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_control_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char* flight_id, uint8_t x_bit)
{
    return fmav_msg_uavionix_adsb_out_control_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        state, baroAltMSL, squawk, emergencyStatus, flight_id, x_bit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_out_control_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_control_t* payload)
{
    fmav_msg_uavionix_adsb_out_control_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_H

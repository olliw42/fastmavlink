//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RADIO_CALIBRATION_H
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_H


//----------------------------------------
//-- Message RADIO_CALIBRATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_radio_calibration_t {
    uint16_t aileron[3];
    uint16_t elevator[3];
    uint16_t rudder[3];
    uint16_t gyro[2];
    uint16_t pitch[5];
    uint16_t throttle[5];
}) fmav_radio_calibration_t;


#define FASTMAVLINK_MSG_ID_RADIO_CALIBRATION  221

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA  71

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FLAGS  0
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FRAME_LEN_MAX  67

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN  10 // length of array = number of bytes
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_NUM  5 // number of elements in array
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN  10 // length of array = number of bytes

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_OFS  0
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_OFS  6
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_OFS  12
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_OFS  18
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_OFS  22
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_OFS  32


//----------------------------------------
//-- Message RADIO_CALIBRATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle,
    fmav_status_t* _status)
{
    fmav_radio_calibration_t* _payload = (fmav_radio_calibration_t*)_msg->payload;


    memcpy(&(_payload->aileron), aileron, sizeof(uint16_t)*3);
    memcpy(&(_payload->elevator), elevator, sizeof(uint16_t)*3);
    memcpy(&(_payload->rudder), rudder, sizeof(uint16_t)*3);
    memcpy(&(_payload->gyro), gyro, sizeof(uint16_t)*2);
    memcpy(&(_payload->pitch), pitch, sizeof(uint16_t)*5);
    memcpy(&(_payload->throttle), throttle, sizeof(uint16_t)*5);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RADIO_CALIBRATION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_calibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_calibration_pack(
        _msg, sysid, compid,
        _payload->aileron, _payload->elevator, _payload->rudder, _payload->gyro, _payload->pitch, _payload->throttle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle,
    fmav_status_t* _status)
{
    fmav_radio_calibration_t* _payload = (fmav_radio_calibration_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->aileron), aileron, sizeof(uint16_t)*3);
    memcpy(&(_payload->elevator), elevator, sizeof(uint16_t)*3);
    memcpy(&(_payload->rudder), rudder, sizeof(uint16_t)*3);
    memcpy(&(_payload->gyro), gyro, sizeof(uint16_t)*2);
    memcpy(&(_payload->pitch), pitch, sizeof(uint16_t)*5);
    memcpy(&(_payload->throttle), throttle, sizeof(uint16_t)*5);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_calibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_calibration_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->aileron, _payload->elevator, _payload->rudder, _payload->gyro, _payload->pitch, _payload->throttle,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle,
    fmav_status_t* _status)
{
    fmav_radio_calibration_t _payload;


    memcpy(&(_payload.aileron), aileron, sizeof(uint16_t)*3);
    memcpy(&(_payload.elevator), elevator, sizeof(uint16_t)*3);
    memcpy(&(_payload.rudder), rudder, sizeof(uint16_t)*3);
    memcpy(&(_payload.gyro), gyro, sizeof(uint16_t)*2);
    memcpy(&(_payload.pitch), pitch, sizeof(uint16_t)*5);
    memcpy(&(_payload.throttle), throttle, sizeof(uint16_t)*5);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RADIO_CALIBRATION,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_calibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RADIO_CALIBRATION,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RADIO_CALIBRATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_calibration_decode(fmav_radio_calibration_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX);
#endif
}





FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_aileron_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_aileron(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_NUM) return 0;
    return ((uint16_t*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_elevator_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_elevator(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_NUM) return 0;
    return ((uint16_t*)&(msg->payload[6]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_rudder_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[12]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_rudder(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_NUM) return 0;
    return ((uint16_t*)&(msg->payload[12]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_gyro_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[18]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_gyro(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_NUM) return 0;
    return ((uint16_t*)&(msg->payload[18]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_pitch_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[22]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_pitch(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_NUM) return 0;
    return ((uint16_t*)&(msg->payload[22]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_radio_calibration_get_field_throttle_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_get_field_throttle(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_NUM) return 0;
    return ((uint16_t*)&(msg->payload[32]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RADIO_CALIBRATION  221

#define mavlink_radio_calibration_t  fmav_radio_calibration_t

#define MAVLINK_MSG_ID_RADIO_CALIBRATION_LEN  42
#define MAVLINK_MSG_ID_RADIO_CALIBRATION_MIN_LEN  42
#define MAVLINK_MSG_ID_221_LEN  42
#define MAVLINK_MSG_ID_221_MIN_LEN  42

#define MAVLINK_MSG_ID_RADIO_CALIBRATION_CRC  71
#define MAVLINK_MSG_ID_221_CRC  71

#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN 2
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN 5
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN 5


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_calibration_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_calibration_pack(
        _msg, sysid, compid,
        aileron, elevator, rudder, gyro, pitch, throttle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_calibration_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_radio_calibration_t* _payload)
{
    return mavlink_msg_radio_calibration_pack(
        sysid,
        compid,
        _msg,
        _payload->aileron, _payload->elevator, _payload->rudder, _payload->gyro, _payload->pitch, _payload->throttle);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_calibration_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle)
{
    return fmav_msg_radio_calibration_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        aileron, elevator, rudder, gyro, pitch, throttle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_radio_calibration_decode(const mavlink_message_t* msg, mavlink_radio_calibration_t* payload)
{
    fmav_msg_radio_calibration_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RADIO_CALIBRATION_H

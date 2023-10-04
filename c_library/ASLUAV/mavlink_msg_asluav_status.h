//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ASLUAV_STATUS_H
#define FASTMAVLINK_MSG_ASLUAV_STATUS_H


//----------------------------------------
//-- Message ASLUAV_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_asluav_status_t {
    float Motor_rpm;
    uint8_t LED_status;
    uint8_t SATCOM_status;
    uint8_t Servo_status[8];
}) fmav_asluav_status_t;


#define FASTMAVLINK_MSG_ID_ASLUAV_STATUS  8006

#define FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_ASLUAV_STATUS_CRCEXTRA  97

#define FASTMAVLINK_MSG_ASLUAV_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ASLUAV_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ASLUAV_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ASLUAV_STATUS_FRAME_LEN_MAX  39

#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_MOTOR_RPM_OFS  0
#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_LED_STATUS_OFS  4
#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_SATCOM_STATUS_OFS  5
#define FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_OFS  6


//----------------------------------------
//-- Message ASLUAV_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t LED_status, uint8_t SATCOM_status, const uint8_t* Servo_status, float Motor_rpm,
    fmav_status_t* _status)
{
    fmav_asluav_status_t* _payload = (fmav_asluav_status_t*)_msg->payload;

    _payload->Motor_rpm = Motor_rpm;
    _payload->LED_status = LED_status;
    _payload->SATCOM_status = SATCOM_status;
    memcpy(&(_payload->Servo_status), Servo_status, sizeof(uint8_t)*8);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ASLUAV_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ASLUAV_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_asluav_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_asluav_status_pack(
        _msg, sysid, compid,
        _payload->LED_status, _payload->SATCOM_status, _payload->Servo_status, _payload->Motor_rpm,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t LED_status, uint8_t SATCOM_status, const uint8_t* Servo_status, float Motor_rpm,
    fmav_status_t* _status)
{
    fmav_asluav_status_t* _payload = (fmav_asluav_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->Motor_rpm = Motor_rpm;
    _payload->LED_status = LED_status;
    _payload->SATCOM_status = SATCOM_status;
    memcpy(&(_payload->Servo_status), Servo_status, sizeof(uint8_t)*8);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ASLUAV_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLUAV_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLUAV_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLUAV_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_asluav_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_asluav_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->LED_status, _payload->SATCOM_status, _payload->Servo_status, _payload->Motor_rpm,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t LED_status, uint8_t SATCOM_status, const uint8_t* Servo_status, float Motor_rpm,
    fmav_status_t* _status)
{
    fmav_asluav_status_t _payload;

    _payload.Motor_rpm = Motor_rpm;
    _payload.LED_status = LED_status;
    _payload.SATCOM_status = SATCOM_status;
    memcpy(&(_payload.Servo_status), Servo_status, sizeof(uint8_t)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ASLUAV_STATUS,
        FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLUAV_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_asluav_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_asluav_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ASLUAV_STATUS,
        FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLUAV_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ASLUAV_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_asluav_status_decode(fmav_asluav_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLUAV_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_asluav_status_get_field_Motor_rpm(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_asluav_status_get_field_LED_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_asluav_status_get_field_SATCOM_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_asluav_status_get_field_Servo_status_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_asluav_status_get_field_Servo_status(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_NUM) return 0;
    return ((uint8_t*)&(msg->payload[6]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ASLUAV_STATUS  8006

#define mavlink_asluav_status_t  fmav_asluav_status_t

#define MAVLINK_MSG_ID_ASLUAV_STATUS_LEN  14
#define MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN  14
#define MAVLINK_MSG_ID_8006_LEN  14
#define MAVLINK_MSG_ID_8006_MIN_LEN  14

#define MAVLINK_MSG_ID_ASLUAV_STATUS_CRC  97
#define MAVLINK_MSG_ID_8006_CRC  97

#define MAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asluav_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t LED_status, uint8_t SATCOM_status, const uint8_t* Servo_status, float Motor_rpm)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_asluav_status_pack(
        _msg, sysid, compid,
        LED_status, SATCOM_status, Servo_status, Motor_rpm,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asluav_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_asluav_status_t* _payload)
{
    return mavlink_msg_asluav_status_pack(
        sysid,
        compid,
        _msg,
        _payload->LED_status, _payload->SATCOM_status, _payload->Servo_status, _payload->Motor_rpm);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_asluav_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t LED_status, uint8_t SATCOM_status, const uint8_t* Servo_status, float Motor_rpm)
{
    return fmav_msg_asluav_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        LED_status, SATCOM_status, Servo_status, Motor_rpm,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_asluav_status_decode(const mavlink_message_t* msg, mavlink_asluav_status_t* payload)
{
    fmav_msg_asluav_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ASLUAV_STATUS_H

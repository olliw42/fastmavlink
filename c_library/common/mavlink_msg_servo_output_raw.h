//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_H
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_H


//----------------------------------------
//-- Message SERVO_OUTPUT_RAW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_servo_output_raw_t {
    uint32_t time_usec;
    uint16_t servo1_raw;
    uint16_t servo2_raw;
    uint16_t servo3_raw;
    uint16_t servo4_raw;
    uint16_t servo5_raw;
    uint16_t servo6_raw;
    uint16_t servo7_raw;
    uint16_t servo8_raw;
    uint8_t port;
    uint16_t servo9_raw;
    uint16_t servo10_raw;
    uint16_t servo11_raw;
    uint16_t servo12_raw;
    uint16_t servo13_raw;
    uint16_t servo14_raw;
    uint16_t servo15_raw;
    uint16_t servo16_raw;
}) fmav_servo_output_raw_t;


#define FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW  36


#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MIN  21
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN  37
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA  222

#define FASTMAVLINK_MSG_ID_36_LEN_MIN  21
#define FASTMAVLINK_MSG_ID_36_LEN_MAX  37
#define FASTMAVLINK_MSG_ID_36_LEN  37
#define FASTMAVLINK_MSG_ID_36_CRCEXTRA  222



#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_FLAGS  0
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_36_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_36_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message SERVO_OUTPUT_RAW packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw,
    fmav_status_t* _status)
{
    fmav_servo_output_raw_t* _payload = (fmav_servo_output_raw_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->servo1_raw = servo1_raw;
    _payload->servo2_raw = servo2_raw;
    _payload->servo3_raw = servo3_raw;
    _payload->servo4_raw = servo4_raw;
    _payload->servo5_raw = servo5_raw;
    _payload->servo6_raw = servo6_raw;
    _payload->servo7_raw = servo7_raw;
    _payload->servo8_raw = servo8_raw;
    _payload->port = port;
    _payload->servo9_raw = servo9_raw;
    _payload->servo10_raw = servo10_raw;
    _payload->servo11_raw = servo11_raw;
    _payload->servo12_raw = servo12_raw;
    _payload->servo13_raw = servo13_raw;
    _payload->servo14_raw = servo14_raw;
    _payload->servo15_raw = servo15_raw;
    _payload->servo16_raw = servo16_raw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_servo_output_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_servo_output_raw_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->port, _payload->servo1_raw, _payload->servo2_raw, _payload->servo3_raw, _payload->servo4_raw, _payload->servo5_raw, _payload->servo6_raw, _payload->servo7_raw, _payload->servo8_raw, _payload->servo9_raw, _payload->servo10_raw, _payload->servo11_raw, _payload->servo12_raw, _payload->servo13_raw, _payload->servo14_raw, _payload->servo15_raw, _payload->servo16_raw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw,
    fmav_status_t* _status)
{
    fmav_servo_output_raw_t* _payload = (fmav_servo_output_raw_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->servo1_raw = servo1_raw;
    _payload->servo2_raw = servo2_raw;
    _payload->servo3_raw = servo3_raw;
    _payload->servo4_raw = servo4_raw;
    _payload->servo5_raw = servo5_raw;
    _payload->servo6_raw = servo6_raw;
    _payload->servo7_raw = servo7_raw;
    _payload->servo8_raw = servo8_raw;
    _payload->port = port;
    _payload->servo9_raw = servo9_raw;
    _payload->servo10_raw = servo10_raw;
    _payload->servo11_raw = servo11_raw;
    _payload->servo12_raw = servo12_raw;
    _payload->servo13_raw = servo13_raw;
    _payload->servo14_raw = servo14_raw;
    _payload->servo15_raw = servo15_raw;
    _payload->servo16_raw = servo16_raw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_servo_output_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_servo_output_raw_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->port, _payload->servo1_raw, _payload->servo2_raw, _payload->servo3_raw, _payload->servo4_raw, _payload->servo5_raw, _payload->servo6_raw, _payload->servo7_raw, _payload->servo8_raw, _payload->servo9_raw, _payload->servo10_raw, _payload->servo11_raw, _payload->servo12_raw, _payload->servo13_raw, _payload->servo14_raw, _payload->servo15_raw, _payload->servo16_raw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw,
    fmav_status_t* _status)
{
    fmav_servo_output_raw_t _payload;

    _payload.time_usec = time_usec;
    _payload.servo1_raw = servo1_raw;
    _payload.servo2_raw = servo2_raw;
    _payload.servo3_raw = servo3_raw;
    _payload.servo4_raw = servo4_raw;
    _payload.servo5_raw = servo5_raw;
    _payload.servo6_raw = servo6_raw;
    _payload.servo7_raw = servo7_raw;
    _payload.servo8_raw = servo8_raw;
    _payload.port = port;
    _payload.servo9_raw = servo9_raw;
    _payload.servo10_raw = servo10_raw;
    _payload.servo11_raw = servo11_raw;
    _payload.servo12_raw = servo12_raw;
    _payload.servo13_raw = servo13_raw;
    _payload.servo14_raw = servo14_raw;
    _payload.servo15_raw = servo15_raw;
    _payload.servo16_raw = servo16_raw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_servo_output_raw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_servo_output_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SERVO_OUTPUT_RAW unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_servo_output_raw_decode(fmav_servo_output_raw_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW  36

#define mavlink_servo_output_raw_t  fmav_servo_output_raw_t

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN  37
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN  21
#define MAVLINK_MSG_ID_36_LEN  37
#define MAVLINK_MSG_ID_36_MIN_LEN  21

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC  222
#define MAVLINK_MSG_ID_36_CRC  222




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_servo_output_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_servo_output_raw_pack(
        msg, sysid, compid,
        time_usec, port, servo1_raw, servo2_raw, servo3_raw, servo4_raw, servo5_raw, servo6_raw, servo7_raw, servo8_raw, servo9_raw, servo10_raw, servo11_raw, servo12_raw, servo13_raw, servo14_raw, servo15_raw, servo16_raw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_servo_output_raw_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)
{
    return fmav_msg_servo_output_raw_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, port, servo1_raw, servo2_raw, servo3_raw, servo4_raw, servo5_raw, servo6_raw, servo7_raw, servo8_raw, servo9_raw, servo10_raw, servo11_raw, servo12_raw, servo13_raw, servo14_raw, servo15_raw, servo16_raw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* payload)
{
    fmav_msg_servo_output_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SERVO_OUTPUT_RAW_H

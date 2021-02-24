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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA  71

#define FASTMAVLINK_MSG_ID_221_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_221_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_221_LEN  42
#define FASTMAVLINK_MSG_ID_221_CRCEXTRA  71

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN  3
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN  3
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN  3
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN  2
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN  5
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN  5

#define FASTMAVLINK_MSG_RADIO_CALIBRATION_FLAGS  0
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RADIO_CALIBRATION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RADIO_CALIBRATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle,
    fmav_status_t* _status)
{
    fmav_radio_calibration_t* _payload = (fmav_radio_calibration_t*)msg->payload;


    memcpy(&(_payload->aileron), aileron, sizeof(uint16_t)*3);
    memcpy(&(_payload->elevator), elevator, sizeof(uint16_t)*3);
    memcpy(&(_payload->rudder), rudder, sizeof(uint16_t)*3);
    memcpy(&(_payload->gyro), gyro, sizeof(uint16_t)*2);
    memcpy(&(_payload->pitch), pitch, sizeof(uint16_t)*5);
    memcpy(&(_payload->throttle), throttle, sizeof(uint16_t)*5);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RADIO_CALIBRATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_calibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_calibration_pack(
        msg, sysid, compid,
        _payload->aileron, _payload->elevator, _payload->rudder, _payload->gyro, _payload->pitch, _payload->throttle,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle,
    fmav_status_t* _status)
{
    fmav_radio_calibration_t* _payload = (fmav_radio_calibration_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->aileron), aileron, sizeof(uint16_t)*3);
    memcpy(&(_payload->elevator), elevator, sizeof(uint16_t)*3);
    memcpy(&(_payload->rudder), rudder, sizeof(uint16_t)*3);
    memcpy(&(_payload->gyro), gyro, sizeof(uint16_t)*2);
    memcpy(&(_payload->pitch), pitch, sizeof(uint16_t)*5);
    memcpy(&(_payload->throttle), throttle, sizeof(uint16_t)*5);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RADIO_CALIBRATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RADIO_CALIBRATION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_radio_calibration_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_radio_calibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_radio_calibration_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->aileron, _payload->elevator, _payload->rudder, _payload->gyro, _payload->pitch, _payload->throttle,
        _status);
}


//----------------------------------------
//-- Message RADIO_CALIBRATION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_radio_calibration_decode(fmav_radio_calibration_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RADIO_CALIBRATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_radio_calibration_pack(
        msg, sysid, compid,
        aileron, elevator, rudder, gyro, pitch, throttle,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_radio_calibration_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const uint16_t* aileron, const uint16_t* elevator, const uint16_t* rudder, const uint16_t* gyro, const uint16_t* pitch, const uint16_t* throttle)
{
    return fmav_msg_radio_calibration_pack_to_frame_buf(
        (uint8_t*)buf,
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

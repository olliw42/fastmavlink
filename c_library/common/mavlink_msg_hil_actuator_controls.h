//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_actuator_controls_t {
    uint64_t time_usec;
    uint64_t flags;
    float controls[16];
    uint8_t mode;
}) fmav_hil_actuator_controls_t;


#define FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93


#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MIN  81
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX  81
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN  81
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA  47

#define FASTMAVLINK_MSG_ID_93_LEN_MIN  81
#define FASTMAVLINK_MSG_ID_93_LEN_MAX  81
#define FASTMAVLINK_MSG_ID_93_LEN  81
#define FASTMAVLINK_MSG_ID_93_CRCEXTRA  47

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN  16

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_93_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_93_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t* _payload = (fmav_hil_actuator_controls_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->flags = flags;
    _payload->mode = mode;
    memcpy(&(_payload->controls), controls, sizeof(float)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->controls, _payload->mode, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags,
    fmav_status_t* _status)
{
    fmav_hil_actuator_controls_t _payload;

    _payload.time_usec = time_usec;
    _payload.flags = flags;
    _payload.mode = mode;
    memcpy(&(_payload.controls), controls, sizeof(float)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_actuator_controls_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_actuator_controls_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_ACTUATOR_CONTROLS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_actuator_controls_decode(fmav_hil_actuator_controls_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS  93

#define mavlink_hil_actuator_controls_t  fmav_hil_actuator_controls_t

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN  81
#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_MIN_LEN  81
#define MAVLINK_MSG_ID_93_LEN  81
#define MAVLINK_MSG_ID_93_MIN_LEN  81

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_CRC  47
#define MAVLINK_MSG_ID_93_CRC  47

#define MAVLINK_MSG_HIL_ACTUATOR_CONTROLS_FIELD_CONTROLS_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_actuator_controls_pack(
        msg, sysid, compid,
        time_usec, controls, mode, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_actuator_controls_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, const float* controls, uint8_t mode, uint64_t flags)
{
    return fmav_msg_hil_actuator_controls_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, controls, mode, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* payload)
{
    fmav_msg_hil_actuator_controls_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_ACTUATOR_CONTROLS_H

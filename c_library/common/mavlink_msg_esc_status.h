//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESC_STATUS_H
#define FASTMAVLINK_MSG_ESC_STATUS_H


//----------------------------------------
//-- Message ESC_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_esc_status_t {
    uint64_t time_usec;
    int32_t rpm[4];
    float voltage[4];
    float current[4];
    uint8_t index;
}) fmav_esc_status_t;


#define FASTMAVLINK_MSG_ID_ESC_STATUS  291


#define FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MIN  57
#define FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX  57
#define FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN  57
#define FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA  10

#define FASTMAVLINK_MSG_ID_291_LEN_MIN  57
#define FASTMAVLINK_MSG_ID_291_LEN_MAX  57
#define FASTMAVLINK_MSG_ID_291_LEN  57
#define FASTMAVLINK_MSG_ID_291_CRCEXTRA  10

#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN  4
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_LEN  4
#define FASTMAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN  4

#define FASTMAVLINK_MSG_ESC_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ESC_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESC_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESC_STATUS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_291_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_291_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message ESC_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t* _payload = (fmav_esc_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->index = index;
    memcpy(&(_payload->rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload->voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload->current), current, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ESC_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_status_pack(
        msg, sysid, compid,
        _payload->index, _payload->time_usec, _payload->rpm, _payload->voltage, _payload->current,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t* _payload = (fmav_esc_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->index = index;
    memcpy(&(_payload->rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload->voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload->current), current, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESC_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESC_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_esc_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->index, _payload->time_usec, _payload->rpm, _payload->voltage, _payload->current,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current,
    fmav_status_t* _status)
{
    fmav_esc_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.index = index;
    memcpy(&(_payload.rpm), rpm, sizeof(int32_t)*4);
    memcpy(&(_payload.voltage), voltage, sizeof(float)*4);
    memcpy(&(_payload.current), current, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESC_STATUS,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_esc_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_esc_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESC_STATUS,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESC_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESC_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_esc_status_decode(fmav_esc_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ESC_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESC_STATUS  291

#define mavlink_esc_status_t  fmav_esc_status_t

#define MAVLINK_MSG_ID_ESC_STATUS_LEN  57
#define MAVLINK_MSG_ID_ESC_STATUS_MIN_LEN  57
#define MAVLINK_MSG_ID_291_LEN  57
#define MAVLINK_MSG_ID_291_MIN_LEN  57

#define MAVLINK_MSG_ID_ESC_STATUS_CRC  10
#define MAVLINK_MSG_ID_291_CRC  10

#define MAVLINK_MSG_ESC_STATUS_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_STATUS_FIELD_CURRENT_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_esc_status_pack(
        msg, sysid, compid,
        index, time_usec, rpm, voltage, current,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_esc_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, uint64_t time_usec, const int32_t* rpm, const float* voltage, const float* current)
{
    return fmav_msg_esc_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        index, time_usec, rpm, voltage, current,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_esc_status_decode(const mavlink_message_t* msg, mavlink_esc_status_t* payload)
{
    fmav_msg_esc_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESC_STATUS_H

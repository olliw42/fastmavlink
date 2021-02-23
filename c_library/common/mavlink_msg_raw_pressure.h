//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RAW_PRESSURE_H
#define FASTMAVLINK_MSG_RAW_PRESSURE_H


//----------------------------------------
//-- Message RAW_PRESSURE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_raw_pressure_t {
    uint64_t time_usec;
    int16_t press_abs;
    int16_t press_diff1;
    int16_t press_diff2;
    int16_t temperature;
}) fmav_raw_pressure_t;


#define FASTMAVLINK_MSG_ID_RAW_PRESSURE  28


#define FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MIN  16
#define FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN  16
#define FASTMAVLINK_MSG_RAW_PRESSURE_CRCEXTRA  67

#define FASTMAVLINK_MSG_ID_28_LEN_MIN  16
#define FASTMAVLINK_MSG_ID_28_LEN_MAX  16
#define FASTMAVLINK_MSG_ID_28_LEN  16
#define FASTMAVLINK_MSG_ID_28_CRCEXTRA  67



#define FASTMAVLINK_MSG_RAW_PRESSURE_FLAGS  0
#define FASTMAVLINK_MSG_RAW_PRESSURE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RAW_PRESSURE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RAW_PRESSURE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_pressure_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_raw_pressure_t* _payload = (fmav_raw_pressure_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->press_abs = press_abs;
    _payload->press_diff1 = press_diff1;
    _payload->press_diff2 = press_diff2;
    _payload->temperature = temperature;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RAW_PRESSURE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RAW_PRESSURE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_pressure_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_pressure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_pressure_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->press_abs, _payload->press_diff1, _payload->press_diff2, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_pressure_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature,
    fmav_status_t* _status)
{
    fmav_raw_pressure_t* _payload = (fmav_raw_pressure_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->press_abs = press_abs;
    _payload->press_diff1 = press_diff1;
    _payload->press_diff2 = press_diff2;
    _payload->temperature = temperature;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RAW_PRESSURE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_PRESSURE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_PRESSURE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_PRESSURE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_pressure_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_pressure_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_pressure_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->press_abs, _payload->press_diff1, _payload->press_diff2, _payload->temperature,
        _status);
}


//----------------------------------------
//-- Message RAW_PRESSURE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_pressure_decode(fmav_raw_pressure_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RAW_PRESSURE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RAW_PRESSURE  28

#define mavlink_raw_pressure_t  fmav_raw_pressure_t

#define MAVLINK_MSG_ID_RAW_PRESSURE_LEN  16
#define MAVLINK_MSG_ID_RAW_PRESSURE_MIN_LEN  16
#define MAVLINK_MSG_ID_28_LEN  16
#define MAVLINK_MSG_ID_28_MIN_LEN  16

#define MAVLINK_MSG_ID_RAW_PRESSURE_CRC  67
#define MAVLINK_MSG_ID_28_CRC  67




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_pressure_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_raw_pressure_pack(
        msg, sysid, compid,
        time_usec, press_abs, press_diff1, press_diff2, temperature,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_pressure_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature)
{
    return fmav_msg_raw_pressure_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, press_abs, press_diff1, press_diff2, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_raw_pressure_decode(const mavlink_message_t* msg, mavlink_raw_pressure_t* payload)
{
    fmav_msg_raw_pressure_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RAW_PRESSURE_H

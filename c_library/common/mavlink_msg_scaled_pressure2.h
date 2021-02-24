//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SCALED_PRESSURE2_H
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_H


//----------------------------------------
//-- Message SCALED_PRESSURE2
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_scaled_pressure2_t {
    uint32_t time_boot_ms;
    float press_abs;
    float press_diff;
    int16_t temperature;
    int16_t temperature_press_diff;
}) fmav_scaled_pressure2_t;


#define FASTMAVLINK_MSG_ID_SCALED_PRESSURE2  137


#define FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MIN  14
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN  16
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_CRCEXTRA  195

#define FASTMAVLINK_MSG_ID_137_LEN_MIN  14
#define FASTMAVLINK_MSG_ID_137_LEN_MAX  16
#define FASTMAVLINK_MSG_ID_137_LEN  16
#define FASTMAVLINK_MSG_ID_137_CRCEXTRA  195



#define FASTMAVLINK_MSG_SCALED_PRESSURE2_FLAGS  0
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SCALED_PRESSURE2_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SCALED_PRESSURE2 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure2_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff,
    fmav_status_t* _status)
{
    fmav_scaled_pressure2_t* _payload = (fmav_scaled_pressure2_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->press_abs = press_abs;
    _payload->press_diff = press_diff;
    _payload->temperature = temperature;
    _payload->temperature_press_diff = temperature_press_diff;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SCALED_PRESSURE2;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SCALED_PRESSURE2_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure2_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_pressure2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_pressure2_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->press_abs, _payload->press_diff, _payload->temperature, _payload->temperature_press_diff,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure2_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff,
    fmav_status_t* _status)
{
    fmav_scaled_pressure2_t* _payload = (fmav_scaled_pressure2_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->press_abs = press_abs;
    _payload->press_diff = press_diff;
    _payload->temperature = temperature;
    _payload->temperature_press_diff = temperature_press_diff;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE2;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE2 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SCALED_PRESSURE2 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SCALED_PRESSURE2_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_scaled_pressure2_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_scaled_pressure2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_scaled_pressure2_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->press_abs, _payload->press_diff, _payload->temperature, _payload->temperature_press_diff,
        _status);
}


//----------------------------------------
//-- Message SCALED_PRESSURE2 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_scaled_pressure2_decode(fmav_scaled_pressure2_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SCALED_PRESSURE2_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SCALED_PRESSURE2  137

#define mavlink_scaled_pressure2_t  fmav_scaled_pressure2_t

#define MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN  16
#define MAVLINK_MSG_ID_SCALED_PRESSURE2_MIN_LEN  14
#define MAVLINK_MSG_ID_137_LEN  16
#define MAVLINK_MSG_ID_137_MIN_LEN  14

#define MAVLINK_MSG_ID_SCALED_PRESSURE2_CRC  195
#define MAVLINK_MSG_ID_137_CRC  195




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_pressure2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_scaled_pressure2_pack(
        msg, sysid, compid,
        time_boot_ms, press_abs, press_diff, temperature, temperature_press_diff,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_scaled_pressure2_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
    return fmav_msg_scaled_pressure2_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, press_abs, press_diff, temperature, temperature_press_diff,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_scaled_pressure2_decode(const mavlink_message_t* msg, mavlink_scaled_pressure2_t* payload)
{
    fmav_msg_scaled_pressure2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SCALED_PRESSURE2_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BATTERY2_H
#define FASTMAVLINK_MSG_BATTERY2_H


//----------------------------------------
//-- Message BATTERY2
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_battery2_t {
    uint16_t voltage;
    int16_t current_battery;
}) fmav_battery2_t;


#define FASTMAVLINK_MSG_ID_BATTERY2  181


#define FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MIN  4
#define FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN  4
#define FASTMAVLINK_MSG_BATTERY2_CRCEXTRA  174

#define FASTMAVLINK_MSG_ID_181_LEN_MIN  4
#define FASTMAVLINK_MSG_ID_181_LEN_MAX  4
#define FASTMAVLINK_MSG_ID_181_LEN  4
#define FASTMAVLINK_MSG_ID_181_CRCEXTRA  174



#define FASTMAVLINK_MSG_BATTERY2_FLAGS  0
#define FASTMAVLINK_MSG_BATTERY2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BATTERY2_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BATTERY2_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_181_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_181_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message BATTERY2 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t voltage, int16_t current_battery,
    fmav_status_t* _status)
{
    fmav_battery2_t* _payload = (fmav_battery2_t*)msg->payload;

    _payload->voltage = voltage;
    _payload->current_battery = current_battery;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_BATTERY2;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_BATTERY2_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery2_pack(
        msg, sysid, compid,
        _payload->voltage, _payload->current_battery,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t voltage, int16_t current_battery,
    fmav_status_t* _status)
{
    fmav_battery2_t* _payload = (fmav_battery2_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->voltage = voltage;
    _payload->current_battery = current_battery;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BATTERY2;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY2 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY2 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery2_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->voltage, _payload->current_battery,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t voltage, int16_t current_battery,
    fmav_status_t* _status)
{
    fmav_battery2_t _payload;

    _payload.voltage = voltage;
    _payload.current_battery = current_battery;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BATTERY2,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery2_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BATTERY2,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BATTERY2 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_battery2_decode(fmav_battery2_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_BATTERY2_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BATTERY2  181

#define mavlink_battery2_t  fmav_battery2_t

#define MAVLINK_MSG_ID_BATTERY2_LEN  4
#define MAVLINK_MSG_ID_BATTERY2_MIN_LEN  4
#define MAVLINK_MSG_ID_181_LEN  4
#define MAVLINK_MSG_ID_181_MIN_LEN  4

#define MAVLINK_MSG_ID_BATTERY2_CRC  174
#define MAVLINK_MSG_ID_181_CRC  174




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t voltage, int16_t current_battery)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_battery2_pack(
        msg, sysid, compid,
        voltage, current_battery,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery2_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t voltage, int16_t current_battery)
{
    return fmav_msg_battery2_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        voltage, current_battery,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_battery2_decode(const mavlink_message_t* msg, mavlink_battery2_t* payload)
{
    fmav_msg_battery2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BATTERY2_H

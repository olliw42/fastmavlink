//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_INJECT_DATA_H
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_H


//----------------------------------------
//-- Message GPS_INJECT_DATA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_inject_data_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t len;
    uint8_t data[110];
}) fmav_gps_inject_data_t;


#define FASTMAVLINK_MSG_ID_GPS_INJECT_DATA  123


#define FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MIN  113
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX  113
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN  113
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA  250

#define FASTMAVLINK_MSG_ID_123_LEN_MIN  113
#define FASTMAVLINK_MSG_ID_123_LEN_MAX  113
#define FASTMAVLINK_MSG_ID_123_LEN  113
#define FASTMAVLINK_MSG_ID_123_CRCEXTRA  250

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN  110

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FLAGS  3
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_INJECT_DATA_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_GPS_INJECT_DATA_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_123_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_123_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message GPS_INJECT_DATA packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t* _payload = (fmav_gps_inject_data_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*110);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS_INJECT_DATA;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_inject_data_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->len, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t* _payload = (fmav_gps_inject_data_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->len = len;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*110);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INJECT_DATA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_inject_data_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->len, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_gps_inject_data_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.len = len;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*110);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_INJECT_DATA,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_inject_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_inject_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_INJECT_DATA,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INJECT_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_INJECT_DATA unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_inject_data_decode(fmav_gps_inject_data_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GPS_INJECT_DATA_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_INJECT_DATA  123

#define mavlink_gps_inject_data_t  fmav_gps_inject_data_t

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_LEN  113
#define MAVLINK_MSG_ID_GPS_INJECT_DATA_MIN_LEN  113
#define MAVLINK_MSG_ID_123_LEN  113
#define MAVLINK_MSG_ID_123_MIN_LEN  113

#define MAVLINK_MSG_ID_GPS_INJECT_DATA_CRC  250
#define MAVLINK_MSG_ID_123_CRC  250

#define MAVLINK_MSG_GPS_INJECT_DATA_FIELD_DATA_LEN 110


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_inject_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_inject_data_pack(
        msg, sysid, compid,
        target_system, target_component, len, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_inject_data_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t len, const uint8_t* data)
{
    return fmav_msg_gps_inject_data_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, len, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_inject_data_decode(const mavlink_message_t* msg, mavlink_gps_inject_data_t* payload)
{
    fmav_msg_gps_inject_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_INJECT_DATA_H

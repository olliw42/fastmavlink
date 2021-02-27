//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_REPORT_H
#define FASTMAVLINK_MSG_TERRAIN_REPORT_H


//----------------------------------------
//-- Message TERRAIN_REPORT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_report_t {
    int32_t lat;
    int32_t lon;
    float terrain_height;
    float current_height;
    uint16_t spacing;
    uint16_t pending;
    uint16_t loaded;
}) fmav_terrain_report_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_REPORT  136


#define FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MIN  22
#define FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA  1

#define FASTMAVLINK_MSG_ID_136_LEN_MIN  22
#define FASTMAVLINK_MSG_ID_136_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_136_LEN  22
#define FASTMAVLINK_MSG_ID_136_CRCEXTRA  1



#define FASTMAVLINK_MSG_TERRAIN_REPORT_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_REPORT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_REPORT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_REPORT_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_136_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_136_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message TERRAIN_REPORT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded,
    fmav_status_t* _status)
{
    fmav_terrain_report_t* _payload = (fmav_terrain_report_t*)msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->terrain_height = terrain_height;
    _payload->current_height = current_height;
    _payload->spacing = spacing;
    _payload->pending = pending;
    _payload->loaded = loaded;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_REPORT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_report_pack(
        msg, sysid, compid,
        _payload->lat, _payload->lon, _payload->spacing, _payload->terrain_height, _payload->current_height, _payload->pending, _payload->loaded,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded,
    fmav_status_t* _status)
{
    fmav_terrain_report_t* _payload = (fmav_terrain_report_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->terrain_height = terrain_height;
    _payload->current_height = current_height;
    _payload->spacing = spacing;
    _payload->pending = pending;
    _payload->loaded = loaded;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_REPORT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REPORT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REPORT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_report_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->lat, _payload->lon, _payload->spacing, _payload->terrain_height, _payload->current_height, _payload->pending, _payload->loaded,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded,
    fmav_status_t* _status)
{
    fmav_terrain_report_t _payload;

    _payload.lat = lat;
    _payload.lon = lon;
    _payload.terrain_height = terrain_height;
    _payload.current_height = current_height;
    _payload.spacing = spacing;
    _payload.pending = pending;
    _payload.loaded = loaded;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REPORT,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REPORT,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_REPORT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_report_decode(fmav_terrain_report_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TERRAIN_REPORT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_REPORT  136

#define mavlink_terrain_report_t  fmav_terrain_report_t

#define MAVLINK_MSG_ID_TERRAIN_REPORT_LEN  22
#define MAVLINK_MSG_ID_TERRAIN_REPORT_MIN_LEN  22
#define MAVLINK_MSG_ID_136_LEN  22
#define MAVLINK_MSG_ID_136_MIN_LEN  22

#define MAVLINK_MSG_ID_TERRAIN_REPORT_CRC  1
#define MAVLINK_MSG_ID_136_CRC  1




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_report_pack(
        msg, sysid, compid,
        lat, lon, spacing, terrain_height, current_height, pending, loaded,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_report_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t spacing, float terrain_height, float current_height, uint16_t pending, uint16_t loaded)
{
    return fmav_msg_terrain_report_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        lat, lon, spacing, terrain_height, current_height, pending, loaded,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_report_decode(const mavlink_message_t* msg, mavlink_terrain_report_t* payload)
{
    fmav_msg_terrain_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_REPORT_H

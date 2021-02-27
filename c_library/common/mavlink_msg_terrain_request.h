//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_REQUEST_H
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_H


//----------------------------------------
//-- Message TERRAIN_REQUEST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_request_t {
    uint64_t mask;
    int32_t lat;
    int32_t lon;
    uint16_t grid_spacing;
}) fmav_terrain_request_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_REQUEST  133


#define FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MIN  18
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX  18
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN  18
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA  6

#define FASTMAVLINK_MSG_ID_133_LEN_MIN  18
#define FASTMAVLINK_MSG_ID_133_LEN_MAX  18
#define FASTMAVLINK_MSG_ID_133_LEN  18
#define FASTMAVLINK_MSG_ID_133_CRCEXTRA  6



#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_REQUEST_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_REQUEST_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_133_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_133_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message TERRAIN_REQUEST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t* _payload = (fmav_terrain_request_t*)msg->payload;

    _payload->mask = mask;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_REQUEST;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_request_pack(
        msg, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->mask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t* _payload = (fmav_terrain_request_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mask = mask;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_request_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->mask,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask,
    fmav_status_t* _status)
{
    fmav_terrain_request_t _payload;

    _payload.mask = mask;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.grid_spacing = grid_spacing;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REQUEST,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_request_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_REQUEST,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_REQUEST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_REQUEST unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_request_decode(fmav_terrain_request_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TERRAIN_REQUEST_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_REQUEST  133

#define mavlink_terrain_request_t  fmav_terrain_request_t

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN  18
#define MAVLINK_MSG_ID_TERRAIN_REQUEST_MIN_LEN  18
#define MAVLINK_MSG_ID_133_LEN  18
#define MAVLINK_MSG_ID_133_MIN_LEN  18

#define MAVLINK_MSG_ID_TERRAIN_REQUEST_CRC  6
#define MAVLINK_MSG_ID_133_CRC  6




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_request_pack(
        msg, sysid, compid,
        lat, lon, grid_spacing, mask,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_request_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint64_t mask)
{
    return fmav_msg_terrain_request_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        lat, lon, grid_spacing, mask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_request_decode(const mavlink_message_t* msg, mavlink_terrain_request_t* payload)
{
    fmav_msg_terrain_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_REQUEST_H

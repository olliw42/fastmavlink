//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_DATA_H
#define FASTMAVLINK_MSG_TERRAIN_DATA_H


//----------------------------------------
//-- Message TERRAIN_DATA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_data_t {
    int32_t lat;
    int32_t lon;
    uint16_t grid_spacing;
    int16_t data[16];
    uint8_t gridbit;
}) fmav_terrain_data_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_DATA  134


#define FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MIN  43
#define FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX  43
#define FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN  43
#define FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA  229

#define FASTMAVLINK_MSG_ID_134_LEN_MIN  43
#define FASTMAVLINK_MSG_ID_134_LEN_MAX  43
#define FASTMAVLINK_MSG_ID_134_LEN  43
#define FASTMAVLINK_MSG_ID_134_CRCEXTRA  229

#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_LEN  16

#define FASTMAVLINK_MSG_TERRAIN_DATA_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_DATA_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message TERRAIN_DATA packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t* data,
    fmav_status_t* _status)
{
    fmav_terrain_data_t* _payload = (fmav_terrain_data_t*)msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;
    _payload->gridbit = gridbit;
    memcpy(&(_payload->data), data, sizeof(int16_t)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_DATA;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_data_pack(
        msg, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->gridbit, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t* data,
    fmav_status_t* _status)
{
    fmav_terrain_data_t* _payload = (fmav_terrain_data_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->grid_spacing = grid_spacing;
    _payload->gridbit = gridbit;
    memcpy(&(_payload->data), data, sizeof(int16_t)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_DATA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_DATA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_DATA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_data_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->lat, _payload->lon, _payload->grid_spacing, _payload->gridbit, _payload->data,
        _status);
}


//----------------------------------------
//-- Message TERRAIN_DATA unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_data_decode(fmav_terrain_data_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_DATA  134

#define mavlink_terrain_data_t  fmav_terrain_data_t

#define MAVLINK_MSG_ID_TERRAIN_DATA_LEN  43
#define MAVLINK_MSG_ID_TERRAIN_DATA_MIN_LEN  43
#define MAVLINK_MSG_ID_134_LEN  43
#define MAVLINK_MSG_ID_134_MIN_LEN  43

#define MAVLINK_MSG_ID_TERRAIN_DATA_CRC  229
#define MAVLINK_MSG_ID_134_CRC  229

#define MAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_data_pack(
        msg, sysid, compid,
        lat, lon, grid_spacing, gridbit, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_data_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t* data)
{
    return fmav_msg_terrain_data_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        lat, lon, grid_spacing, gridbit, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_data_decode(const mavlink_message_t* msg, mavlink_terrain_data_t* payload)
{
    fmav_msg_terrain_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_DATA_H

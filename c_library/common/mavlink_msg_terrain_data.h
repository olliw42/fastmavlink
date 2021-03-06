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

#define FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX  43
#define FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA  229

#define FASTMAVLINK_MSG_TERRAIN_DATA_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_DATA_FRAME_LEN_MAX  68

#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_LAT_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_LON_OFS  4
#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_GRID_SPACING_OFS  8
#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_OFS  10
#define FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_GRIDBIT_OFS  42


//----------------------------------------
//-- Message TERRAIN_DATA packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t* data,
    fmav_status_t* _status)
{
    fmav_terrain_data_t _payload;

    _payload.lat = lat;
    _payload.lon = lon;
    _payload.grid_spacing = grid_spacing;
    _payload.gridbit = gridbit;
    memcpy(&(_payload.data), data, sizeof(int16_t)*16);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_DATA,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_DATA,
        FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_DATA unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_terrain_data_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_terrain_data_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_data_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_data_decode(fmav_terrain_data_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_DATA_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_data_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_data_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_data_get_field_grid_spacing(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_terrain_data_get_field_gridbit(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_terrain_data_get_field_data_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[10]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_terrain_data_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_NUM) return 0;
    return ((int16_t*)&(msg->payload[10]))[index];
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

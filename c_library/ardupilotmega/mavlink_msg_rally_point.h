//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RALLY_POINT_H
#define FASTMAVLINK_MSG_RALLY_POINT_H


//----------------------------------------
//-- Message RALLY_POINT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rally_point_t {
    int32_t lat;
    int32_t lng;
    int16_t alt;
    int16_t break_alt;
    uint16_t land_dir;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t idx;
    uint8_t count;
    uint8_t flags;
}) fmav_rally_point_t;


#define FASTMAVLINK_MSG_ID_RALLY_POINT  175

#define FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA  138

#define FASTMAVLINK_MSG_RALLY_POINT_FLAGS  3
#define FASTMAVLINK_MSG_RALLY_POINT_TARGET_SYSTEM_OFS  14
#define FASTMAVLINK_MSG_RALLY_POINT_TARGET_COMPONENT_OFS  15

#define FASTMAVLINK_MSG_RALLY_POINT_FRAME_LEN_MAX  44



#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_LAT_OFS  0
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_LNG_OFS  4
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_ALT_OFS  8
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_BREAK_ALT_OFS  10
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_LAND_DIR_OFS  12
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_TARGET_SYSTEM_OFS  14
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_TARGET_COMPONENT_OFS  15
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_IDX_OFS  16
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_COUNT_OFS  17
#define FASTMAVLINK_MSG_RALLY_POINT_FIELD_FLAGS_OFS  18


//----------------------------------------
//-- Message RALLY_POINT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_rally_point_t* _payload = (fmav_rally_point_t*)msg->payload;

    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt = alt;
    _payload->break_alt = break_alt;
    _payload->land_dir = land_dir;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->idx = idx;
    _payload->count = count;
    _payload->flags = flags;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RALLY_POINT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rally_point_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rally_point_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->idx, _payload->count, _payload->lat, _payload->lng, _payload->alt, _payload->break_alt, _payload->land_dir, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_rally_point_t* _payload = (fmav_rally_point_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt = alt;
    _payload->break_alt = break_alt;
    _payload->land_dir = land_dir;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->idx = idx;
    _payload->count = count;
    _payload->flags = flags;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RALLY_POINT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RALLY_POINT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RALLY_POINT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rally_point_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rally_point_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->idx, _payload->count, _payload->lat, _payload->lng, _payload->alt, _payload->break_alt, _payload->land_dir, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_rally_point_t _payload;

    _payload.lat = lat;
    _payload.lng = lng;
    _payload.alt = alt;
    _payload.break_alt = break_alt;
    _payload.land_dir = land_dir;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.idx = idx;
    _payload.count = count;
    _payload.flags = flags;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RALLY_POINT,
        FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rally_point_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RALLY_POINT,
        FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RALLY_POINT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RALLY_POINT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_rally_point_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_rally_point_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rally_point_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rally_point_decode(fmav_rally_point_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RALLY_POINT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_rally_point_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_rally_point_get_field_lng(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rally_point_get_field_alt(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_rally_point_get_field_break_alt(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rally_point_get_field_land_dir(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rally_point_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rally_point_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rally_point_get_field_idx(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rally_point_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_rally_point_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RALLY_POINT  175

#define mavlink_rally_point_t  fmav_rally_point_t

#define MAVLINK_MSG_ID_RALLY_POINT_LEN  19
#define MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN  19
#define MAVLINK_MSG_ID_175_LEN  19
#define MAVLINK_MSG_ID_175_MIN_LEN  19

#define MAVLINK_MSG_ID_RALLY_POINT_CRC  138
#define MAVLINK_MSG_ID_175_CRC  138




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rally_point_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rally_point_pack(
        msg, sysid, compid,
        target_system, target_component, idx, count, lat, lng, alt, break_alt, land_dir, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rally_point_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t idx, uint8_t count, int32_t lat, int32_t lng, int16_t alt, int16_t break_alt, uint16_t land_dir, uint8_t flags)
{
    return fmav_msg_rally_point_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, idx, count, lat, lng, alt, break_alt, land_dir, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rally_point_decode(const mavlink_message_t* msg, mavlink_rally_point_t* payload)
{
    fmav_msg_rally_point_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RALLY_POINT_H

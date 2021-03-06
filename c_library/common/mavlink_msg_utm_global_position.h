//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_utm_global_position_t {
    uint64_t time;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int32_t next_lat;
    int32_t next_lon;
    int32_t next_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t h_acc;
    uint16_t v_acc;
    uint16_t vel_acc;
    uint16_t update_rate;
    uint8_t uas_id[18];
    uint8_t flight_state;
    uint8_t flags;
}) fmav_utm_global_position_t;


#define FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX  70
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA  99

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FLAGS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FRAME_LEN_MAX  95

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_NUM  18 // number of elements in array
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN  18 // length of array = number of bytes

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_TIME_OFS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_RELATIVE_ALT_OFS  20
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_LAT_OFS  24
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_LON_OFS  28
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_NEXT_ALT_OFS  32
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VX_OFS  36
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VY_OFS  38
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VZ_OFS  40
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_H_ACC_OFS  42
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_V_ACC_OFS  44
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_VEL_ACC_OFS  46
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UPDATE_RATE_OFS  48
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_OFS  50
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_FLIGHT_STATE_OFS  68
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_FLAGS_OFS  69


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)msg->payload;

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack(
        msg, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t _payload;

    _payload.time = time;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.relative_alt = relative_alt;
    _payload.next_lat = next_lat;
    _payload.next_lon = next_lon;
    _payload.next_alt = next_alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_acc = vel_acc;
    _payload.update_rate = update_rate;
    _payload.flight_state = flight_state;
    _payload.flags = flags;
    memcpy(&(_payload.uas_id), uas_id, sizeof(uint8_t)*18);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_utm_global_position_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_utm_global_position_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_utm_global_position_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_utm_global_position_decode(fmav_utm_global_position_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_utm_global_position_get_field_time(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_relative_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_utm_global_position_get_field_next_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vx(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vy(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_utm_global_position_get_field_vz(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_h_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_v_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_vel_acc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_get_field_update_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_flight_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[68]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_flags(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[69]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_utm_global_position_get_field_uas_id_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[50]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_utm_global_position_get_field_uas_id(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_NUM) return 0;
    return ((uint8_t*)&(msg->payload[50]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340

#define mavlink_utm_global_position_t  fmav_utm_global_position_t

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN  70
#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_MIN_LEN  70
#define MAVLINK_MSG_ID_340_LEN  70
#define MAVLINK_MSG_ID_340_MIN_LEN  70

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_CRC  99
#define MAVLINK_MSG_ID_340_CRC  99

#define MAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN 18


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_utm_global_position_pack(
        msg, sysid, compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_utm_global_position_decode(const mavlink_message_t* msg, mavlink_utm_global_position_t* payload)
{
    fmav_msg_utm_global_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H

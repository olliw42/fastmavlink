//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GLOBAL_POSITION_INT_H
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_H


//----------------------------------------
//-- Message GLOBAL_POSITION_INT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_global_position_int_t {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t hdg;
}) fmav_global_position_int_t;


#define FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT  33

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA  104

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FLAGS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FRAME_LEN_MAX  53



#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_LAT_OFS  4
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_LON_OFS  8
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_ALT_OFS  12
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_RELATIVE_ALT_OFS  16
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_VX_OFS  20
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_VY_OFS  22
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_VZ_OFS  24
#define FASTMAVLINK_MSG_GLOBAL_POSITION_INT_FIELD_HDG_OFS  26


//----------------------------------------
//-- Message GLOBAL_POSITION_INT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg,
    fmav_status_t* _status)
{
    fmav_global_position_int_t* _payload = (fmav_global_position_int_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->hdg = hdg;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->hdg,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg,
    fmav_status_t* _status)
{
    fmav_global_position_int_t* _payload = (fmav_global_position_int_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->hdg = hdg;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_global_position_int_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->hdg,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg,
    fmav_status_t* _status)
{
    fmav_global_position_int_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.relative_alt = relative_alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.hdg = hdg;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_global_position_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GLOBAL_POSITION_INT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GLOBAL_POSITION_INT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_global_position_int_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_global_position_int_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_position_int_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_global_position_int_decode(fmav_global_position_int_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GLOBAL_POSITION_INT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_global_position_int_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_global_position_int_get_field_relative_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_global_position_int_get_field_vx(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_global_position_int_get_field_vy(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_global_position_int_get_field_vz(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_global_position_int_get_field_hdg(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33

#define mavlink_global_position_int_t  fmav_global_position_int_t

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN  28
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN  28
#define MAVLINK_MSG_ID_33_LEN  28
#define MAVLINK_MSG_ID_33_MIN_LEN  28

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC  104
#define MAVLINK_MSG_ID_33_CRC  104




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_global_position_int_pack(
        msg, sysid, compid,
        time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_global_position_int_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
    return fmav_msg_global_position_int_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* payload)
{
    fmav_msg_global_position_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GLOBAL_POSITION_INT_H

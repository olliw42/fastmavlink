//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_GPS_H
#define FASTMAVLINK_MSG_HIL_GPS_H


//----------------------------------------
//-- Message HIL_GPS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_gps_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    int16_t vn;
    int16_t ve;
    int16_t vd;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
    uint8_t id;
    uint16_t yaw;
}) fmav_hil_gps_t;


#define FASTMAVLINK_MSG_ID_HIL_GPS  113

#define FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX  39
#define FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA  124

#define FASTMAVLINK_MSG_HIL_GPS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_GPS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_GPS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_GPS_FRAME_LEN_MAX  64



#define FASTMAVLINK_MSG_HIL_GPS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_EPH_OFS  20
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_EPV_OFS  22
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_VEL_OFS  24
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_VN_OFS  26
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_VE_OFS  28
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_VD_OFS  30
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_COG_OFS  32
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_FIX_TYPE_OFS  34
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_SATELLITES_VISIBLE_OFS  35
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_ID_OFS  36
#define FASTMAVLINK_MSG_HIL_GPS_FIELD_YAW_OFS  37


//----------------------------------------
//-- Message HIL_GPS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t id, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_hil_gps_t* _payload = (fmav_hil_gps_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->id = id;
    _payload->yaw = yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_GPS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_gps_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_gps_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->vn, _payload->ve, _payload->vd, _payload->cog, _payload->satellites_visible, _payload->id, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t id, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_hil_gps_t* _payload = (fmav_hil_gps_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->id = id;
    _payload->yaw = yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_GPS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_GPS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_GPS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_gps_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_gps_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->vn, _payload->ve, _payload->vd, _payload->cog, _payload->satellites_visible, _payload->id, _payload->yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t id, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_hil_gps_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.vel = vel;
    _payload.vn = vn;
    _payload.ve = ve;
    _payload.vd = vd;
    _payload.cog = cog;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.id = id;
    _payload.yaw = yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_GPS,
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_gps_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_GPS,
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_GPS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hil_gps_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hil_gps_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_gps_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_gps_decode(fmav_hil_gps_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_gps_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_gps_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_gps_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_gps_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_get_field_eph(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_get_field_epv(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_get_field_vel(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_gps_get_field_vn(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_gps_get_field_ve(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_gps_get_field_vd(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_get_field_cog(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_gps_get_field_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_gps_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_gps_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_gps_get_field_yaw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_GPS  113

#define mavlink_hil_gps_t  fmav_hil_gps_t

#define MAVLINK_MSG_ID_HIL_GPS_LEN  39
#define MAVLINK_MSG_ID_HIL_GPS_MIN_LEN  36
#define MAVLINK_MSG_ID_113_LEN  39
#define MAVLINK_MSG_ID_113_MIN_LEN  36

#define MAVLINK_MSG_ID_HIL_GPS_CRC  124
#define MAVLINK_MSG_ID_113_CRC  124




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_gps_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t id, uint16_t yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_gps_pack(
        msg, sysid, compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, id, yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_gps_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible, uint8_t id, uint16_t yaw)
{
    return fmav_msg_hil_gps_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible, id, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_gps_decode(const mavlink_message_t* msg, mavlink_hil_gps_t* payload)
{
    fmav_msg_hil_gps_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_GPS_H

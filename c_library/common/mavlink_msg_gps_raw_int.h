//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_RAW_INT_H
#define FASTMAVLINK_MSG_GPS_RAW_INT_H


//----------------------------------------
//-- Message GPS_RAW_INT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_raw_int_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
    int32_t alt_ellipsoid;
    uint32_t h_acc;
    uint32_t v_acc;
    uint32_t vel_acc;
    uint32_t hdg_acc;
    uint16_t yaw;
}) fmav_gps_raw_int_t;


#define FASTMAVLINK_MSG_ID_GPS_RAW_INT  24

#define FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA  24

#define FASTMAVLINK_MSG_GPS_RAW_INT_FLAGS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS_RAW_INT_FRAME_LEN_MAX  77



#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_EPH_OFS  20
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_EPV_OFS  22
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_VEL_OFS  24
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_COG_OFS  26
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_FIX_TYPE_OFS  28
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_SATELLITES_VISIBLE_OFS  29
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_ALT_ELLIPSOID_OFS  30
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_H_ACC_OFS  34
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_V_ACC_OFS  38
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_VEL_ACC_OFS  42
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_HDG_ACC_OFS  46
#define FASTMAVLINK_MSG_GPS_RAW_INT_FIELD_YAW_OFS  50


//----------------------------------------
//-- Message GPS_RAW_INT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t* _payload = (fmav_gps_raw_int_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;
    _payload->yaw = yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS_RAW_INT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_raw_int_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t* _payload = (fmav_gps_raw_int_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->alt_ellipsoid = alt_ellipsoid;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->hdg_acc = hdg_acc;
    _payload->yaw = yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_RAW_INT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_raw_int_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->alt_ellipsoid, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->hdg_acc, _payload->yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_raw_int_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.vel = vel;
    _payload.cog = cog;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.alt_ellipsoid = alt_ellipsoid;
    _payload.h_acc = h_acc;
    _payload.v_acc = v_acc;
    _payload.vel_acc = vel_acc;
    _payload.hdg_acc = hdg_acc;
    _payload.yaw = yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_RAW_INT,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_raw_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_RAW_INT,
        FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_RAW_INT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_RAW_INT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gps_raw_int_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gps_raw_int_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_raw_int_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_raw_int_decode(fmav_gps_raw_int_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_RAW_INT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_gps_raw_int_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_eph(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_epv(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_vel(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_cog(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_raw_int_get_field_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_raw_int_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_raw_int_get_field_alt_ellipsoid(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_h_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_v_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_vel_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_raw_int_get_field_hdg_acc(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_raw_int_get_field_yaw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_RAW_INT  24

#define mavlink_gps_raw_int_t  fmav_gps_raw_int_t

#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN  52
#define MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN  30
#define MAVLINK_MSG_ID_24_LEN  52
#define MAVLINK_MSG_ID_24_MIN_LEN  30

#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC  24
#define MAVLINK_MSG_ID_24_CRC  24




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_raw_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_raw_int_pack(
        msg, sysid, compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_raw_int_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
{
    return fmav_msg_gps_raw_int_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_raw_int_decode(const mavlink_message_t* msg, mavlink_gps_raw_int_t* payload)
{
    fmav_msg_gps_raw_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_RAW_INT_H

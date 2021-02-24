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


#define FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MIN  36
#define FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX  39
#define FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN  39
#define FASTMAVLINK_MSG_HIL_GPS_CRCEXTRA  124

#define FASTMAVLINK_MSG_ID_113_LEN_MIN  36
#define FASTMAVLINK_MSG_ID_113_LEN_MAX  39
#define FASTMAVLINK_MSG_ID_113_LEN  39
#define FASTMAVLINK_MSG_ID_113_CRCEXTRA  124



#define FASTMAVLINK_MSG_HIL_GPS_FLAGS  0
#define FASTMAVLINK_MSG_HIL_GPS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_GPS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIL_GPS packing routines
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
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message HIL_GPS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_gps_decode(fmav_hil_gps_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_GPS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
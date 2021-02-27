//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS2_RAW_H
#define FASTMAVLINK_MSG_GPS2_RAW_H


//----------------------------------------
//-- Message GPS2_RAW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps2_raw_t {
    uint64_t time_usec;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint32_t dgps_age;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    uint16_t cog;
    uint8_t fix_type;
    uint8_t satellites_visible;
    uint8_t dgps_numch;
    uint16_t yaw;
}) fmav_gps2_raw_t;


#define FASTMAVLINK_MSG_ID_GPS2_RAW  124


#define FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MIN  35
#define FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN  37
#define FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA  87

#define FASTMAVLINK_MSG_ID_124_LEN_MIN  35
#define FASTMAVLINK_MSG_ID_124_LEN_MAX  37
#define FASTMAVLINK_MSG_ID_124_LEN  37
#define FASTMAVLINK_MSG_ID_124_CRCEXTRA  87



#define FASTMAVLINK_MSG_GPS2_RAW_FLAGS  0
#define FASTMAVLINK_MSG_GPS2_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS2_RAW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS2_RAW_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_124_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_124_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message GPS2_RAW packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t* _payload = (fmav_gps2_raw_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->dgps_age = dgps_age;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->dgps_numch = dgps_numch;
    _payload->yaw = yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS2_RAW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_raw_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->dgps_numch, _payload->dgps_age, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t* _payload = (fmav_gps2_raw_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->dgps_age = dgps_age;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->vel = vel;
    _payload->cog = cog;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->dgps_numch = dgps_numch;
    _payload->yaw = yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS2_RAW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RAW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RAW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_raw_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->eph, _payload->epv, _payload->vel, _payload->cog, _payload->satellites_visible, _payload->dgps_numch, _payload->dgps_age, _payload->yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps2_raw_t _payload;

    _payload.time_usec = time_usec;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.dgps_age = dgps_age;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.vel = vel;
    _payload.cog = cog;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.dgps_numch = dgps_numch;
    _payload.yaw = yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS2_RAW,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_raw_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS2_RAW,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RAW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS2_RAW unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps2_raw_decode(fmav_gps2_raw_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GPS2_RAW_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS2_RAW  124

#define mavlink_gps2_raw_t  fmav_gps2_raw_t

#define MAVLINK_MSG_ID_GPS2_RAW_LEN  37
#define MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN  35
#define MAVLINK_MSG_ID_124_LEN  37
#define MAVLINK_MSG_ID_124_MIN_LEN  35

#define MAVLINK_MSG_ID_GPS2_RAW_CRC  87
#define MAVLINK_MSG_ID_124_CRC  87




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps2_raw_pack(
        msg, sysid, compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_raw_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw)
{
    return fmav_msg_gps2_raw_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps2_raw_decode(const mavlink_message_t* msg, mavlink_gps2_raw_t* payload)
{
    fmav_msg_gps2_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS2_RAW_H

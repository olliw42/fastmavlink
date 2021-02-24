//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ADSB_VEHICLE_H
#define FASTMAVLINK_MSG_ADSB_VEHICLE_H


//----------------------------------------
//-- Message ADSB_VEHICLE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_adsb_vehicle_t {
    uint32_t ICAO_address;
    int32_t lat;
    int32_t lon;
    int32_t altitude;
    uint16_t heading;
    uint16_t hor_velocity;
    int16_t ver_velocity;
    uint16_t flags;
    uint16_t squawk;
    uint8_t altitude_type;
    char callsign[9];
    uint8_t emitter_type;
    uint8_t tslc;
}) fmav_adsb_vehicle_t;


#define FASTMAVLINK_MSG_ID_ADSB_VEHICLE  246


#define FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MIN  38
#define FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN  38
#define FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA  184

#define FASTMAVLINK_MSG_ID_246_LEN_MIN  38
#define FASTMAVLINK_MSG_ID_246_LEN_MAX  38
#define FASTMAVLINK_MSG_ID_246_LEN  38
#define FASTMAVLINK_MSG_ID_246_CRCEXTRA  184

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN  9

#define FASTMAVLINK_MSG_ADSB_VEHICLE_FLAGS  0
#define FASTMAVLINK_MSG_ADSB_VEHICLE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ADSB_VEHICLE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ADSB_VEHICLE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_adsb_vehicle_t* _payload = (fmav_adsb_vehicle_t*)msg->payload;

    _payload->ICAO_address = ICAO_address;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->altitude = altitude;
    _payload->heading = heading;
    _payload->hor_velocity = hor_velocity;
    _payload->ver_velocity = ver_velocity;
    _payload->flags = flags;
    _payload->squawk = squawk;
    _payload->altitude_type = altitude_type;
    _payload->emitter_type = emitter_type;
    _payload->tslc = tslc;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ADSB_VEHICLE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_adsb_vehicle_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_adsb_vehicle_pack(
        msg, sysid, compid,
        _payload->ICAO_address, _payload->lat, _payload->lon, _payload->altitude_type, _payload->altitude, _payload->heading, _payload->hor_velocity, _payload->ver_velocity, _payload->callsign, _payload->emitter_type, _payload->tslc, _payload->flags, _payload->squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_adsb_vehicle_t* _payload = (fmav_adsb_vehicle_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ICAO_address = ICAO_address;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->altitude = altitude;
    _payload->heading = heading;
    _payload->hor_velocity = hor_velocity;
    _payload->ver_velocity = ver_velocity;
    _payload->flags = flags;
    _payload->squawk = squawk;
    _payload->altitude_type = altitude_type;
    _payload->emitter_type = emitter_type;
    _payload->tslc = tslc;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ADSB_VEHICLE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ADSB_VEHICLE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_adsb_vehicle_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_adsb_vehicle_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_adsb_vehicle_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ICAO_address, _payload->lat, _payload->lon, _payload->altitude_type, _payload->altitude, _payload->heading, _payload->hor_velocity, _payload->ver_velocity, _payload->callsign, _payload->emitter_type, _payload->tslc, _payload->flags, _payload->squawk,
        _status);
}


//----------------------------------------
//-- Message ADSB_VEHICLE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_adsb_vehicle_decode(fmav_adsb_vehicle_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ADSB_VEHICLE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ADSB_VEHICLE  246

#define mavlink_adsb_vehicle_t  fmav_adsb_vehicle_t

#define MAVLINK_MSG_ID_ADSB_VEHICLE_LEN  38
#define MAVLINK_MSG_ID_ADSB_VEHICLE_MIN_LEN  38
#define MAVLINK_MSG_ID_246_LEN  38
#define MAVLINK_MSG_ID_246_MIN_LEN  38

#define MAVLINK_MSG_ID_ADSB_VEHICLE_CRC  184
#define MAVLINK_MSG_ID_246_CRC  184

#define MAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN 9


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_adsb_vehicle_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_adsb_vehicle_pack(
        msg, sysid, compid,
        ICAO_address, lat, lon, altitude_type, altitude, heading, hor_velocity, ver_velocity, callsign, emitter_type, tslc, flags, squawk,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_adsb_vehicle_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, int16_t ver_velocity, const char* callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
    return fmav_msg_adsb_vehicle_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ICAO_address, lat, lon, altitude_type, altitude, heading, hor_velocity, ver_velocity, callsign, emitter_type, tslc, flags, squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_adsb_vehicle_decode(const mavlink_message_t* msg, mavlink_adsb_vehicle_t* payload)
{
    fmav_msg_adsb_vehicle_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ADSB_VEHICLE_H

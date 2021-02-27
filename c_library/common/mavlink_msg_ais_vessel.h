//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AIS_VESSEL_H
#define FASTMAVLINK_MSG_AIS_VESSEL_H


//----------------------------------------
//-- Message AIS_VESSEL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ais_vessel_t {
    uint32_t MMSI;
    int32_t lat;
    int32_t lon;
    uint16_t COG;
    uint16_t heading;
    uint16_t velocity;
    uint16_t dimension_bow;
    uint16_t dimension_stern;
    uint16_t tslc;
    uint16_t flags;
    int8_t turn_rate;
    uint8_t navigational_status;
    uint8_t type;
    uint8_t dimension_port;
    uint8_t dimension_starboard;
    char callsign[7];
    char name[20];
}) fmav_ais_vessel_t;


#define FASTMAVLINK_MSG_ID_AIS_VESSEL  301


#define FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MIN  58
#define FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX  58
#define FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN  58
#define FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA  243

#define FASTMAVLINK_MSG_ID_301_LEN_MIN  58
#define FASTMAVLINK_MSG_ID_301_LEN_MAX  58
#define FASTMAVLINK_MSG_ID_301_LEN  58
#define FASTMAVLINK_MSG_ID_301_CRCEXTRA  243

#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_LEN  7
#define FASTMAVLINK_MSG_AIS_VESSEL_FIELD_NAME_LEN  20

#define FASTMAVLINK_MSG_AIS_VESSEL_FLAGS  0
#define FASTMAVLINK_MSG_AIS_VESSEL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIS_VESSEL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIS_VESSEL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_301_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_301_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message AIS_VESSEL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t* _payload = (fmav_ais_vessel_t*)msg->payload;

    _payload->MMSI = MMSI;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->COG = COG;
    _payload->heading = heading;
    _payload->velocity = velocity;
    _payload->dimension_bow = dimension_bow;
    _payload->dimension_stern = dimension_stern;
    _payload->tslc = tslc;
    _payload->flags = flags;
    _payload->turn_rate = turn_rate;
    _payload->navigational_status = navigational_status;
    _payload->type = type;
    _payload->dimension_port = dimension_port;
    _payload->dimension_starboard = dimension_starboard;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload->name), name, sizeof(char)*20);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AIS_VESSEL;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ais_vessel_pack(
        msg, sysid, compid,
        _payload->MMSI, _payload->lat, _payload->lon, _payload->COG, _payload->heading, _payload->velocity, _payload->turn_rate, _payload->navigational_status, _payload->type, _payload->dimension_bow, _payload->dimension_stern, _payload->dimension_port, _payload->dimension_starboard, _payload->callsign, _payload->name, _payload->tslc, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t* _payload = (fmav_ais_vessel_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->MMSI = MMSI;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->COG = COG;
    _payload->heading = heading;
    _payload->velocity = velocity;
    _payload->dimension_bow = dimension_bow;
    _payload->dimension_stern = dimension_stern;
    _payload->tslc = tslc;
    _payload->flags = flags;
    _payload->turn_rate = turn_rate;
    _payload->navigational_status = navigational_status;
    _payload->type = type;
    _payload->dimension_port = dimension_port;
    _payload->dimension_starboard = dimension_starboard;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload->name), name, sizeof(char)*20);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AIS_VESSEL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AIS_VESSEL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AIS_VESSEL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ais_vessel_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->MMSI, _payload->lat, _payload->lon, _payload->COG, _payload->heading, _payload->velocity, _payload->turn_rate, _payload->navigational_status, _payload->type, _payload->dimension_bow, _payload->dimension_stern, _payload->dimension_port, _payload->dimension_starboard, _payload->callsign, _payload->name, _payload->tslc, _payload->flags,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags,
    fmav_status_t* _status)
{
    fmav_ais_vessel_t _payload;

    _payload.MMSI = MMSI;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.COG = COG;
    _payload.heading = heading;
    _payload.velocity = velocity;
    _payload.dimension_bow = dimension_bow;
    _payload.dimension_stern = dimension_stern;
    _payload.tslc = tslc;
    _payload.flags = flags;
    _payload.turn_rate = turn_rate;
    _payload.navigational_status = navigational_status;
    _payload.type = type;
    _payload.dimension_port = dimension_port;
    _payload.dimension_starboard = dimension_starboard;
    memcpy(&(_payload.callsign), callsign, sizeof(char)*7);
    memcpy(&(_payload.name), name, sizeof(char)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AIS_VESSEL,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ais_vessel_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ais_vessel_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AIS_VESSEL,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIS_VESSEL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AIS_VESSEL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ais_vessel_decode(fmav_ais_vessel_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AIS_VESSEL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIS_VESSEL  301

#define mavlink_ais_vessel_t  fmav_ais_vessel_t

#define MAVLINK_MSG_ID_AIS_VESSEL_LEN  58
#define MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN  58
#define MAVLINK_MSG_ID_301_LEN  58
#define MAVLINK_MSG_ID_301_MIN_LEN  58

#define MAVLINK_MSG_ID_AIS_VESSEL_CRC  243
#define MAVLINK_MSG_ID_301_CRC  243

#define MAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_LEN 7
#define MAVLINK_MSG_AIS_VESSEL_FIELD_NAME_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ais_vessel_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ais_vessel_pack(
        msg, sysid, compid,
        MMSI, lat, lon, COG, heading, velocity, turn_rate, navigational_status, type, dimension_bow, dimension_stern, dimension_port, dimension_starboard, callsign, name, tslc, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ais_vessel_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char* callsign, const char* name, uint16_t tslc, uint16_t flags)
{
    return fmav_msg_ais_vessel_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        MMSI, lat, lon, COG, heading, velocity, turn_rate, navigational_status, type, dimension_bow, dimension_stern, dimension_port, dimension_starboard, callsign, name, tslc, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ais_vessel_decode(const mavlink_message_t* msg, mavlink_ais_vessel_t* payload)
{
    fmav_msg_ais_vessel_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIS_VESSEL_H

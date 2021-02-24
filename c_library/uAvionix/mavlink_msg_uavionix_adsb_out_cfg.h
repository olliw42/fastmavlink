//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_out_cfg_t {
    uint32_t ICAO;
    uint16_t stallSpeed;
    char callsign[9];
    uint8_t emitterType;
    uint8_t aircraftSize;
    uint8_t gpsOffsetLat;
    uint8_t gpsOffsetLon;
    uint8_t rfSelect;
}) fmav_uavionix_adsb_out_cfg_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG  10001


#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MIN  20
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN  20
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA  209

#define FASTMAVLINK_MSG_ID_10001_LEN_MIN  20
#define FASTMAVLINK_MSG_ID_10001_LEN_MAX  20
#define FASTMAVLINK_MSG_ID_10001_LEN  20
#define FASTMAVLINK_MSG_ID_10001_CRCEXTRA  209

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN  9

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_t* _payload = (fmav_uavionix_adsb_out_cfg_t*)msg->payload;

    _payload->ICAO = ICAO;
    _payload->stallSpeed = stallSpeed;
    _payload->emitterType = emitterType;
    _payload->aircraftSize = aircraftSize;
    _payload->gpsOffsetLat = gpsOffsetLat;
    _payload->gpsOffsetLon = gpsOffsetLon;
    _payload->rfSelect = rfSelect;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack(
        msg, sysid, compid,
        _payload->ICAO, _payload->callsign, _payload->emitterType, _payload->aircraftSize, _payload->gpsOffsetLat, _payload->gpsOffsetLon, _payload->stallSpeed, _payload->rfSelect,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_t* _payload = (fmav_uavionix_adsb_out_cfg_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ICAO = ICAO;
    _payload->stallSpeed = stallSpeed;
    _payload->emitterType = emitterType;
    _payload->aircraftSize = aircraftSize;
    _payload->gpsOffsetLat = gpsOffsetLat;
    _payload->gpsOffsetLon = gpsOffsetLon;
    _payload->rfSelect = rfSelect;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ICAO, _payload->callsign, _payload->emitterType, _payload->aircraftSize, _payload->gpsOffsetLat, _payload->gpsOffsetLon, _payload->stallSpeed, _payload->rfSelect,
        _status);
}


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_cfg_decode(fmav_uavionix_adsb_out_cfg_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG  10001

#define mavlink_uavionix_adsb_out_cfg_t  fmav_uavionix_adsb_out_cfg_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN  20
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN  20
#define MAVLINK_MSG_ID_10001_LEN  20
#define MAVLINK_MSG_ID_10001_MIN_LEN  20

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC  209
#define MAVLINK_MSG_ID_10001_CRC  209

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN 9


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_cfg_pack(
        msg, sysid, compid,
        ICAO, callsign, emitterType, aircraftSize, gpsOffsetLat, gpsOffsetLon, stallSpeed, rfSelect,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ICAO, callsign, emitterType, aircraftSize, gpsOffsetLat, gpsOffsetLon, stallSpeed, rfSelect,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_out_cfg_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_cfg_t* payload)
{
    fmav_msg_uavionix_adsb_out_cfg_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_H

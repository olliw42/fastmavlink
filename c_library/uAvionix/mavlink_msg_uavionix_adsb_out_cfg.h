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

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA  209

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FRAME_LEN_MAX  45

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_NUM  9 // number of elements in array
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN  9 // length of array = number of bytes

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_ICAO_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_STALLSPEED_OFS  4
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_OFS  6
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_EMITTERTYPE_OFS  15
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_AIRCRAFTSIZE_OFS  16
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_GPSOFFSETLAT_OFS  17
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_GPSOFFSETLON_OFS  18
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_RFSELECT_OFS  19


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_t _payload;

    _payload.ICAO = ICAO;
    _payload.stallSpeed = stallSpeed;
    _payload.emitterType = emitterType;
    _payload.aircraftSize = aircraftSize;
    _payload.gpsOffsetLat = gpsOffsetLat;
    _payload.gpsOffsetLon = gpsOffsetLon;
    _payload.rfSelect = rfSelect;
    memcpy(&(_payload.callsign), callsign, sizeof(char)*9);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_CFG unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_uavionix_adsb_out_cfg_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_uavionix_adsb_out_cfg_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_cfg_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_cfg_decode(fmav_uavionix_adsb_out_cfg_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavionix_adsb_out_cfg_get_field_ICAO(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_get_field_stallSpeed(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_cfg_get_field_emitterType(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_cfg_get_field_aircraftSize(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_cfg_get_field_gpsOffsetLat(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_cfg_get_field_gpsOffsetLon(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_cfg_get_field_rfSelect(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_uavionix_adsb_out_cfg_get_field_callsign_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_uavionix_adsb_out_cfg_get_field_callsign(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_NUM) return 0;
    return ((char*)&(msg->payload[6]))[index];
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

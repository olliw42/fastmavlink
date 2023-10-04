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

// fields are ordered, as they appear on the wire
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
//-- Message UAVIONIX_ADSB_OUT_CFG pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_t* _payload = (fmav_uavionix_adsb_out_cfg_t*)_msg->payload;

    _payload->ICAO = ICAO;
    _payload->stallSpeed = stallSpeed;
    _payload->emitterType = emitterType;
    _payload->aircraftSize = aircraftSize;
    _payload->gpsOffsetLat = gpsOffsetLat;
    _payload->gpsOffsetLon = gpsOffsetLon;
    _payload->rfSelect = rfSelect;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack(
        _msg, sysid, compid,
        _payload->ICAO, _payload->callsign, _payload->emitterType, _payload->aircraftSize, _payload->gpsOffsetLat, _payload->gpsOffsetLon, _payload->stallSpeed, _payload->rfSelect,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_cfg_t* _payload = (fmav_uavionix_adsb_out_cfg_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ICAO = ICAO;
    _payload->stallSpeed = stallSpeed;
    _payload->emitterType = emitterType;
    _payload->aircraftSize = aircraftSize;
    _payload->gpsOffsetLat = gpsOffsetLat;
    _payload->gpsOffsetLon = gpsOffsetLon;
    _payload->rfSelect = rfSelect;
    memcpy(&(_payload->callsign), callsign, sizeof(char)*9);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_cfg_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_cfg_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
        _buf, sysid, compid,
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
//-- Message UAVIONIX_ADSB_OUT_CFG decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_cfg_decode(fmav_uavionix_adsb_out_cfg_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_PAYLOAD_LEN_MAX);
#endif
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
    mavlink_message_t* _msg,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_cfg_pack(
        _msg, sysid, compid,
        ICAO, callsign, emitterType, aircraftSize, gpsOffsetLat, gpsOffsetLon, stallSpeed, rfSelect,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_out_cfg_t* _payload)
{
    return mavlink_msg_uavionix_adsb_out_cfg_pack(
        sysid,
        compid,
        _msg,
        _payload->ICAO, _payload->callsign, _payload->emitterType, _payload->aircraftSize, _payload->gpsOffsetLat, _payload->gpsOffsetLon, _payload->stallSpeed, _payload->rfSelect);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ICAO, const char* callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
    return fmav_msg_uavionix_adsb_out_cfg_pack_to_frame_buf(
        (uint8_t*)_buf,
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_out_dynamic_t {
    uint32_t utcTime;
    int32_t gpsLat;
    int32_t gpsLon;
    int32_t gpsAlt;
    int32_t baroAltMSL;
    uint32_t accuracyHor;
    uint16_t accuracyVert;
    uint16_t accuracyVel;
    int16_t velVert;
    int16_t velNS;
    int16_t VelEW;
    uint16_t state;
    uint16_t squawk;
    uint8_t gpsFix;
    uint8_t numSats;
    uint8_t emergencyStatus;
}) fmav_uavionix_adsb_out_dynamic_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC  10002

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA  186

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FRAME_LEN_MAX  66



#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_UTCTIME_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_GPSLAT_OFS  4
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_GPSLON_OFS  8
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_GPSALT_OFS  12
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_BAROALTMSL_OFS  16
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_ACCURACYHOR_OFS  20
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_ACCURACYVERT_OFS  24
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_ACCURACYVEL_OFS  26
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_VELVERT_OFS  28
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_VELNS_OFS  30
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_VELEW_OFS  32
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_STATE_OFS  34
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_SQUAWK_OFS  36
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_GPSFIX_OFS  38
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_NUMSATS_OFS  39
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FIELD_EMERGENCYSTATUS_OFS  40


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_dynamic_t* _payload = (fmav_uavionix_adsb_out_dynamic_t*)_msg->payload;

    _payload->utcTime = utcTime;
    _payload->gpsLat = gpsLat;
    _payload->gpsLon = gpsLon;
    _payload->gpsAlt = gpsAlt;
    _payload->baroAltMSL = baroAltMSL;
    _payload->accuracyHor = accuracyHor;
    _payload->accuracyVert = accuracyVert;
    _payload->accuracyVel = accuracyVel;
    _payload->velVert = velVert;
    _payload->velNS = velNS;
    _payload->VelEW = VelEW;
    _payload->state = state;
    _payload->squawk = squawk;
    _payload->gpsFix = gpsFix;
    _payload->numSats = numSats;
    _payload->emergencyStatus = emergencyStatus;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_dynamic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack(
        _msg, sysid, compid,
        _payload->utcTime, _payload->gpsLat, _payload->gpsLon, _payload->gpsAlt, _payload->gpsFix, _payload->numSats, _payload->baroAltMSL, _payload->accuracyHor, _payload->accuracyVert, _payload->accuracyVel, _payload->velVert, _payload->velNS, _payload->VelEW, _payload->emergencyStatus, _payload->state, _payload->squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_dynamic_t* _payload = (fmav_uavionix_adsb_out_dynamic_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->utcTime = utcTime;
    _payload->gpsLat = gpsLat;
    _payload->gpsLon = gpsLon;
    _payload->gpsAlt = gpsAlt;
    _payload->baroAltMSL = baroAltMSL;
    _payload->accuracyHor = accuracyHor;
    _payload->accuracyVert = accuracyVert;
    _payload->accuracyVel = accuracyVel;
    _payload->velVert = velVert;
    _payload->velNS = velNS;
    _payload->VelEW = VelEW;
    _payload->state = state;
    _payload->squawk = squawk;
    _payload->gpsFix = gpsFix;
    _payload->numSats = numSats;
    _payload->emergencyStatus = emergencyStatus;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_dynamic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->utcTime, _payload->gpsLat, _payload->gpsLon, _payload->gpsAlt, _payload->gpsFix, _payload->numSats, _payload->baroAltMSL, _payload->accuracyHor, _payload->accuracyVert, _payload->accuracyVel, _payload->velVert, _payload->velNS, _payload->VelEW, _payload->emergencyStatus, _payload->state, _payload->squawk,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_dynamic_t _payload;

    _payload.utcTime = utcTime;
    _payload.gpsLat = gpsLat;
    _payload.gpsLon = gpsLon;
    _payload.gpsAlt = gpsAlt;
    _payload.baroAltMSL = baroAltMSL;
    _payload.accuracyHor = accuracyHor;
    _payload.accuracyVert = accuracyVert;
    _payload.accuracyVel = accuracyVel;
    _payload.velVert = velVert;
    _payload.velNS = velNS;
    _payload.VelEW = VelEW;
    _payload.state = state;
    _payload.squawk = squawk;
    _payload.gpsFix = gpsFix;
    _payload.numSats = numSats;
    _payload.emergencyStatus = emergencyStatus;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_dynamic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_dynamic_decode(fmav_uavionix_adsb_out_dynamic_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_utcTime(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_gpsLat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_gpsLon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_gpsAlt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_baroAltMSL(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavionix_adsb_out_dynamic_get_field_accuracyHor(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_accuracyVert(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_accuracyVel(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_velVert(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_velNS(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_VelEW(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_state(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_get_field_squawk(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_dynamic_get_field_gpsFix(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_dynamic_get_field_numSats(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[39]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_out_dynamic_get_field_emergencyStatus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC  10002

#define mavlink_uavionix_adsb_out_dynamic_t  fmav_uavionix_adsb_out_dynamic_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN  41
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN  41
#define MAVLINK_MSG_ID_10002_LEN  41
#define MAVLINK_MSG_ID_10002_MIN_LEN  41

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC  186
#define MAVLINK_MSG_ID_10002_CRC  186




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_dynamic_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_dynamic_pack(
        _msg, sysid, compid,
        utcTime, gpsLat, gpsLon, gpsAlt, gpsFix, numSats, baroAltMSL, accuracyHor, accuracyVert, accuracyVel, velVert, velNS, VelEW, emergencyStatus, state, squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_dynamic_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_out_dynamic_t* _payload)
{
    return mavlink_msg_uavionix_adsb_out_dynamic_pack(
        sysid,
        compid,
        _msg,
        _payload->utcTime, _payload->gpsLat, _payload->gpsLon, _payload->gpsAlt, _payload->gpsFix, _payload->numSats, _payload->baroAltMSL, _payload->accuracyHor, _payload->accuracyVert, _payload->accuracyVel, _payload->velVert, _payload->velNS, _payload->VelEW, _payload->emergencyStatus, _payload->state, _payload->squawk);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_dynamic_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        utcTime, gpsLat, gpsLon, gpsAlt, gpsFix, numSats, baroAltMSL, accuracyHor, accuracyVert, accuracyVel, velVert, velNS, VelEW, emergencyStatus, state, squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_out_dynamic_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_dynamic_t* payload)
{
    fmav_msg_uavionix_adsb_out_dynamic_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_H

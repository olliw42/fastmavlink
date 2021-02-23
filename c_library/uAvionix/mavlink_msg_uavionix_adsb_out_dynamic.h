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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MIN  41
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN  41
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA  186

#define FASTMAVLINK_MSG_ID_10002_LEN_MIN  41
#define FASTMAVLINK_MSG_ID_10002_LEN_MAX  41
#define FASTMAVLINK_MSG_ID_10002_LEN  41
#define FASTMAVLINK_MSG_ID_10002_CRCEXTRA  186



#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_dynamic_t* _payload = (fmav_uavionix_adsb_out_dynamic_t*)msg->payload;

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


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_dynamic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack(
        msg, sysid, compid,
        _payload->utcTime, _payload->gpsLat, _payload->gpsLon, _payload->gpsAlt, _payload->gpsFix, _payload->numSats, _payload->baroAltMSL, _payload->accuracyHor, _payload->accuracyVert, _payload->accuracyVel, _payload->velVert, _payload->velNS, _payload->VelEW, _payload->emergencyStatus, _payload->state, _payload->squawk,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_out_dynamic_t* _payload = (fmav_uavionix_adsb_out_dynamic_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_out_dynamic_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_out_dynamic_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->utcTime, _payload->gpsLat, _payload->gpsLon, _payload->gpsAlt, _payload->gpsFix, _payload->numSats, _payload->baroAltMSL, _payload->accuracyHor, _payload->accuracyVert, _payload->accuracyVel, _payload->velVert, _payload->velNS, _payload->VelEW, _payload->emergencyStatus, _payload->state, _payload->squawk,
        _status);
}


//----------------------------------------
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_dynamic_decode(fmav_uavionix_adsb_out_dynamic_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_out_dynamic_pack(
        msg, sysid, compid,
        utcTime, gpsLat, gpsLon, gpsAlt, gpsFix, numSats, baroAltMSL, accuracyHor, accuracyVert, accuracyVel, velVert, velNS, VelEW, emergencyStatus, state, squawk,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_out_dynamic_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
    return fmav_msg_uavionix_adsb_out_dynamic_pack_to_frame_buf(
        (uint8_t*)buf,
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

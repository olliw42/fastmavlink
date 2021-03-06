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
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC packing routines, for sending
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
//-- Message UAVIONIX_ADSB_OUT_DYNAMIC unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_uavionix_adsb_out_dynamic_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_uavionix_adsb_out_dynamic_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_dynamic_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_out_dynamic_decode(fmav_uavionix_adsb_out_dynamic_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_OUT_DYNAMIC_PAYLOAD_LEN_MAX);
    }
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

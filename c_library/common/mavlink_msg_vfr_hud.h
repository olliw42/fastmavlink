//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VFR_HUD_H
#define FASTMAVLINK_MSG_VFR_HUD_H


//----------------------------------------
//-- Message VFR_HUD
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vfr_hud_t {
    float airspeed;
    float groundspeed;
    float alt;
    float climb;
    int16_t heading;
    uint16_t throttle;
}) fmav_vfr_hud_t;


#define FASTMAVLINK_MSG_ID_VFR_HUD  74

#define FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA  20

#define FASTMAVLINK_MSG_VFR_HUD_FLAGS  0
#define FASTMAVLINK_MSG_VFR_HUD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VFR_HUD_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VFR_HUD_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_VFR_HUD_FIELD_AIRSPEED_OFS  0
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_GROUNDSPEED_OFS  4
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_ALT_OFS  8
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_CLIMB_OFS  12
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_HEADING_OFS  16
#define FASTMAVLINK_MSG_VFR_HUD_FIELD_THROTTLE_OFS  18


//----------------------------------------
//-- Message VFR_HUD packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t* _payload = (fmav_vfr_hud_t*)msg->payload;

    _payload->airspeed = airspeed;
    _payload->groundspeed = groundspeed;
    _payload->alt = alt;
    _payload->climb = climb;
    _payload->heading = heading;
    _payload->throttle = throttle;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VFR_HUD;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vfr_hud_pack(
        msg, sysid, compid,
        _payload->airspeed, _payload->groundspeed, _payload->heading, _payload->throttle, _payload->alt, _payload->climb,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t* _payload = (fmav_vfr_hud_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->airspeed = airspeed;
    _payload->groundspeed = groundspeed;
    _payload->alt = alt;
    _payload->climb = climb;
    _payload->heading = heading;
    _payload->throttle = throttle;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VFR_HUD;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VFR_HUD >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VFR_HUD >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vfr_hud_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->airspeed, _payload->groundspeed, _payload->heading, _payload->throttle, _payload->alt, _payload->climb,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb,
    fmav_status_t* _status)
{
    fmav_vfr_hud_t _payload;

    _payload.airspeed = airspeed;
    _payload.groundspeed = groundspeed;
    _payload.alt = alt;
    _payload.climb = climb;
    _payload.heading = heading;
    _payload.throttle = throttle;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VFR_HUD,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_vfr_hud_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VFR_HUD,
        FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VFR_HUD_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VFR_HUD unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_vfr_hud_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_vfr_hud_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vfr_hud_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vfr_hud_decode(fmav_vfr_hud_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VFR_HUD_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_airspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_groundspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_vfr_hud_get_field_climb(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_vfr_hud_get_field_heading(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vfr_hud_get_field_throttle(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VFR_HUD  74

#define mavlink_vfr_hud_t  fmav_vfr_hud_t

#define MAVLINK_MSG_ID_VFR_HUD_LEN  20
#define MAVLINK_MSG_ID_VFR_HUD_MIN_LEN  20
#define MAVLINK_MSG_ID_74_LEN  20
#define MAVLINK_MSG_ID_74_MIN_LEN  20

#define MAVLINK_MSG_ID_VFR_HUD_CRC  20
#define MAVLINK_MSG_ID_74_CRC  20




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vfr_hud_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vfr_hud_pack(
        msg, sysid, compid,
        airspeed, groundspeed, heading, throttle, alt, climb,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vfr_hud_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
{
    return fmav_msg_vfr_hud_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        airspeed, groundspeed, heading, throttle, alt, climb,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vfr_hud_decode(const mavlink_message_t* msg, mavlink_vfr_hud_t* payload)
{
    fmav_msg_vfr_hud_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VFR_HUD_H

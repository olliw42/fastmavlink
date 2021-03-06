//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_STATE_H
#define FASTMAVLINK_MSG_HIL_STATE_H


//----------------------------------------
//-- Message HIL_STATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_state_t {
    uint64_t time_usec;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
}) fmav_hil_state_t;


#define FASTMAVLINK_MSG_ID_HIL_STATE  90

#define FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX  56
#define FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA  183

#define FASTMAVLINK_MSG_HIL_STATE_FLAGS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_STATE_FRAME_LEN_MAX  81



#define FASTMAVLINK_MSG_HIL_STATE_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ROLL_OFS  8
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_PITCH_OFS  12
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YAW_OFS  16
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ROLLSPEED_OFS  20
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_PITCHSPEED_OFS  24
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YAWSPEED_OFS  28
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_LAT_OFS  32
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_LON_OFS  36
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ALT_OFS  40
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VX_OFS  44
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VY_OFS  46
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_VZ_OFS  48
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_XACC_OFS  50
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_YACC_OFS  52
#define FASTMAVLINK_MSG_HIL_STATE_FIELD_ZACC_OFS  54


//----------------------------------------
//-- Message HIL_STATE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_STATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t* _payload = (fmav_hil_state_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_STATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_STATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed, _payload->lat, _payload->lon, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->xacc, _payload->yacc, _payload->zacc,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc,
    fmav_status_t* _status)
{
    fmav_hil_state_t _payload;

    _payload.time_usec = time_usec;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.rollspeed = rollspeed;
    _payload.pitchspeed = pitchspeed;
    _payload.yawspeed = yawspeed;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.xacc = xacc;
    _payload.yacc = yacc;
    _payload.zacc = zacc;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_STATE,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_state_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_STATE,
        FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_STATE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_STATE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hil_state_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hil_state_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_state_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_state_decode(fmav_hil_state_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_STATE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_state_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_rollspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_pitchspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_state_get_field_yawspeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_hil_state_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vx(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vy(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[46]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_vz(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_xacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_yacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[52]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_state_get_field_zacc(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[54]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_STATE  90

#define mavlink_hil_state_t  fmav_hil_state_t

#define MAVLINK_MSG_ID_HIL_STATE_LEN  56
#define MAVLINK_MSG_ID_HIL_STATE_MIN_LEN  56
#define MAVLINK_MSG_ID_90_LEN  56
#define MAVLINK_MSG_ID_90_MIN_LEN  56

#define MAVLINK_MSG_ID_HIL_STATE_CRC  183
#define MAVLINK_MSG_ID_90_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_state_pack(
        msg, sysid, compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_state_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
    return fmav_msg_hil_state_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* payload)
{
    fmav_msg_hil_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_STATE_H

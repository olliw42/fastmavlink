//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SIMSTATE_H
#define FASTMAVLINK_MSG_SIMSTATE_H


//----------------------------------------
//-- Message SIMSTATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_simstate_t {
    float roll;
    float pitch;
    float yaw;
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
    int32_t lat;
    int32_t lng;
}) fmav_simstate_t;


#define FASTMAVLINK_MSG_ID_SIMSTATE  164


#define FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MIN  44
#define FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX  44
#define FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN  44
#define FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA  154

#define FASTMAVLINK_MSG_ID_164_LEN_MIN  44
#define FASTMAVLINK_MSG_ID_164_LEN_MAX  44
#define FASTMAVLINK_MSG_ID_164_LEN  44
#define FASTMAVLINK_MSG_ID_164_CRCEXTRA  154



#define FASTMAVLINK_MSG_SIMSTATE_FLAGS  0
#define FASTMAVLINK_MSG_SIMSTATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SIMSTATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SIMSTATE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_simstate_t* _payload = (fmav_simstate_t*)msg->payload;

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lng = lng;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SIMSTATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_simstate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_simstate_pack(
        msg, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_simstate_t* _payload = (fmav_simstate_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->xacc = xacc;
    _payload->yacc = yacc;
    _payload->zacc = zacc;
    _payload->xgyro = xgyro;
    _payload->ygyro = ygyro;
    _payload->zgyro = zgyro;
    _payload->lat = lat;
    _payload->lng = lng;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SIMSTATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SIMSTATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SIMSTATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIMSTATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_simstate_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_simstate_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_simstate_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lng,
        _status);
}


//----------------------------------------
//-- Message SIMSTATE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_simstate_decode(fmav_simstate_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SIMSTATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SIMSTATE  164

#define mavlink_simstate_t  fmav_simstate_t

#define MAVLINK_MSG_ID_SIMSTATE_LEN  44
#define MAVLINK_MSG_ID_SIMSTATE_MIN_LEN  44
#define MAVLINK_MSG_ID_164_LEN  44
#define MAVLINK_MSG_ID_164_MIN_LEN  44

#define MAVLINK_MSG_ID_SIMSTATE_CRC  154
#define MAVLINK_MSG_ID_164_CRC  154




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_simstate_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_simstate_pack(
        msg, sysid, compid,
        roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lng,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_simstate_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
    return fmav_msg_simstate_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_simstate_decode(const mavlink_message_t* msg, mavlink_simstate_t* payload)
{
    fmav_msg_simstate_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SIMSTATE_H

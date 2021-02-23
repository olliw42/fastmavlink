//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SIM_STATE_H
#define FASTMAVLINK_MSG_SIM_STATE_H


//----------------------------------------
//-- Message SIM_STATE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sim_state_t {
    float q1;
    float q2;
    float q3;
    float q4;
    float roll;
    float pitch;
    float yaw;
    float xacc;
    float yacc;
    float zacc;
    float xgyro;
    float ygyro;
    float zgyro;
    float lat;
    float lon;
    float alt;
    float std_dev_horz;
    float std_dev_vert;
    float vn;
    float ve;
    float vd;
}) fmav_sim_state_t;


#define FASTMAVLINK_MSG_ID_SIM_STATE  108


#define FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MIN  84
#define FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX  84
#define FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN  84
#define FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA  32

#define FASTMAVLINK_MSG_ID_108_LEN_MIN  84
#define FASTMAVLINK_MSG_ID_108_LEN_MAX  84
#define FASTMAVLINK_MSG_ID_108_LEN  84
#define FASTMAVLINK_MSG_ID_108_CRCEXTRA  32



#define FASTMAVLINK_MSG_SIM_STATE_FLAGS  0
#define FASTMAVLINK_MSG_SIM_STATE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SIM_STATE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SIM_STATE packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd,
    fmav_status_t* _status)
{
    fmav_sim_state_t* _payload = (fmav_sim_state_t*)msg->payload;

    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
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
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->std_dev_horz = std_dev_horz;
    _payload->std_dev_vert = std_dev_vert;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SIM_STATE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sim_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sim_state_pack(
        msg, sysid, compid,
        _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lon, _payload->alt, _payload->std_dev_horz, _payload->std_dev_vert, _payload->vn, _payload->ve, _payload->vd,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd,
    fmav_status_t* _status)
{
    fmav_sim_state_t* _payload = (fmav_sim_state_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->q1 = q1;
    _payload->q2 = q2;
    _payload->q3 = q3;
    _payload->q4 = q4;
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
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->std_dev_horz = std_dev_horz;
    _payload->std_dev_vert = std_dev_vert;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SIM_STATE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SIM_STATE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SIM_STATE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SIM_STATE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sim_state_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sim_state_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sim_state_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->q1, _payload->q2, _payload->q3, _payload->q4, _payload->roll, _payload->pitch, _payload->yaw, _payload->xacc, _payload->yacc, _payload->zacc, _payload->xgyro, _payload->ygyro, _payload->zgyro, _payload->lat, _payload->lon, _payload->alt, _payload->std_dev_horz, _payload->std_dev_vert, _payload->vn, _payload->ve, _payload->vd,
        _status);
}


//----------------------------------------
//-- Message SIM_STATE unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sim_state_decode(fmav_sim_state_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SIM_STATE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SIM_STATE  108

#define mavlink_sim_state_t  fmav_sim_state_t

#define MAVLINK_MSG_ID_SIM_STATE_LEN  84
#define MAVLINK_MSG_ID_SIM_STATE_MIN_LEN  84
#define MAVLINK_MSG_ID_108_LEN  84
#define MAVLINK_MSG_ID_108_MIN_LEN  84

#define MAVLINK_MSG_ID_SIM_STATE_CRC  32
#define MAVLINK_MSG_ID_108_CRC  32




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sim_state_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sim_state_pack(
        msg, sysid, compid,
        q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sim_state_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float q1, float q2, float q3, float q4, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float lat, float lon, float alt, float std_dev_horz, float std_dev_vert, float vn, float ve, float vd)
{
    return fmav_msg_sim_state_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        q1, q2, q3, q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, std_dev_horz, std_dev_vert, vn, ve, vd,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sim_state_decode(const mavlink_message_t* msg, mavlink_sim_state_t* payload)
{
    fmav_msg_sim_state_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SIM_STATE_H

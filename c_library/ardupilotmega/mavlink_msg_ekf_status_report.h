//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EKF_STATUS_REPORT_H
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_H


//----------------------------------------
//-- Message EKF_STATUS_REPORT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ekf_status_report_t {
    float velocity_variance;
    float pos_horiz_variance;
    float pos_vert_variance;
    float compass_variance;
    float terrain_alt_variance;
    uint16_t flags;
    float airspeed_variance;
}) fmav_ekf_status_report_t;


#define FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT  193

#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA  71

#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FLAGS  0
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FRAME_LEN_MAX  51



#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_VELOCITY_VARIANCE_OFS  0
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_POS_HORIZ_VARIANCE_OFS  4
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_POS_VERT_VARIANCE_OFS  8
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_COMPASS_VARIANCE_OFS  12
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_TERRAIN_ALT_VARIANCE_OFS  16
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_FLAGS_OFS  20
#define FASTMAVLINK_MSG_EKF_STATUS_REPORT_FIELD_AIRSPEED_VARIANCE_OFS  22


//----------------------------------------
//-- Message EKF_STATUS_REPORT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance,
    fmav_status_t* _status)
{
    fmav_ekf_status_report_t* _payload = (fmav_ekf_status_report_t*)msg->payload;

    _payload->velocity_variance = velocity_variance;
    _payload->pos_horiz_variance = pos_horiz_variance;
    _payload->pos_vert_variance = pos_vert_variance;
    _payload->compass_variance = compass_variance;
    _payload->terrain_alt_variance = terrain_alt_variance;
    _payload->flags = flags;
    _payload->airspeed_variance = airspeed_variance;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_status_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_status_report_pack(
        msg, sysid, compid,
        _payload->flags, _payload->velocity_variance, _payload->pos_horiz_variance, _payload->pos_vert_variance, _payload->compass_variance, _payload->terrain_alt_variance, _payload->airspeed_variance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance,
    fmav_status_t* _status)
{
    fmav_ekf_status_report_t* _payload = (fmav_ekf_status_report_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->velocity_variance = velocity_variance;
    _payload->pos_horiz_variance = pos_horiz_variance;
    _payload->pos_vert_variance = pos_vert_variance;
    _payload->compass_variance = compass_variance;
    _payload->terrain_alt_variance = terrain_alt_variance;
    _payload->flags = flags;
    _payload->airspeed_variance = airspeed_variance;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_status_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_status_report_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->flags, _payload->velocity_variance, _payload->pos_horiz_variance, _payload->pos_vert_variance, _payload->compass_variance, _payload->terrain_alt_variance, _payload->airspeed_variance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance,
    fmav_status_t* _status)
{
    fmav_ekf_status_report_t _payload;

    _payload.velocity_variance = velocity_variance;
    _payload.pos_horiz_variance = pos_horiz_variance;
    _payload.pos_vert_variance = pos_vert_variance;
    _payload.compass_variance = compass_variance;
    _payload.terrain_alt_variance = terrain_alt_variance;
    _payload.flags = flags;
    _payload.airspeed_variance = airspeed_variance;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_status_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message EKF_STATUS_REPORT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_ekf_status_report_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_ekf_status_report_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ekf_status_report_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ekf_status_report_decode(fmav_ekf_status_report_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_velocity_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_pos_horiz_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_pos_vert_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_compass_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_terrain_alt_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ekf_status_report_get_field_airspeed_variance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[22]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT  193

#define mavlink_ekf_status_report_t  fmav_ekf_status_report_t

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN  26
#define MAVLINK_MSG_ID_EKF_STATUS_REPORT_MIN_LEN  22
#define MAVLINK_MSG_ID_193_LEN  26
#define MAVLINK_MSG_ID_193_MIN_LEN  22

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC  71
#define MAVLINK_MSG_ID_193_CRC  71




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_status_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ekf_status_report_pack(
        msg, sysid, compid,
        flags, velocity_variance, pos_horiz_variance, pos_vert_variance, compass_variance, terrain_alt_variance, airspeed_variance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_status_report_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance)
{
    return fmav_msg_ekf_status_report_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        flags, velocity_variance, pos_horiz_variance, pos_vert_variance, compass_variance, terrain_alt_variance, airspeed_variance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ekf_status_report_decode(const mavlink_message_t* msg, mavlink_ekf_status_report_t* payload)
{
    fmav_msg_ekf_status_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EKF_STATUS_REPORT_H

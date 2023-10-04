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

// fields are ordered, as they appear on the wire
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
//-- Message EKF_STATUS_REPORT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance,
    fmav_status_t* _status)
{
    fmav_ekf_status_report_t* _payload = (fmav_ekf_status_report_t*)_msg->payload;

    _payload->velocity_variance = velocity_variance;
    _payload->pos_horiz_variance = pos_horiz_variance;
    _payload->pos_vert_variance = pos_vert_variance;
    _payload->compass_variance = compass_variance;
    _payload->terrain_alt_variance = terrain_alt_variance;
    _payload->flags = flags;
    _payload->airspeed_variance = airspeed_variance;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_status_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_status_report_pack(
        _msg, sysid, compid,
        _payload->flags, _payload->velocity_variance, _payload->pos_horiz_variance, _payload->pos_vert_variance, _payload->compass_variance, _payload->terrain_alt_variance, _payload->airspeed_variance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance,
    fmav_status_t* _status)
{
    fmav_ekf_status_report_t* _payload = (fmav_ekf_status_report_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->velocity_variance = velocity_variance;
    _payload->pos_horiz_variance = pos_horiz_variance;
    _payload->pos_vert_variance = pos_vert_variance;
    _payload->compass_variance = compass_variance;
    _payload->terrain_alt_variance = terrain_alt_variance;
    _payload->flags = flags;
    _payload->airspeed_variance = airspeed_variance;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EKF_STATUS_REPORT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EKF_STATUS_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ekf_status_report_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ekf_status_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ekf_status_report_pack_to_frame_buf(
        _buf, sysid, compid,
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
//-- Message EKF_STATUS_REPORT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ekf_status_report_decode(fmav_ekf_status_report_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_EKF_STATUS_REPORT_PAYLOAD_LEN_MAX);
#endif
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
    mavlink_message_t* _msg,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ekf_status_report_pack(
        _msg, sysid, compid,
        flags, velocity_variance, pos_horiz_variance, pos_vert_variance, compass_variance, terrain_alt_variance, airspeed_variance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_status_report_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_ekf_status_report_t* _payload)
{
    return mavlink_msg_ekf_status_report_pack(
        sysid,
        compid,
        _msg,
        _payload->flags, _payload->velocity_variance, _payload->pos_horiz_variance, _payload->pos_vert_variance, _payload->compass_variance, _payload->terrain_alt_variance, _payload->airspeed_variance);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ekf_status_report_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance, float airspeed_variance)
{
    return fmav_msg_ekf_status_report_pack_to_frame_buf(
        (uint8_t*)_buf,
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

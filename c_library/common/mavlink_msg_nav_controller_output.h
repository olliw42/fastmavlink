//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_nav_controller_output_t {
    float nav_roll;
    float nav_pitch;
    float alt_error;
    float aspd_error;
    float xtrack_error;
    int16_t nav_bearing;
    int16_t target_bearing;
    uint16_t wp_dist;
}) fmav_nav_controller_output_t;


#define FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  62

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA  183

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FLAGS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FRAME_LEN_MAX  51



#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_ROLL_OFS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_PITCH_OFS  4
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_ALT_ERROR_OFS  8
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_ASPD_ERROR_OFS  12
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_XTRACK_ERROR_OFS  16
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_NAV_BEARING_OFS  20
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_TARGET_BEARING_OFS  22
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FIELD_WP_DIST_OFS  24


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t* _payload = (fmav_nav_controller_output_t*)msg->payload;

    _payload->nav_roll = nav_roll;
    _payload->nav_pitch = nav_pitch;
    _payload->alt_error = alt_error;
    _payload->aspd_error = aspd_error;
    _payload->xtrack_error = xtrack_error;
    _payload->nav_bearing = nav_bearing;
    _payload->target_bearing = target_bearing;
    _payload->wp_dist = wp_dist;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_controller_output_pack(
        msg, sysid, compid,
        _payload->nav_roll, _payload->nav_pitch, _payload->nav_bearing, _payload->target_bearing, _payload->wp_dist, _payload->alt_error, _payload->aspd_error, _payload->xtrack_error,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t* _payload = (fmav_nav_controller_output_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->nav_roll = nav_roll;
    _payload->nav_pitch = nav_pitch;
    _payload->alt_error = alt_error;
    _payload->aspd_error = aspd_error;
    _payload->xtrack_error = xtrack_error;
    _payload->nav_bearing = nav_bearing;
    _payload->target_bearing = target_bearing;
    _payload->wp_dist = wp_dist;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_nav_controller_output_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->nav_roll, _payload->nav_pitch, _payload->nav_bearing, _payload->target_bearing, _payload->wp_dist, _payload->alt_error, _payload->aspd_error, _payload->xtrack_error,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error,
    fmav_status_t* _status)
{
    fmav_nav_controller_output_t _payload;

    _payload.nav_roll = nav_roll;
    _payload.nav_pitch = nav_pitch;
    _payload.alt_error = alt_error;
    _payload.aspd_error = aspd_error;
    _payload.xtrack_error = xtrack_error;
    _payload.nav_bearing = nav_bearing;
    _payload.target_bearing = target_bearing;
    _payload.wp_dist = wp_dist;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_nav_controller_output_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_nav_controller_output_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_nav_controller_output_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_controller_output_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_controller_output_decode(fmav_nav_controller_output_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_nav_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_nav_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_alt_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_aspd_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_nav_controller_output_get_field_xtrack_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_nav_controller_output_get_field_nav_bearing(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_nav_controller_output_get_field_target_bearing(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_nav_controller_output_get_field_wp_dist(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  62

#define mavlink_nav_controller_output_t  fmav_nav_controller_output_t

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN  26
#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MIN_LEN  26
#define MAVLINK_MSG_ID_62_LEN  26
#define MAVLINK_MSG_ID_62_MIN_LEN  26

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC  183
#define MAVLINK_MSG_ID_62_CRC  183




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_controller_output_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_nav_controller_output_pack(
        msg, sysid, compid,
        nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_nav_controller_output_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
    return fmav_msg_nav_controller_output_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_nav_controller_output_decode(const mavlink_message_t* msg, mavlink_nav_controller_output_t* payload)
{
    fmav_msg_nav_controller_output_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_H

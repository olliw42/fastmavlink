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


#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MIN  26
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX  26
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN  26
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_CRCEXTRA  183

#define FASTMAVLINK_MSG_ID_62_LEN_MIN  26
#define FASTMAVLINK_MSG_ID_62_LEN_MAX  26
#define FASTMAVLINK_MSG_ID_62_LEN  26
#define FASTMAVLINK_MSG_ID_62_CRCEXTRA  183



#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_FLAGS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_TARGET_COMPONENT_OFS  0


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
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message NAV_CONTROLLER_OUTPUT unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_nav_controller_output_decode(fmav_nav_controller_output_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_NAV_CONTROLLER_OUTPUT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

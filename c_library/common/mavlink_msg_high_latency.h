//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIGH_LATENCY_H
#define FASTMAVLINK_MSG_HIGH_LATENCY_H


//----------------------------------------
//-- Message HIGH_LATENCY
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_high_latency_t {
    uint32_t custom_mode;
    int32_t latitude;
    int32_t longitude;
    int16_t roll;
    int16_t pitch;
    uint16_t heading;
    int16_t heading_sp;
    int16_t altitude_amsl;
    int16_t altitude_sp;
    uint16_t wp_distance;
    uint8_t base_mode;
    uint8_t landed_state;
    int8_t throttle;
    uint8_t airspeed;
    uint8_t airspeed_sp;
    uint8_t groundspeed;
    int8_t climb_rate;
    uint8_t gps_nsat;
    uint8_t gps_fix_type;
    uint8_t battery_remaining;
    int8_t temperature;
    int8_t temperature_air;
    uint8_t failsafe;
    uint8_t wp_num;
}) fmav_high_latency_t;


#define FASTMAVLINK_MSG_ID_HIGH_LATENCY  234


#define FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MIN  40
#define FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN  40
#define FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA  150

#define FASTMAVLINK_MSG_ID_234_LEN_MIN  40
#define FASTMAVLINK_MSG_ID_234_LEN_MAX  40
#define FASTMAVLINK_MSG_ID_234_LEN  40
#define FASTMAVLINK_MSG_ID_234_CRCEXTRA  150



#define FASTMAVLINK_MSG_HIGH_LATENCY_FLAGS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIGH_LATENCY packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance,
    fmav_status_t* _status)
{
    fmav_high_latency_t* _payload = (fmav_high_latency_t*)msg->payload;

    _payload->custom_mode = custom_mode;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->heading = heading;
    _payload->heading_sp = heading_sp;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_sp = altitude_sp;
    _payload->wp_distance = wp_distance;
    _payload->base_mode = base_mode;
    _payload->landed_state = landed_state;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->climb_rate = climb_rate;
    _payload->gps_nsat = gps_nsat;
    _payload->gps_fix_type = gps_fix_type;
    _payload->battery_remaining = battery_remaining;
    _payload->temperature = temperature;
    _payload->temperature_air = temperature_air;
    _payload->failsafe = failsafe;
    _payload->wp_num = wp_num;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIGH_LATENCY;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency_pack(
        msg, sysid, compid,
        _payload->base_mode, _payload->custom_mode, _payload->landed_state, _payload->roll, _payload->pitch, _payload->heading, _payload->throttle, _payload->heading_sp, _payload->latitude, _payload->longitude, _payload->altitude_amsl, _payload->altitude_sp, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->climb_rate, _payload->gps_nsat, _payload->gps_fix_type, _payload->battery_remaining, _payload->temperature, _payload->temperature_air, _payload->failsafe, _payload->wp_num, _payload->wp_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance,
    fmav_status_t* _status)
{
    fmav_high_latency_t* _payload = (fmav_high_latency_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->custom_mode = custom_mode;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->heading = heading;
    _payload->heading_sp = heading_sp;
    _payload->altitude_amsl = altitude_amsl;
    _payload->altitude_sp = altitude_sp;
    _payload->wp_distance = wp_distance;
    _payload->base_mode = base_mode;
    _payload->landed_state = landed_state;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->climb_rate = climb_rate;
    _payload->gps_nsat = gps_nsat;
    _payload->gps_fix_type = gps_fix_type;
    _payload->battery_remaining = battery_remaining;
    _payload->temperature = temperature;
    _payload->temperature_air = temperature_air;
    _payload->failsafe = failsafe;
    _payload->wp_num = wp_num;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->base_mode, _payload->custom_mode, _payload->landed_state, _payload->roll, _payload->pitch, _payload->heading, _payload->throttle, _payload->heading_sp, _payload->latitude, _payload->longitude, _payload->altitude_amsl, _payload->altitude_sp, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->climb_rate, _payload->gps_nsat, _payload->gps_fix_type, _payload->battery_remaining, _payload->temperature, _payload->temperature_air, _payload->failsafe, _payload->wp_num, _payload->wp_distance,
        _status);
}


//----------------------------------------
//-- Message HIGH_LATENCY unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_high_latency_decode(fmav_high_latency_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIGH_LATENCY_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIGH_LATENCY  234

#define mavlink_high_latency_t  fmav_high_latency_t

#define MAVLINK_MSG_ID_HIGH_LATENCY_LEN  40
#define MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN  40
#define MAVLINK_MSG_ID_234_LEN  40
#define MAVLINK_MSG_ID_234_MIN_LEN  40

#define MAVLINK_MSG_ID_HIGH_LATENCY_CRC  150
#define MAVLINK_MSG_ID_234_CRC  150




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_high_latency_pack(
        msg, sysid, compid,
        base_mode, custom_mode, landed_state, roll, pitch, heading, throttle, heading_sp, latitude, longitude, altitude_amsl, altitude_sp, airspeed, airspeed_sp, groundspeed, climb_rate, gps_nsat, gps_fix_type, battery_remaining, temperature, temperature_air, failsafe, wp_num, wp_distance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
    return fmav_msg_high_latency_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        base_mode, custom_mode, landed_state, roll, pitch, heading, throttle, heading_sp, latitude, longitude, altitude_amsl, altitude_sp, airspeed, airspeed_sp, groundspeed, climb_rate, gps_nsat, gps_fix_type, battery_remaining, temperature, temperature_air, failsafe, wp_num, wp_distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_high_latency_decode(const mavlink_message_t* msg, mavlink_high_latency_t* payload)
{
    fmav_msg_high_latency_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIGH_LATENCY_H

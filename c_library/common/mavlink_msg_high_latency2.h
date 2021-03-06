//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIGH_LATENCY2_H
#define FASTMAVLINK_MSG_HIGH_LATENCY2_H


//----------------------------------------
//-- Message HIGH_LATENCY2
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_high_latency2_t {
    uint32_t timestamp;
    int32_t latitude;
    int32_t longitude;
    uint16_t custom_mode;
    int16_t altitude;
    int16_t target_altitude;
    uint16_t target_distance;
    uint16_t wp_num;
    uint16_t failure_flags;
    uint8_t type;
    uint8_t autopilot;
    uint8_t heading;
    uint8_t target_heading;
    uint8_t throttle;
    uint8_t airspeed;
    uint8_t airspeed_sp;
    uint8_t groundspeed;
    uint8_t windspeed;
    uint8_t wind_heading;
    uint8_t eph;
    uint8_t epv;
    int8_t temperature_air;
    int8_t climb_rate;
    int8_t battery;
    int8_t custom0;
    int8_t custom1;
    int8_t custom2;
}) fmav_high_latency2_t;


#define FASTMAVLINK_MSG_ID_HIGH_LATENCY2  235

#define FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA  179

#define FASTMAVLINK_MSG_HIGH_LATENCY2_FLAGS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY2_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIGH_LATENCY2_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_LATITUDE_OFS  4
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_LONGITUDE_OFS  8
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_CUSTOM_MODE_OFS  12
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_ALTITUDE_OFS  14
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TARGET_ALTITUDE_OFS  16
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TARGET_DISTANCE_OFS  18
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_WP_NUM_OFS  20
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_FAILURE_FLAGS_OFS  22
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TYPE_OFS  24
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_AUTOPILOT_OFS  25
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_HEADING_OFS  26
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TARGET_HEADING_OFS  27
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_THROTTLE_OFS  28
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_AIRSPEED_OFS  29
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_AIRSPEED_SP_OFS  30
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_GROUNDSPEED_OFS  31
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_WINDSPEED_OFS  32
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_WIND_HEADING_OFS  33
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_EPH_OFS  34
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_EPV_OFS  35
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_TEMPERATURE_AIR_OFS  36
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_CLIMB_RATE_OFS  37
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_BATTERY_OFS  38
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_CUSTOM0_OFS  39
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_CUSTOM1_OFS  40
#define FASTMAVLINK_MSG_HIGH_LATENCY2_FIELD_CUSTOM2_OFS  41


//----------------------------------------
//-- Message HIGH_LATENCY2 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2,
    fmav_status_t* _status)
{
    fmav_high_latency2_t* _payload = (fmav_high_latency2_t*)msg->payload;

    _payload->timestamp = timestamp;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->custom_mode = custom_mode;
    _payload->altitude = altitude;
    _payload->target_altitude = target_altitude;
    _payload->target_distance = target_distance;
    _payload->wp_num = wp_num;
    _payload->failure_flags = failure_flags;
    _payload->type = type;
    _payload->autopilot = autopilot;
    _payload->heading = heading;
    _payload->target_heading = target_heading;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->windspeed = windspeed;
    _payload->wind_heading = wind_heading;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->temperature_air = temperature_air;
    _payload->climb_rate = climb_rate;
    _payload->battery = battery;
    _payload->custom0 = custom0;
    _payload->custom1 = custom1;
    _payload->custom2 = custom2;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIGH_LATENCY2;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency2_pack(
        msg, sysid, compid,
        _payload->timestamp, _payload->type, _payload->autopilot, _payload->custom_mode, _payload->latitude, _payload->longitude, _payload->altitude, _payload->target_altitude, _payload->heading, _payload->target_heading, _payload->target_distance, _payload->throttle, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->windspeed, _payload->wind_heading, _payload->eph, _payload->epv, _payload->temperature_air, _payload->climb_rate, _payload->battery, _payload->wp_num, _payload->failure_flags, _payload->custom0, _payload->custom1, _payload->custom2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2,
    fmav_status_t* _status)
{
    fmav_high_latency2_t* _payload = (fmav_high_latency2_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->custom_mode = custom_mode;
    _payload->altitude = altitude;
    _payload->target_altitude = target_altitude;
    _payload->target_distance = target_distance;
    _payload->wp_num = wp_num;
    _payload->failure_flags = failure_flags;
    _payload->type = type;
    _payload->autopilot = autopilot;
    _payload->heading = heading;
    _payload->target_heading = target_heading;
    _payload->throttle = throttle;
    _payload->airspeed = airspeed;
    _payload->airspeed_sp = airspeed_sp;
    _payload->groundspeed = groundspeed;
    _payload->windspeed = windspeed;
    _payload->wind_heading = wind_heading;
    _payload->eph = eph;
    _payload->epv = epv;
    _payload->temperature_air = temperature_air;
    _payload->climb_rate = climb_rate;
    _payload->battery = battery;
    _payload->custom0 = custom0;
    _payload->custom1 = custom1;
    _payload->custom2 = custom2;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY2;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY2 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIGH_LATENCY2 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_high_latency2_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->timestamp, _payload->type, _payload->autopilot, _payload->custom_mode, _payload->latitude, _payload->longitude, _payload->altitude, _payload->target_altitude, _payload->heading, _payload->target_heading, _payload->target_distance, _payload->throttle, _payload->airspeed, _payload->airspeed_sp, _payload->groundspeed, _payload->windspeed, _payload->wind_heading, _payload->eph, _payload->epv, _payload->temperature_air, _payload->climb_rate, _payload->battery, _payload->wp_num, _payload->failure_flags, _payload->custom0, _payload->custom1, _payload->custom2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2,
    fmav_status_t* _status)
{
    fmav_high_latency2_t _payload;

    _payload.timestamp = timestamp;
    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.custom_mode = custom_mode;
    _payload.altitude = altitude;
    _payload.target_altitude = target_altitude;
    _payload.target_distance = target_distance;
    _payload.wp_num = wp_num;
    _payload.failure_flags = failure_flags;
    _payload.type = type;
    _payload.autopilot = autopilot;
    _payload.heading = heading;
    _payload.target_heading = target_heading;
    _payload.throttle = throttle;
    _payload.airspeed = airspeed;
    _payload.airspeed_sp = airspeed_sp;
    _payload.groundspeed = groundspeed;
    _payload.windspeed = windspeed;
    _payload.wind_heading = wind_heading;
    _payload.eph = eph;
    _payload.epv = epv;
    _payload.temperature_air = temperature_air;
    _payload.climb_rate = climb_rate;
    _payload.battery = battery;
    _payload.custom0 = custom0;
    _payload.custom1 = custom1;
    _payload.custom2 = custom2;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIGH_LATENCY2,
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_high_latency2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIGH_LATENCY2,
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIGH_LATENCY2 unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_high_latency2_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_high_latency2_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_high_latency2_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_high_latency2_decode(fmav_high_latency2_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_high_latency2_get_field_timestamp(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_high_latency2_get_field_latitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_high_latency2_get_field_longitude(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_get_field_custom_mode(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency2_get_field_altitude(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_high_latency2_get_field_target_altitude(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_get_field_target_distance(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_get_field_wp_num(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_high_latency2_get_field_failure_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_autopilot(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_heading(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_target_heading(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_throttle(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_airspeed(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_airspeed_sp(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_groundspeed(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[31]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_windspeed(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_wind_heading(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[33]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_eph(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_high_latency2_get_field_epv(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[35]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_temperature_air(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_climb_rate(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_battery(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_custom0(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[39]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_custom1(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_high_latency2_get_field_custom2(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(int8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIGH_LATENCY2  235

#define mavlink_high_latency2_t  fmav_high_latency2_t

#define MAVLINK_MSG_ID_HIGH_LATENCY2_LEN  42
#define MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN  42
#define MAVLINK_MSG_ID_235_LEN  42
#define MAVLINK_MSG_ID_235_MIN_LEN  42

#define MAVLINK_MSG_ID_HIGH_LATENCY2_CRC  179
#define MAVLINK_MSG_ID_235_CRC  179




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_high_latency2_pack(
        msg, sysid, compid,
        timestamp, type, autopilot, custom_mode, latitude, longitude, altitude, target_altitude, heading, target_heading, target_distance, throttle, airspeed, airspeed_sp, groundspeed, windspeed, wind_heading, eph, epv, temperature_air, climb_rate, battery, wp_num, failure_flags, custom0, custom1, custom2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_high_latency2_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2)
{
    return fmav_msg_high_latency2_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        timestamp, type, autopilot, custom_mode, latitude, longitude, altitude, target_altitude, heading, target_heading, target_distance, throttle, airspeed, airspeed_sp, groundspeed, windspeed, wind_heading, eph, epv, temperature_air, climb_rate, battery, wp_num, failure_flags, custom0, custom1, custom2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_high_latency2_decode(const mavlink_message_t* msg, mavlink_high_latency2_t* payload)
{
    fmav_msg_high_latency2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIGH_LATENCY2_H

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


#define FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA  179

#define FASTMAVLINK_MSG_ID_235_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_235_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_235_LEN  42
#define FASTMAVLINK_MSG_ID_235_CRCEXTRA  179



#define FASTMAVLINK_MSG_HIGH_LATENCY2_FLAGS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIGH_LATENCY2_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIGH_LATENCY2_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_235_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_235_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


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
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIGH_LATENCY2_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIGH_LATENCY2 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_high_latency2_decode(fmav_high_latency2_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIGH_LATENCY2_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

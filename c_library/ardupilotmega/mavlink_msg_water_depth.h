//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WATER_DEPTH_H
#define FASTMAVLINK_MSG_WATER_DEPTH_H


//----------------------------------------
//-- Message WATER_DEPTH
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_water_depth_t {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lng;
    float alt;
    float roll;
    float pitch;
    float yaw;
    float distance;
    float temperature;
    uint8_t id;
    uint8_t healthy;
}) fmav_water_depth_t;


#define FASTMAVLINK_MSG_ID_WATER_DEPTH  11038

#define FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA  47

#define FASTMAVLINK_MSG_WATER_DEPTH_FLAGS  0
#define FASTMAVLINK_MSG_WATER_DEPTH_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WATER_DEPTH_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WATER_DEPTH_FRAME_LEN_MAX  63



#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_LAT_OFS  4
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_LNG_OFS  8
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_ALT_OFS  12
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_ROLL_OFS  16
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_PITCH_OFS  20
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_YAW_OFS  24
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_DISTANCE_OFS  28
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_TEMPERATURE_OFS  32
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_ID_OFS  36
#define FASTMAVLINK_MSG_WATER_DEPTH_FIELD_HEALTHY_OFS  37


//----------------------------------------
//-- Message WATER_DEPTH pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature,
    fmav_status_t* _status)
{
    fmav_water_depth_t* _payload = (fmav_water_depth_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt = alt;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->distance = distance;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->healthy = healthy;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_WATER_DEPTH;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_water_depth_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_water_depth_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->id, _payload->healthy, _payload->lat, _payload->lng, _payload->alt, _payload->roll, _payload->pitch, _payload->yaw, _payload->distance, _payload->temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature,
    fmav_status_t* _status)
{
    fmav_water_depth_t* _payload = (fmav_water_depth_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->alt = alt;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->distance = distance;
    _payload->temperature = temperature;
    _payload->id = id;
    _payload->healthy = healthy;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WATER_DEPTH;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WATER_DEPTH >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WATER_DEPTH >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_water_depth_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_water_depth_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->id, _payload->healthy, _payload->lat, _payload->lng, _payload->alt, _payload->roll, _payload->pitch, _payload->yaw, _payload->distance, _payload->temperature,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature,
    fmav_status_t* _status)
{
    fmav_water_depth_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.lat = lat;
    _payload.lng = lng;
    _payload.alt = alt;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.distance = distance;
    _payload.temperature = temperature;
    _payload.id = id;
    _payload.healthy = healthy;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WATER_DEPTH,
        FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_water_depth_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_water_depth_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WATER_DEPTH,
        FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WATER_DEPTH_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WATER_DEPTH decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_water_depth_decode(fmav_water_depth_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_WATER_DEPTH_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_water_depth_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_water_depth_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_water_depth_get_field_lng(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_water_depth_get_field_temperature(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_water_depth_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_water_depth_get_field_healthy(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WATER_DEPTH  11038

#define mavlink_water_depth_t  fmav_water_depth_t

#define MAVLINK_MSG_ID_WATER_DEPTH_LEN  38
#define MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN  38
#define MAVLINK_MSG_ID_11038_LEN  38
#define MAVLINK_MSG_ID_11038_MIN_LEN  38

#define MAVLINK_MSG_ID_WATER_DEPTH_CRC  47
#define MAVLINK_MSG_ID_11038_CRC  47




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_water_depth_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_water_depth_pack(
        _msg, sysid, compid,
        time_boot_ms, id, healthy, lat, lng, alt, roll, pitch, yaw, distance, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_water_depth_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_water_depth_t* _payload)
{
    return mavlink_msg_water_depth_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->id, _payload->healthy, _payload->lat, _payload->lng, _payload->alt, _payload->roll, _payload->pitch, _payload->yaw, _payload->distance, _payload->temperature);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_water_depth_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t id, uint8_t healthy, int32_t lat, int32_t lng, float alt, float roll, float pitch, float yaw, float distance, float temperature)
{
    return fmav_msg_water_depth_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, id, healthy, lat, lng, alt, roll, pitch, yaw, distance, temperature,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_water_depth_decode(const mavlink_message_t* msg, mavlink_water_depth_t* payload)
{
    fmav_msg_water_depth_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WATER_DEPTH_H

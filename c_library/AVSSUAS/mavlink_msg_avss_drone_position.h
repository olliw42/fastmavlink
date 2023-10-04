//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AVSS_DRONE_POSITION_H
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_H


//----------------------------------------
//-- Message AVSS_DRONE_POSITION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_avss_drone_position_t {
    uint32_t time_boot_ms;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    float ground_alt;
    float barometer_alt;
}) fmav_avss_drone_position_t;


#define FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION  60051

#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_CRCEXTRA  245

#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FLAGS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FRAME_LEN_MAX  49



#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_LAT_OFS  4
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_LON_OFS  8
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_ALT_OFS  12
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_GROUND_ALT_OFS  16
#define FASTMAVLINK_MSG_AVSS_DRONE_POSITION_FIELD_BAROMETER_ALT_OFS  20


//----------------------------------------
//-- Message AVSS_DRONE_POSITION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt,
    fmav_status_t* _status)
{
    fmav_avss_drone_position_t* _payload = (fmav_avss_drone_position_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->ground_alt = ground_alt;
    _payload->barometer_alt = barometer_alt;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AVSS_DRONE_POSITION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_drone_position_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->lat, _payload->lon, _payload->alt, _payload->ground_alt, _payload->barometer_alt,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt,
    fmav_status_t* _status)
{
    fmav_avss_drone_position_t* _payload = (fmav_avss_drone_position_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->ground_alt = ground_alt;
    _payload->barometer_alt = barometer_alt;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_drone_position_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->lat, _payload->lon, _payload->alt, _payload->ground_alt, _payload->barometer_alt,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt,
    fmav_status_t* _status)
{
    fmav_avss_drone_position_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.ground_alt = ground_alt;
    _payload.barometer_alt = barometer_alt;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_position_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AVSS_DRONE_POSITION,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_POSITION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AVSS_DRONE_POSITION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_avss_drone_position_decode(fmav_avss_drone_position_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_DRONE_POSITION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_avss_drone_position_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_avss_drone_position_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_avss_drone_position_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_avss_drone_position_get_field_alt(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_avss_drone_position_get_field_ground_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_avss_drone_position_get_field_barometer_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION  60051

#define mavlink_avss_drone_position_t  fmav_avss_drone_position_t

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_LEN  24
#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_MIN_LEN  24
#define MAVLINK_MSG_ID_60051_LEN  24
#define MAVLINK_MSG_ID_60051_MIN_LEN  24

#define MAVLINK_MSG_ID_AVSS_DRONE_POSITION_CRC  245
#define MAVLINK_MSG_ID_60051_CRC  245




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_avss_drone_position_pack(
        _msg, sysid, compid,
        time_boot_ms, lat, lon, alt, ground_alt, barometer_alt,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_position_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_avss_drone_position_t* _payload)
{
    return mavlink_msg_avss_drone_position_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->lat, _payload->lon, _payload->alt, _payload->ground_alt, _payload->barometer_alt);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_position_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, float ground_alt, float barometer_alt)
{
    return fmav_msg_avss_drone_position_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, lat, lon, alt, ground_alt, barometer_alt,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_avss_drone_position_decode(const mavlink_message_t* msg, mavlink_avss_drone_position_t* payload)
{
    fmav_msg_avss_drone_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AVSS_DRONE_POSITION_H

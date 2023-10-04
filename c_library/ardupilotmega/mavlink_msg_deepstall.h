//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEEPSTALL_H
#define FASTMAVLINK_MSG_DEEPSTALL_H


//----------------------------------------
//-- Message DEEPSTALL
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_deepstall_t {
    int32_t landing_lat;
    int32_t landing_lon;
    int32_t path_lat;
    int32_t path_lon;
    int32_t arc_entry_lat;
    int32_t arc_entry_lon;
    float altitude;
    float expected_travel_distance;
    float cross_track_error;
    uint8_t stage;
}) fmav_deepstall_t;


#define FASTMAVLINK_MSG_ID_DEEPSTALL  195

#define FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA  120

#define FASTMAVLINK_MSG_DEEPSTALL_FLAGS  0
#define FASTMAVLINK_MSG_DEEPSTALL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DEEPSTALL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DEEPSTALL_FRAME_LEN_MAX  62



#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_LANDING_LAT_OFS  0
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_LANDING_LON_OFS  4
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_PATH_LAT_OFS  8
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_PATH_LON_OFS  12
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_ARC_ENTRY_LAT_OFS  16
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_ARC_ENTRY_LON_OFS  20
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_ALTITUDE_OFS  24
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_EXPECTED_TRAVEL_DISTANCE_OFS  28
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_CROSS_TRACK_ERROR_OFS  32
#define FASTMAVLINK_MSG_DEEPSTALL_FIELD_STAGE_OFS  36


//----------------------------------------
//-- Message DEEPSTALL pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage,
    fmav_status_t* _status)
{
    fmav_deepstall_t* _payload = (fmav_deepstall_t*)_msg->payload;

    _payload->landing_lat = landing_lat;
    _payload->landing_lon = landing_lon;
    _payload->path_lat = path_lat;
    _payload->path_lon = path_lon;
    _payload->arc_entry_lat = arc_entry_lat;
    _payload->arc_entry_lon = arc_entry_lon;
    _payload->altitude = altitude;
    _payload->expected_travel_distance = expected_travel_distance;
    _payload->cross_track_error = cross_track_error;
    _payload->stage = stage;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DEEPSTALL;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_deepstall_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_deepstall_pack(
        _msg, sysid, compid,
        _payload->landing_lat, _payload->landing_lon, _payload->path_lat, _payload->path_lon, _payload->arc_entry_lat, _payload->arc_entry_lon, _payload->altitude, _payload->expected_travel_distance, _payload->cross_track_error, _payload->stage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage,
    fmav_status_t* _status)
{
    fmav_deepstall_t* _payload = (fmav_deepstall_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->landing_lat = landing_lat;
    _payload->landing_lon = landing_lon;
    _payload->path_lat = path_lat;
    _payload->path_lon = path_lon;
    _payload->arc_entry_lat = arc_entry_lat;
    _payload->arc_entry_lon = arc_entry_lon;
    _payload->altitude = altitude;
    _payload->expected_travel_distance = expected_travel_distance;
    _payload->cross_track_error = cross_track_error;
    _payload->stage = stage;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEEPSTALL;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEEPSTALL >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEEPSTALL >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_deepstall_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_deepstall_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->landing_lat, _payload->landing_lon, _payload->path_lat, _payload->path_lon, _payload->arc_entry_lat, _payload->arc_entry_lon, _payload->altitude, _payload->expected_travel_distance, _payload->cross_track_error, _payload->stage,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage,
    fmav_status_t* _status)
{
    fmav_deepstall_t _payload;

    _payload.landing_lat = landing_lat;
    _payload.landing_lon = landing_lon;
    _payload.path_lat = path_lat;
    _payload.path_lon = path_lon;
    _payload.arc_entry_lat = arc_entry_lat;
    _payload.arc_entry_lon = arc_entry_lon;
    _payload.altitude = altitude;
    _payload.expected_travel_distance = expected_travel_distance;
    _payload.cross_track_error = cross_track_error;
    _payload.stage = stage;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DEEPSTALL,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_deepstall_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DEEPSTALL,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEEPSTALL decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_deepstall_decode(fmav_deepstall_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_landing_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_landing_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_path_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_path_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_arc_entry_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_deepstall_get_field_arc_entry_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_deepstall_get_field_altitude(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_deepstall_get_field_expected_travel_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_deepstall_get_field_cross_track_error(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_deepstall_get_field_stage(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEEPSTALL  195

#define mavlink_deepstall_t  fmav_deepstall_t

#define MAVLINK_MSG_ID_DEEPSTALL_LEN  37
#define MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN  37
#define MAVLINK_MSG_ID_195_LEN  37
#define MAVLINK_MSG_ID_195_MIN_LEN  37

#define MAVLINK_MSG_ID_DEEPSTALL_CRC  120
#define MAVLINK_MSG_ID_195_CRC  120




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_deepstall_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_deepstall_pack(
        _msg, sysid, compid,
        landing_lat, landing_lon, path_lat, path_lon, arc_entry_lat, arc_entry_lon, altitude, expected_travel_distance, cross_track_error, stage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_deepstall_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_deepstall_t* _payload)
{
    return mavlink_msg_deepstall_pack(
        sysid,
        compid,
        _msg,
        _payload->landing_lat, _payload->landing_lon, _payload->path_lat, _payload->path_lon, _payload->arc_entry_lat, _payload->arc_entry_lon, _payload->altitude, _payload->expected_travel_distance, _payload->cross_track_error, _payload->stage);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_deepstall_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
    return fmav_msg_deepstall_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        landing_lat, landing_lon, path_lat, path_lon, arc_entry_lat, arc_entry_lon, altitude, expected_travel_distance, cross_track_error, stage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_deepstall_decode(const mavlink_message_t* msg, mavlink_deepstall_t* payload)
{
    fmav_msg_deepstall_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEEPSTALL_H

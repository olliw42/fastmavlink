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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MIN  37
#define FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX  37
#define FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN  37
#define FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA  120

#define FASTMAVLINK_MSG_ID_195_LEN_MIN  37
#define FASTMAVLINK_MSG_ID_195_LEN_MAX  37
#define FASTMAVLINK_MSG_ID_195_LEN  37
#define FASTMAVLINK_MSG_ID_195_CRCEXTRA  120



#define FASTMAVLINK_MSG_DEEPSTALL_FLAGS  0
#define FASTMAVLINK_MSG_DEEPSTALL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DEEPSTALL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DEEPSTALL_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_195_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_195_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message DEEPSTALL packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage,
    fmav_status_t* _status)
{
    fmav_deepstall_t* _payload = (fmav_deepstall_t*)msg->payload;

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


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DEEPSTALL;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_deepstall_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_deepstall_pack(
        msg, sysid, compid,
        _payload->landing_lat, _payload->landing_lon, _payload->path_lat, _payload->path_lon, _payload->arc_entry_lat, _payload->arc_entry_lon, _payload->altitude, _payload->expected_travel_distance, _payload->cross_track_error, _payload->stage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage,
    fmav_status_t* _status)
{
    fmav_deepstall_t* _payload = (fmav_deepstall_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEEPSTALL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEEPSTALL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEEPSTALL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_deepstall_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_deepstall_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_deepstall_pack_to_frame_buf(
        buf, sysid, compid,
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
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEEPSTALL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEEPSTALL unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_deepstall_decode(fmav_deepstall_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DEEPSTALL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_deepstall_pack(
        msg, sysid, compid,
        landing_lat, landing_lon, path_lat, path_lon, arc_entry_lat, arc_entry_lon, altitude, expected_travel_distance, cross_track_error, stage,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_deepstall_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t landing_lat, int32_t landing_lon, int32_t path_lat, int32_t path_lon, int32_t arc_entry_lat, int32_t arc_entry_lon, float altitude, float expected_travel_distance, float cross_track_error, uint8_t stage)
{
    return fmav_msg_deepstall_pack_to_frame_buf(
        (uint8_t*)buf,
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

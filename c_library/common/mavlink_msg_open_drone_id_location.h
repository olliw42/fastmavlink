//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_open_drone_id_location_t {
    int32_t latitude;
    int32_t longitude;
    float altitude_barometric;
    float altitude_geodetic;
    float height;
    float timestamp;
    uint16_t direction;
    uint16_t speed_horizontal;
    int16_t speed_vertical;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t id_or_mac[20];
    uint8_t status;
    uint8_t height_reference;
    uint8_t horizontal_accuracy;
    uint8_t vertical_accuracy;
    uint8_t barometer_accuracy;
    uint8_t speed_accuracy;
    uint8_t timestamp_accuracy;
}) fmav_open_drone_id_location_t;


#define FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION  12901


#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MIN  59
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX  59
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN  59
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA  254

#define FASTMAVLINK_MSG_ID_12901_LEN_MIN  59
#define FASTMAVLINK_MSG_ID_12901_LEN_MAX  59
#define FASTMAVLINK_MSG_ID_12901_LEN  59
#define FASTMAVLINK_MSG_ID_12901_CRCEXTRA  254

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_LEN  20

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FLAGS  3
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_TARGET_SYSTEM_OFS  30
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_TARGET_COMPONENT_OFS  31

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_12901_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_12901_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t* _payload = (fmav_open_drone_id_location_t*)msg->payload;

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude_barometric = altitude_barometric;
    _payload->altitude_geodetic = altitude_geodetic;
    _payload->height = height;
    _payload->timestamp = timestamp;
    _payload->direction = direction;
    _payload->speed_horizontal = speed_horizontal;
    _payload->speed_vertical = speed_vertical;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;
    _payload->height_reference = height_reference;
    _payload->horizontal_accuracy = horizontal_accuracy;
    _payload->vertical_accuracy = vertical_accuracy;
    _payload->barometer_accuracy = barometer_accuracy;
    _payload->speed_accuracy = speed_accuracy;
    _payload->timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_location_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->status, _payload->direction, _payload->speed_horizontal, _payload->speed_vertical, _payload->latitude, _payload->longitude, _payload->altitude_barometric, _payload->altitude_geodetic, _payload->height_reference, _payload->height, _payload->horizontal_accuracy, _payload->vertical_accuracy, _payload->barometer_accuracy, _payload->speed_accuracy, _payload->timestamp, _payload->timestamp_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t* _payload = (fmav_open_drone_id_location_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude_barometric = altitude_barometric;
    _payload->altitude_geodetic = altitude_geodetic;
    _payload->height = height;
    _payload->timestamp = timestamp;
    _payload->direction = direction;
    _payload->speed_horizontal = speed_horizontal;
    _payload->speed_vertical = speed_vertical;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->status = status;
    _payload->height_reference = height_reference;
    _payload->horizontal_accuracy = horizontal_accuracy;
    _payload->vertical_accuracy = vertical_accuracy;
    _payload->barometer_accuracy = barometer_accuracy;
    _payload->speed_accuracy = speed_accuracy;
    _payload->timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_location_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->status, _payload->direction, _payload->speed_horizontal, _payload->speed_vertical, _payload->latitude, _payload->longitude, _payload->altitude_barometric, _payload->altitude_geodetic, _payload->height_reference, _payload->height, _payload->horizontal_accuracy, _payload->vertical_accuracy, _payload->barometer_accuracy, _payload->speed_accuracy, _payload->timestamp, _payload->timestamp_accuracy,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy,
    fmav_status_t* _status)
{
    fmav_open_drone_id_location_t _payload;

    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.altitude_barometric = altitude_barometric;
    _payload.altitude_geodetic = altitude_geodetic;
    _payload.height = height;
    _payload.timestamp = timestamp;
    _payload.direction = direction;
    _payload.speed_horizontal = speed_horizontal;
    _payload.speed_vertical = speed_vertical;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.status = status;
    _payload.height_reference = height_reference;
    _payload.horizontal_accuracy = horizontal_accuracy;
    _payload.vertical_accuracy = vertical_accuracy;
    _payload.barometer_accuracy = barometer_accuracy;
    _payload.speed_accuracy = speed_accuracy;
    _payload.timestamp_accuracy = timestamp_accuracy;
    memcpy(&(_payload.id_or_mac), id_or_mac, sizeof(uint8_t)*20);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_location_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_location_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPEN_DRONE_ID_LOCATION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_open_drone_id_location_decode(fmav_open_drone_id_location_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION  12901

#define mavlink_open_drone_id_location_t  fmav_open_drone_id_location_t

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_LEN  59
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_MIN_LEN  59
#define MAVLINK_MSG_ID_12901_LEN  59
#define MAVLINK_MSG_ID_12901_MIN_LEN  59

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_CRC  254
#define MAVLINK_MSG_ID_12901_CRC  254

#define MAVLINK_MSG_OPEN_DRONE_ID_LOCATION_FIELD_ID_OR_MAC_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_location_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_open_drone_id_location_pack(
        msg, sysid, compid,
        target_system, target_component, id_or_mac, status, direction, speed_horizontal, speed_vertical, latitude, longitude, altitude_barometric, altitude_geodetic, height_reference, height, horizontal_accuracy, vertical_accuracy, barometer_accuracy, speed_accuracy, timestamp, timestamp_accuracy,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_location_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t status, uint16_t direction, uint16_t speed_horizontal, int16_t speed_vertical, int32_t latitude, int32_t longitude, float altitude_barometric, float altitude_geodetic, uint8_t height_reference, float height, uint8_t horizontal_accuracy, uint8_t vertical_accuracy, uint8_t barometer_accuracy, uint8_t speed_accuracy, float timestamp, uint8_t timestamp_accuracy)
{
    return fmav_msg_open_drone_id_location_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, id_or_mac, status, direction, speed_horizontal, speed_vertical, latitude, longitude, altitude_barometric, altitude_geodetic, height_reference, height, horizontal_accuracy, vertical_accuracy, barometer_accuracy, speed_accuracy, timestamp, timestamp_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_open_drone_id_location_decode(const mavlink_message_t* msg, mavlink_open_drone_id_location_t* payload)
{
    fmav_msg_open_drone_id_location_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPEN_DRONE_ID_LOCATION_H

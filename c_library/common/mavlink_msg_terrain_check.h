//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TERRAIN_CHECK_H
#define FASTMAVLINK_MSG_TERRAIN_CHECK_H


//----------------------------------------
//-- Message TERRAIN_CHECK
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_terrain_check_t {
    int32_t lat;
    int32_t lon;
}) fmav_terrain_check_t;


#define FASTMAVLINK_MSG_ID_TERRAIN_CHECK  135

#define FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA  203

#define FASTMAVLINK_MSG_TERRAIN_CHECK_FLAGS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TERRAIN_CHECK_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_TERRAIN_CHECK_FIELD_LAT_OFS  0
#define FASTMAVLINK_MSG_TERRAIN_CHECK_FIELD_LON_OFS  4


//----------------------------------------
//-- Message TERRAIN_CHECK packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t* _payload = (fmav_terrain_check_t*)msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TERRAIN_CHECK;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_check_pack(
        msg, sysid, compid,
        _payload->lat, _payload->lon,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t* _payload = (fmav_terrain_check_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TERRAIN_CHECK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_terrain_check_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->lat, _payload->lon,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon,
    fmav_status_t* _status)
{
    fmav_terrain_check_t _payload;

    _payload.lat = lat;
    _payload.lon = lon;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_CHECK,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_terrain_check_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_terrain_check_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TERRAIN_CHECK,
        FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TERRAIN_CHECK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TERRAIN_CHECK unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_terrain_check_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_terrain_check_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_check_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_terrain_check_decode(fmav_terrain_check_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TERRAIN_CHECK_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_check_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_terrain_check_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TERRAIN_CHECK  135

#define mavlink_terrain_check_t  fmav_terrain_check_t

#define MAVLINK_MSG_ID_TERRAIN_CHECK_LEN  8
#define MAVLINK_MSG_ID_TERRAIN_CHECK_MIN_LEN  8
#define MAVLINK_MSG_ID_135_LEN  8
#define MAVLINK_MSG_ID_135_MIN_LEN  8

#define MAVLINK_MSG_ID_TERRAIN_CHECK_CRC  203
#define MAVLINK_MSG_ID_135_CRC  203




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_check_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int32_t lat, int32_t lon)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_terrain_check_pack(
        msg, sysid, compid,
        lat, lon,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_terrain_check_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t lat, int32_t lon)
{
    return fmav_msg_terrain_check_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        lat, lon,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_terrain_check_decode(const mavlink_message_t* msg, mavlink_terrain_check_t* payload)
{
    fmav_msg_terrain_check_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TERRAIN_CHECK_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_HOME_POSITION_H
#define FASTMAVLINK_MSG_SET_HOME_POSITION_H


//----------------------------------------
//-- Message SET_HOME_POSITION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_home_position_t {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float x;
    float y;
    float z;
    float q[4];
    float approach_x;
    float approach_y;
    float approach_z;
    uint8_t target_system;
    uint64_t time_usec;
}) fmav_set_home_position_t;


#define FASTMAVLINK_MSG_ID_SET_HOME_POSITION  243


#define FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MIN  53
#define FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX  61
#define FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN  61
#define FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA  85

#define FASTMAVLINK_MSG_ID_243_LEN_MIN  53
#define FASTMAVLINK_MSG_ID_243_LEN_MAX  61
#define FASTMAVLINK_MSG_ID_243_LEN  61
#define FASTMAVLINK_MSG_ID_243_CRCEXTRA  85

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_LEN  4

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FLAGS  1
#define FASTMAVLINK_MSG_SET_HOME_POSITION_TARGET_SYSTEM_OFS  52
#define FASTMAVLINK_MSG_SET_HOME_POSITION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SET_HOME_POSITION_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_243_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_243_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message SET_HOME_POSITION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t* _payload = (fmav_set_home_position_t*)msg->payload;

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude = altitude;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->approach_x = approach_x;
    _payload->approach_y = approach_y;
    _payload->approach_z = approach_z;
    _payload->target_system = target_system;
    _payload->time_usec = time_usec;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SET_HOME_POSITION;

    msg->target_sysid = target_system;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_home_position_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->latitude, _payload->longitude, _payload->altitude, _payload->x, _payload->y, _payload->z, _payload->q, _payload->approach_x, _payload->approach_y, _payload->approach_z, _payload->time_usec,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t* _payload = (fmav_set_home_position_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->latitude = latitude;
    _payload->longitude = longitude;
    _payload->altitude = altitude;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->approach_x = approach_x;
    _payload->approach_y = approach_y;
    _payload->approach_z = approach_z;
    _payload->target_system = target_system;
    _payload->time_usec = time_usec;
    memcpy(&(_payload->q), q, sizeof(float)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_HOME_POSITION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_home_position_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->latitude, _payload->longitude, _payload->altitude, _payload->x, _payload->y, _payload->z, _payload->q, _payload->approach_x, _payload->approach_y, _payload->approach_z, _payload->time_usec,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec,
    fmav_status_t* _status)
{
    fmav_set_home_position_t _payload;

    _payload.latitude = latitude;
    _payload.longitude = longitude;
    _payload.altitude = altitude;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.approach_x = approach_x;
    _payload.approach_y = approach_y;
    _payload.approach_z = approach_z;
    _payload.target_system = target_system;
    _payload.time_usec = time_usec;
    memcpy(&(_payload.q), q, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_HOME_POSITION,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_home_position_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_home_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_HOME_POSITION,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_HOME_POSITION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_HOME_POSITION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_home_position_decode(fmav_set_home_position_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SET_HOME_POSITION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_HOME_POSITION  243

#define mavlink_set_home_position_t  fmav_set_home_position_t

#define MAVLINK_MSG_ID_SET_HOME_POSITION_LEN  61
#define MAVLINK_MSG_ID_SET_HOME_POSITION_MIN_LEN  53
#define MAVLINK_MSG_ID_243_LEN  61
#define MAVLINK_MSG_ID_243_MIN_LEN  53

#define MAVLINK_MSG_ID_SET_HOME_POSITION_CRC  85
#define MAVLINK_MSG_ID_243_CRC  85

#define MAVLINK_MSG_SET_HOME_POSITION_FIELD_Q_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_home_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_home_position_pack(
        msg, sysid, compid,
        target_system, latitude, longitude, altitude, x, y, z, q, approach_x, approach_y, approach_z, time_usec,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_home_position_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float* q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
{
    return fmav_msg_set_home_position_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, latitude, longitude, altitude, x, y, z, q, approach_x, approach_y, approach_z, time_usec,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_home_position_decode(const mavlink_message_t* msg, mavlink_set_home_position_t* payload)
{
    fmav_msg_set_home_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_HOME_POSITION_H

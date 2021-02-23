//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AHRS2_H
#define FASTMAVLINK_MSG_AHRS2_H


//----------------------------------------
//-- Message AHRS2
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ahrs2_t {
    float roll;
    float pitch;
    float yaw;
    float altitude;
    int32_t lat;
    int32_t lng;
}) fmav_ahrs2_t;


#define FASTMAVLINK_MSG_ID_AHRS2  178


#define FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MIN  24
#define FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN  24
#define FASTMAVLINK_MSG_AHRS2_CRCEXTRA  47

#define FASTMAVLINK_MSG_ID_178_LEN_MIN  24
#define FASTMAVLINK_MSG_ID_178_LEN_MAX  24
#define FASTMAVLINK_MSG_ID_178_LEN  24
#define FASTMAVLINK_MSG_ID_178_CRCEXTRA  47



#define FASTMAVLINK_MSG_AHRS2_FLAGS  0
#define FASTMAVLINK_MSG_AHRS2_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AHRS2_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message AHRS2 packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs2_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_ahrs2_t* _payload = (fmav_ahrs2_t*)msg->payload;

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->altitude = altitude;
    _payload->lat = lat;
    _payload->lng = lng;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AHRS2;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AHRS2_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs2_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs2_pack(
        msg, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->altitude, _payload->lat, _payload->lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs2_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng,
    fmav_status_t* _status)
{
    fmav_ahrs2_t* _payload = (fmav_ahrs2_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->altitude = altitude;
    _payload->lat = lat;
    _payload->lng = lng;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AHRS2;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS2 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS2 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS2_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs2_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs2_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs2_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->altitude, _payload->lat, _payload->lng,
        _status);
}


//----------------------------------------
//-- Message AHRS2 unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ahrs2_decode(fmav_ahrs2_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AHRS2_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AHRS2  178

#define mavlink_ahrs2_t  fmav_ahrs2_t

#define MAVLINK_MSG_ID_AHRS2_LEN  24
#define MAVLINK_MSG_ID_AHRS2_MIN_LEN  24
#define MAVLINK_MSG_ID_178_LEN  24
#define MAVLINK_MSG_ID_178_MIN_LEN  24

#define MAVLINK_MSG_ID_AHRS2_CRC  47
#define MAVLINK_MSG_ID_178_CRC  47




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs2_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ahrs2_pack(
        msg, sysid, compid,
        roll, pitch, yaw, altitude, lat, lng,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs2_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
    return fmav_msg_ahrs2_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        roll, pitch, yaw, altitude, lat, lng,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ahrs2_decode(const mavlink_message_t* msg, mavlink_ahrs2_t* payload)
{
    fmav_msg_ahrs2_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AHRS2_H

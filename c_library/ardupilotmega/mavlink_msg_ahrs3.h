//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AHRS3_H
#define FASTMAVLINK_MSG_AHRS3_H


//----------------------------------------
//-- Message AHRS3
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_ahrs3_t {
    float roll;
    float pitch;
    float yaw;
    float altitude;
    int32_t lat;
    int32_t lng;
    float v1;
    float v2;
    float v3;
    float v4;
}) fmav_ahrs3_t;


#define FASTMAVLINK_MSG_ID_AHRS3  182

#define FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_AHRS3_CRCEXTRA  229

#define FASTMAVLINK_MSG_AHRS3_FLAGS  0
#define FASTMAVLINK_MSG_AHRS3_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AHRS3_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AHRS3_FRAME_LEN_MAX  65



#define FASTMAVLINK_MSG_AHRS3_FIELD_ROLL_OFS  0
#define FASTMAVLINK_MSG_AHRS3_FIELD_PITCH_OFS  4
#define FASTMAVLINK_MSG_AHRS3_FIELD_YAW_OFS  8
#define FASTMAVLINK_MSG_AHRS3_FIELD_ALTITUDE_OFS  12
#define FASTMAVLINK_MSG_AHRS3_FIELD_LAT_OFS  16
#define FASTMAVLINK_MSG_AHRS3_FIELD_LNG_OFS  20
#define FASTMAVLINK_MSG_AHRS3_FIELD_V1_OFS  24
#define FASTMAVLINK_MSG_AHRS3_FIELD_V2_OFS  28
#define FASTMAVLINK_MSG_AHRS3_FIELD_V3_OFS  32
#define FASTMAVLINK_MSG_AHRS3_FIELD_V4_OFS  36


//----------------------------------------
//-- Message AHRS3 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4,
    fmav_status_t* _status)
{
    fmav_ahrs3_t* _payload = (fmav_ahrs3_t*)msg->payload;

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->altitude = altitude;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->v1 = v1;
    _payload->v2 = v2;
    _payload->v3 = v3;
    _payload->v4 = v4;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AHRS3;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AHRS3_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs3_pack(
        msg, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->altitude, _payload->lat, _payload->lng, _payload->v1, _payload->v2, _payload->v3, _payload->v4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4,
    fmav_status_t* _status)
{
    fmav_ahrs3_t* _payload = (fmav_ahrs3_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->altitude = altitude;
    _payload->lat = lat;
    _payload->lng = lng;
    _payload->v1 = v1;
    _payload->v2 = v2;
    _payload->v3 = v3;
    _payload->v4 = v4;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AHRS3;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS3 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AHRS3 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_ahrs3_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->roll, _payload->pitch, _payload->yaw, _payload->altitude, _payload->lat, _payload->lng, _payload->v1, _payload->v2, _payload->v3, _payload->v4,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4,
    fmav_status_t* _status)
{
    fmav_ahrs3_t _payload;

    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.altitude = altitude;
    _payload.lat = lat;
    _payload.lng = lng;
    _payload.v1 = v1;
    _payload.v2 = v2;
    _payload.v3 = v3;
    _payload.v4 = v4;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AHRS3,
        FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_ahrs3_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_ahrs3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AHRS3,
        FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AHRS3_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AHRS3 unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_ahrs3_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_ahrs3_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ahrs3_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_ahrs3_decode(fmav_ahrs3_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AHRS3_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_altitude(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_ahrs3_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_ahrs3_get_field_lng(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_v1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_v2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_v3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_ahrs3_get_field_v4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AHRS3  182

#define mavlink_ahrs3_t  fmav_ahrs3_t

#define MAVLINK_MSG_ID_AHRS3_LEN  40
#define MAVLINK_MSG_ID_AHRS3_MIN_LEN  40
#define MAVLINK_MSG_ID_182_LEN  40
#define MAVLINK_MSG_ID_182_MIN_LEN  40

#define MAVLINK_MSG_ID_AHRS3_CRC  229
#define MAVLINK_MSG_ID_182_CRC  229




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs3_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_ahrs3_pack(
        msg, sysid, compid,
        roll, pitch, yaw, altitude, lat, lng, v1, v2, v3, v4,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_ahrs3_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng, float v1, float v2, float v3, float v4)
{
    return fmav_msg_ahrs3_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        roll, pitch, yaw, altitude, lat, lng, v1, v2, v3, v4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_ahrs3_decode(const mavlink_message_t* msg, mavlink_ahrs3_t* payload)
{
    fmav_msg_ahrs3_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AHRS3_H

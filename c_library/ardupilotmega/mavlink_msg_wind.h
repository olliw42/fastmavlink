//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIND_H
#define FASTMAVLINK_MSG_WIND_H


//----------------------------------------
//-- Message WIND
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wind_t {
    float direction;
    float speed;
    float speed_z;
}) fmav_wind_t;


#define FASTMAVLINK_MSG_ID_WIND  168

#define FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX  12
#define FASTMAVLINK_MSG_WIND_CRCEXTRA  1

#define FASTMAVLINK_MSG_WIND_FLAGS  0
#define FASTMAVLINK_MSG_WIND_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIND_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_WIND_FRAME_LEN_MAX  37



#define FASTMAVLINK_MSG_WIND_FIELD_DIRECTION_OFS  0
#define FASTMAVLINK_MSG_WIND_FIELD_SPEED_OFS  4
#define FASTMAVLINK_MSG_WIND_FIELD_SPEED_Z_OFS  8


//----------------------------------------
//-- Message WIND packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float direction, float speed, float speed_z,
    fmav_status_t* _status)
{
    fmav_wind_t* _payload = (fmav_wind_t*)msg->payload;

    _payload->direction = direction;
    _payload->speed = speed;
    _payload->speed_z = speed_z;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_WIND;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_WIND_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_pack(
        msg, sysid, compid,
        _payload->direction, _payload->speed, _payload->speed_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float direction, float speed, float speed_z,
    fmav_status_t* _status)
{
    fmav_wind_t* _payload = (fmav_wind_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->direction = direction;
    _payload->speed = speed;
    _payload->speed_z = speed_z;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIND;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->direction, _payload->speed, _payload->speed_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float direction, float speed, float speed_z,
    fmav_status_t* _status)
{
    fmav_wind_t _payload;

    _payload.direction = direction;
    _payload.speed = speed;
    _payload.speed_z = speed_z;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_WIND,
        FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_WIND,
        FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message WIND unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_wind_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_wind_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wind_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wind_decode(fmav_wind_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_WIND_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_get_field_direction(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_get_field_speed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_wind_get_field_speed_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIND  168

#define mavlink_wind_t  fmav_wind_t

#define MAVLINK_MSG_ID_WIND_LEN  12
#define MAVLINK_MSG_ID_WIND_MIN_LEN  12
#define MAVLINK_MSG_ID_168_LEN  12
#define MAVLINK_MSG_ID_168_MIN_LEN  12

#define MAVLINK_MSG_ID_WIND_CRC  1
#define MAVLINK_MSG_ID_168_CRC  1




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float direction, float speed, float speed_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wind_pack(
        msg, sysid, compid,
        direction, speed, speed_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float direction, float speed, float speed_z)
{
    return fmav_msg_wind_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        direction, speed, speed_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wind_decode(const mavlink_message_t* msg, mavlink_wind_t* payload)
{
    fmav_msg_wind_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIND_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_H
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_H


//----------------------------------------
//-- Message ORBIT_EXECUTION_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_orbit_execution_status_t {
    uint64_t time_usec;
    float radius;
    int32_t x;
    int32_t y;
    float z;
    uint8_t frame;
}) fmav_orbit_execution_status_t;


#define FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS  360

#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX  25
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA  11

#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FRAME_LEN_MAX  50



#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_RADIUS_OFS  8
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_X_OFS  12
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_Y_OFS  16
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_Z_OFS  20
#define FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_FIELD_FRAME_OFS  24


//----------------------------------------
//-- Message ORBIT_EXECUTION_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_orbit_execution_status_t* _payload = (fmav_orbit_execution_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->radius = radius;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->frame = frame;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_orbit_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_orbit_execution_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->radius, _payload->frame, _payload->x, _payload->y, _payload->z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_orbit_execution_status_t* _payload = (fmav_orbit_execution_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->radius = radius;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->frame = frame;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_orbit_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_orbit_execution_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->radius, _payload->frame, _payload->x, _payload->y, _payload->z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_orbit_execution_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.radius = radius;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.frame = frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_orbit_execution_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_orbit_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ORBIT_EXECUTION_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_orbit_execution_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_orbit_execution_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_orbit_execution_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_orbit_execution_status_decode(fmav_orbit_execution_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_orbit_execution_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_orbit_execution_status_get_field_radius(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_orbit_execution_status_get_field_x(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_orbit_execution_status_get_field_y(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_orbit_execution_status_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_orbit_execution_status_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS  360

#define mavlink_orbit_execution_status_t  fmav_orbit_execution_status_t

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN  25
#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_MIN_LEN  25
#define MAVLINK_MSG_ID_360_LEN  25
#define MAVLINK_MSG_ID_360_MIN_LEN  25

#define MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_CRC  11
#define MAVLINK_MSG_ID_360_CRC  11




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_orbit_execution_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_orbit_execution_status_pack(
        msg, sysid, compid,
        time_usec, radius, frame, x, y, z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_orbit_execution_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float radius, uint8_t frame, int32_t x, int32_t y, float z)
{
    return fmav_msg_orbit_execution_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, radius, frame, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_orbit_execution_status_decode(const mavlink_message_t* msg, mavlink_orbit_execution_status_t* payload)
{
    fmav_msg_orbit_execution_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ORBIT_EXECUTION_STATUS_H

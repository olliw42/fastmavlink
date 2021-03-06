//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COLLISION_H
#define FASTMAVLINK_MSG_COLLISION_H


//----------------------------------------
//-- Message COLLISION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_collision_t {
    uint32_t id;
    float time_to_minimum_delta;
    float altitude_minimum_delta;
    float horizontal_minimum_delta;
    uint8_t src;
    uint8_t action;
    uint8_t threat_level;
}) fmav_collision_t;


#define FASTMAVLINK_MSG_ID_COLLISION  247

#define FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_COLLISION_CRCEXTRA  81

#define FASTMAVLINK_MSG_COLLISION_FLAGS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COLLISION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COLLISION_FRAME_LEN_MAX  44



#define FASTMAVLINK_MSG_COLLISION_FIELD_ID_OFS  0
#define FASTMAVLINK_MSG_COLLISION_FIELD_TIME_TO_MINIMUM_DELTA_OFS  4
#define FASTMAVLINK_MSG_COLLISION_FIELD_ALTITUDE_MINIMUM_DELTA_OFS  8
#define FASTMAVLINK_MSG_COLLISION_FIELD_HORIZONTAL_MINIMUM_DELTA_OFS  12
#define FASTMAVLINK_MSG_COLLISION_FIELD_SRC_OFS  16
#define FASTMAVLINK_MSG_COLLISION_FIELD_ACTION_OFS  17
#define FASTMAVLINK_MSG_COLLISION_FIELD_THREAT_LEVEL_OFS  18


//----------------------------------------
//-- Message COLLISION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)msg->payload;

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COLLISION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_COLLISION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack(
        msg, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t* _payload = (fmav_collision_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->id = id;
    _payload->time_to_minimum_delta = time_to_minimum_delta;
    _payload->altitude_minimum_delta = altitude_minimum_delta;
    _payload->horizontal_minimum_delta = horizontal_minimum_delta;
    _payload->src = src;
    _payload->action = action;
    _payload->threat_level = threat_level;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COLLISION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COLLISION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_collision_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->src, _payload->id, _payload->action, _payload->threat_level, _payload->time_to_minimum_delta, _payload->altitude_minimum_delta, _payload->horizontal_minimum_delta,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta,
    fmav_status_t* _status)
{
    fmav_collision_t _payload;

    _payload.id = id;
    _payload.time_to_minimum_delta = time_to_minimum_delta;
    _payload.altitude_minimum_delta = altitude_minimum_delta;
    _payload.horizontal_minimum_delta = horizontal_minimum_delta;
    _payload.src = src;
    _payload.action = action;
    _payload.threat_level = threat_level;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COLLISION,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_collision_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_collision_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COLLISION,
        FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COLLISION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COLLISION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_collision_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_collision_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_collision_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_collision_decode(fmav_collision_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COLLISION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_collision_get_field_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_time_to_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_altitude_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_collision_get_field_horizontal_minimum_delta(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_src(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_action(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_collision_get_field_threat_level(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COLLISION  247

#define mavlink_collision_t  fmav_collision_t

#define MAVLINK_MSG_ID_COLLISION_LEN  19
#define MAVLINK_MSG_ID_COLLISION_MIN_LEN  19
#define MAVLINK_MSG_ID_247_LEN  19
#define MAVLINK_MSG_ID_247_MIN_LEN  19

#define MAVLINK_MSG_ID_COLLISION_CRC  81
#define MAVLINK_MSG_ID_247_CRC  81




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_collision_pack(
        msg, sysid, compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_collision_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t src, uint32_t id, uint8_t action, uint8_t threat_level, float time_to_minimum_delta, float altitude_minimum_delta, float horizontal_minimum_delta)
{
    return fmav_msg_collision_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        src, id, action, threat_level, time_to_minimum_delta, altitude_minimum_delta, horizontal_minimum_delta,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_collision_decode(const mavlink_message_t* msg, mavlink_collision_t* payload)
{
    fmav_msg_collision_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COLLISION_H

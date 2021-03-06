//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_COUNT_H
#define FASTMAVLINK_MSG_MISSION_COUNT_H


//----------------------------------------
//-- Message MISSION_COUNT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_count_t {
    uint16_t count;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t mission_type;
}) fmav_mission_count_t;


#define FASTMAVLINK_MSG_ID_MISSION_COUNT  44

#define FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA  221

#define FASTMAVLINK_MSG_MISSION_COUNT_FLAGS  3
#define FASTMAVLINK_MSG_MISSION_COUNT_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_MISSION_COUNT_TARGET_COMPONENT_OFS  3

#define FASTMAVLINK_MSG_MISSION_COUNT_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_MISSION_COUNT_FIELD_COUNT_OFS  0
#define FASTMAVLINK_MSG_MISSION_COUNT_FIELD_TARGET_SYSTEM_OFS  2
#define FASTMAVLINK_MSG_MISSION_COUNT_FIELD_TARGET_COMPONENT_OFS  3
#define FASTMAVLINK_MSG_MISSION_COUNT_FIELD_MISSION_TYPE_OFS  4


//----------------------------------------
//-- Message MISSION_COUNT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_count_t* _payload = (fmav_mission_count_t*)msg->payload;

    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_COUNT;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_count_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_count_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_count_t* _payload = (fmav_mission_count_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->count = count;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_COUNT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_COUNT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_COUNT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_count_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_count_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->count, _payload->mission_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_count_t _payload;

    _payload.count = count;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.mission_type = mission_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_COUNT,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_count_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_COUNT,
        FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_COUNT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_COUNT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_mission_count_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_mission_count_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_count_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_count_decode(fmav_mission_count_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_COUNT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_count_get_field_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_count_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_count_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_count_get_field_mission_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_COUNT  44

#define mavlink_mission_count_t  fmav_mission_count_t

#define MAVLINK_MSG_ID_MISSION_COUNT_LEN  5
#define MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN  4
#define MAVLINK_MSG_ID_44_LEN  5
#define MAVLINK_MSG_ID_44_MIN_LEN  4

#define MAVLINK_MSG_ID_MISSION_COUNT_CRC  221
#define MAVLINK_MSG_ID_44_CRC  221




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_count_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_count_pack(
        msg, sysid, compid,
        target_system, target_component, count, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_count_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type)
{
    return fmav_msg_mission_count_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, count, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_count_decode(const mavlink_message_t* msg, mavlink_mission_count_t* payload)
{
    fmav_msg_mission_count_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_COUNT_H

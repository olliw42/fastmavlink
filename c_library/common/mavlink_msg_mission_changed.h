//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_CHANGED_H
#define FASTMAVLINK_MSG_MISSION_CHANGED_H


//----------------------------------------
//-- Message MISSION_CHANGED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_changed_t {
    int16_t start_index;
    int16_t end_index;
    uint8_t origin_sysid;
    uint8_t origin_compid;
    uint8_t mission_type;
}) fmav_mission_changed_t;


#define FASTMAVLINK_MSG_ID_MISSION_CHANGED  52

#define FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA  132

#define FASTMAVLINK_MSG_MISSION_CHANGED_FLAGS  0
#define FASTMAVLINK_MSG_MISSION_CHANGED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MISSION_CHANGED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MISSION_CHANGED_FRAME_LEN_MAX  32



#define FASTMAVLINK_MSG_MISSION_CHANGED_FIELD_START_INDEX_OFS  0
#define FASTMAVLINK_MSG_MISSION_CHANGED_FIELD_END_INDEX_OFS  2
#define FASTMAVLINK_MSG_MISSION_CHANGED_FIELD_ORIGIN_SYSID_OFS  4
#define FASTMAVLINK_MSG_MISSION_CHANGED_FIELD_ORIGIN_COMPID_OFS  5
#define FASTMAVLINK_MSG_MISSION_CHANGED_FIELD_MISSION_TYPE_OFS  6


//----------------------------------------
//-- Message MISSION_CHANGED packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_changed_t* _payload = (fmav_mission_changed_t*)msg->payload;

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->origin_sysid = origin_sysid;
    _payload->origin_compid = origin_compid;
    _payload->mission_type = mission_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_CHANGED;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_changed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_changed_pack(
        msg, sysid, compid,
        _payload->start_index, _payload->end_index, _payload->origin_sysid, _payload->origin_compid, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_changed_t* _payload = (fmav_mission_changed_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->origin_sysid = origin_sysid;
    _payload->origin_compid = origin_compid;
    _payload->mission_type = mission_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_changed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_changed_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->start_index, _payload->end_index, _payload->origin_sysid, _payload->origin_compid, _payload->mission_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_changed_t _payload;

    _payload.start_index = start_index;
    _payload.end_index = end_index;
    _payload.origin_sysid = origin_sysid;
    _payload.origin_compid = origin_compid;
    _payload.mission_type = mission_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_CHANGED,
        FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_changed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_CHANGED,
        FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_CHANGED unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_mission_changed_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_mission_changed_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_changed_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_changed_decode(fmav_mission_changed_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_mission_changed_get_field_start_index(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_mission_changed_get_field_end_index(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_changed_get_field_origin_sysid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_changed_get_field_origin_compid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_changed_get_field_mission_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_CHANGED  52

#define mavlink_mission_changed_t  fmav_mission_changed_t

#define MAVLINK_MSG_ID_MISSION_CHANGED_LEN  7
#define MAVLINK_MSG_ID_MISSION_CHANGED_MIN_LEN  7
#define MAVLINK_MSG_ID_52_LEN  7
#define MAVLINK_MSG_ID_52_MIN_LEN  7

#define MAVLINK_MSG_ID_MISSION_CHANGED_CRC  132
#define MAVLINK_MSG_ID_52_CRC  132




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_changed_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_changed_pack(
        msg, sysid, compid,
        start_index, end_index, origin_sysid, origin_compid, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_changed_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type)
{
    return fmav_msg_mission_changed_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        start_index, end_index, origin_sysid, origin_compid, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_changed_decode(const mavlink_message_t* msg, mavlink_mission_changed_t* payload)
{
    fmav_msg_mission_changed_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_CHANGED_H

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

// fields are ordered, as they appear on the wire
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
//-- Message MISSION_CHANGED pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_changed_t* _payload = (fmav_mission_changed_t*)_msg->payload;

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->origin_sysid = origin_sysid;
    _payload->origin_compid = origin_compid;
    _payload->mission_type = mission_type;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MISSION_CHANGED;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_changed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_changed_pack(
        _msg, sysid, compid,
        _payload->start_index, _payload->end_index, _payload->origin_sysid, _payload->origin_compid, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_changed_t* _payload = (fmav_mission_changed_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->origin_sysid = origin_sysid;
    _payload->origin_compid = origin_compid;
    _payload->mission_type = mission_type;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHANGED >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHANGED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_changed_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_changed_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_changed_pack_to_frame_buf(
        _buf, sysid, compid,
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
//-- Message MISSION_CHANGED decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_changed_decode(fmav_mission_changed_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_CHANGED_PAYLOAD_LEN_MAX);
#endif
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
    mavlink_message_t* _msg,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_changed_pack(
        _msg, sysid, compid,
        start_index, end_index, origin_sysid, origin_compid, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_changed_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int16_t start_index, int16_t end_index, uint8_t origin_sysid, uint8_t origin_compid, uint8_t mission_type)
{
    return fmav_msg_mission_changed_pack_to_frame_buf(
        (uint8_t*)_buf,
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

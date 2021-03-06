//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_H
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_H


//----------------------------------------
//-- Message TIME_ESTIMATE_TO_TARGET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_time_estimate_to_target_t {
    int32_t safe_return;
    int32_t land;
    int32_t mission_next_item;
    int32_t mission_end;
    int32_t commanded_action;
}) fmav_time_estimate_to_target_t;


#define FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET  380

#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA  232

#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FIELD_SAFE_RETURN_OFS  0
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FIELD_LAND_OFS  4
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FIELD_MISSION_NEXT_ITEM_OFS  8
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FIELD_MISSION_END_OFS  12
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FIELD_COMMANDED_ACTION_OFS  16


//----------------------------------------
//-- Message TIME_ESTIMATE_TO_TARGET packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action,
    fmav_status_t* _status)
{
    fmav_time_estimate_to_target_t* _payload = (fmav_time_estimate_to_target_t*)msg->payload;

    _payload->safe_return = safe_return;
    _payload->land = land;
    _payload->mission_next_item = mission_next_item;
    _payload->mission_end = mission_end;
    _payload->commanded_action = commanded_action;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_time_estimate_to_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_time_estimate_to_target_pack(
        msg, sysid, compid,
        _payload->safe_return, _payload->land, _payload->mission_next_item, _payload->mission_end, _payload->commanded_action,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action,
    fmav_status_t* _status)
{
    fmav_time_estimate_to_target_t* _payload = (fmav_time_estimate_to_target_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->safe_return = safe_return;
    _payload->land = land;
    _payload->mission_next_item = mission_next_item;
    _payload->mission_end = mission_end;
    _payload->commanded_action = commanded_action;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_time_estimate_to_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_time_estimate_to_target_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->safe_return, _payload->land, _payload->mission_next_item, _payload->mission_end, _payload->commanded_action,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action,
    fmav_status_t* _status)
{
    fmav_time_estimate_to_target_t _payload;

    _payload.safe_return = safe_return;
    _payload.land = land;
    _payload.mission_next_item = mission_next_item;
    _payload.mission_end = mission_end;
    _payload.commanded_action = commanded_action;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_time_estimate_to_target_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_time_estimate_to_target_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TIME_ESTIMATE_TO_TARGET unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_time_estimate_to_target_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_time_estimate_to_target_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_time_estimate_to_target_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_time_estimate_to_target_decode(fmav_time_estimate_to_target_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_time_estimate_to_target_get_field_safe_return(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_time_estimate_to_target_get_field_land(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_time_estimate_to_target_get_field_mission_next_item(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_time_estimate_to_target_get_field_mission_end(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_time_estimate_to_target_get_field_commanded_action(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET  380

#define mavlink_time_estimate_to_target_t  fmav_time_estimate_to_target_t

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_LEN  20
#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_MIN_LEN  20
#define MAVLINK_MSG_ID_380_LEN  20
#define MAVLINK_MSG_ID_380_MIN_LEN  20

#define MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET_CRC  232
#define MAVLINK_MSG_ID_380_CRC  232




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_time_estimate_to_target_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_time_estimate_to_target_pack(
        msg, sysid, compid,
        safe_return, land, mission_next_item, mission_end, commanded_action,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_time_estimate_to_target_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int32_t safe_return, int32_t land, int32_t mission_next_item, int32_t mission_end, int32_t commanded_action)
{
    return fmav_msg_time_estimate_to_target_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        safe_return, land, mission_next_item, mission_end, commanded_action,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_time_estimate_to_target_decode(const mavlink_message_t* msg, mavlink_time_estimate_to_target_t* payload)
{
    fmav_msg_time_estimate_to_target_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_H

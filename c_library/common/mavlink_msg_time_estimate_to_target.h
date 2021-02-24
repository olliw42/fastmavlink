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


#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MIN  20
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN  20
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_CRCEXTRA  232

#define FASTMAVLINK_MSG_ID_380_LEN_MIN  20
#define FASTMAVLINK_MSG_ID_380_LEN_MAX  20
#define FASTMAVLINK_MSG_ID_380_LEN  20
#define FASTMAVLINK_MSG_ID_380_CRCEXTRA  232



#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_FLAGS  0
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_TARGET_COMPONENT_OFS  0


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
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message TIME_ESTIMATE_TO_TARGET unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_time_estimate_to_target_decode(fmav_time_estimate_to_target_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TIME_ESTIMATE_TO_TARGET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

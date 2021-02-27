//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_H
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_H


//----------------------------------------
//-- Message MISSION_REQUEST_PARTIAL_LIST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_request_partial_list_t {
    int16_t start_index;
    int16_t end_index;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t mission_type;
}) fmav_mission_request_partial_list_t;


#define FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST  37


#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MIN  6
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX  7
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN  7
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA  212

#define FASTMAVLINK_MSG_ID_37_LEN_MIN  6
#define FASTMAVLINK_MSG_ID_37_LEN_MAX  7
#define FASTMAVLINK_MSG_ID_37_LEN  7
#define FASTMAVLINK_MSG_ID_37_CRCEXTRA  212



#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_FLAGS  3
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_37_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_37_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message MISSION_REQUEST_PARTIAL_LIST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_request_partial_list_t* _payload = (fmav_mission_request_partial_list_t*)msg->payload;

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_request_partial_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_request_partial_list_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->start_index, _payload->end_index, _payload->mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_request_partial_list_t* _payload = (fmav_mission_request_partial_list_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->start_index = start_index;
    _payload->end_index = end_index;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mission_type = mission_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_request_partial_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_request_partial_list_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->start_index, _payload->end_index, _payload->mission_type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type,
    fmav_status_t* _status)
{
    fmav_mission_request_partial_list_t _payload;

    _payload.start_index = start_index;
    _payload.end_index = end_index;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.mission_type = mission_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_request_partial_list_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_request_partial_list_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_REQUEST_PARTIAL_LIST unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_request_partial_list_decode(fmav_mission_request_partial_list_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST  37

#define mavlink_mission_request_partial_list_t  fmav_mission_request_partial_list_t

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN  7
#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_MIN_LEN  6
#define MAVLINK_MSG_ID_37_LEN  7
#define MAVLINK_MSG_ID_37_MIN_LEN  6

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC  212
#define MAVLINK_MSG_ID_37_CRC  212




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_request_partial_list_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_request_partial_list_pack(
        msg, sysid, compid,
        target_system, target_component, start_index, end_index, mission_type,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_request_partial_list_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index, uint8_t mission_type)
{
    return fmav_msg_mission_request_partial_list_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, start_index, end_index, mission_type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_request_partial_list_decode(const mavlink_message_t* msg, mavlink_mission_request_partial_list_t* payload)
{
    fmav_msg_mission_request_partial_list_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_REQUEST_PARTIAL_LIST_H

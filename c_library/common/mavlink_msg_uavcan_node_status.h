//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavcan_node_status_t {
    uint64_t time_usec;
    uint32_t uptime_sec;
    uint16_t vendor_specific_status_code;
    uint8_t health;
    uint8_t mode;
    uint8_t sub_mode;
}) fmav_uavcan_node_status_t;


#define FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS  310


#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MIN  17
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN  17
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA  28

#define FASTMAVLINK_MSG_ID_310_LEN_MIN  17
#define FASTMAVLINK_MSG_ID_310_LEN_MAX  17
#define FASTMAVLINK_MSG_ID_310_LEN  17
#define FASTMAVLINK_MSG_ID_310_CRCEXTRA  28



#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code,
    fmav_status_t* _status)
{
    fmav_uavcan_node_status_t* _payload = (fmav_uavcan_node_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->vendor_specific_status_code = vendor_specific_status_code;
    _payload->health = health;
    _payload->mode = mode;
    _payload->sub_mode = sub_mode;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->health, _payload->mode, _payload->sub_mode, _payload->vendor_specific_status_code,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code,
    fmav_status_t* _status)
{
    fmav_uavcan_node_status_t* _payload = (fmav_uavcan_node_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->vendor_specific_status_code = vendor_specific_status_code;
    _payload->health = health;
    _payload->mode = mode;
    _payload->sub_mode = sub_mode;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->health, _payload->mode, _payload->sub_mode, _payload->vendor_specific_status_code,
        _status);
}


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavcan_node_status_decode(fmav_uavcan_node_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS  310

#define mavlink_uavcan_node_status_t  fmav_uavcan_node_status_t

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN  17
#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN  17
#define MAVLINK_MSG_ID_310_LEN  17
#define MAVLINK_MSG_ID_310_MIN_LEN  17

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC  28
#define MAVLINK_MSG_ID_310_CRC  28




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavcan_node_status_pack(
        msg, sysid, compid,
        time_usec, uptime_sec, health, mode, sub_mode, vendor_specific_status_code,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
    return fmav_msg_uavcan_node_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, uptime_sec, health, mode, sub_mode, vendor_specific_status_code,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavcan_node_status_decode(const mavlink_message_t* msg, mavlink_uavcan_node_status_t* payload)
{
    fmav_msg_uavcan_node_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H


//----------------------------------------
//-- Message UAVCAN_NODE_INFO
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavcan_node_info_t {
    uint64_t time_usec;
    uint32_t uptime_sec;
    uint32_t sw_vcs_commit;
    char name[80];
    uint8_t hw_version_major;
    uint8_t hw_version_minor;
    uint8_t hw_unique_id[16];
    uint8_t sw_version_major;
    uint8_t sw_version_minor;
}) fmav_uavcan_node_info_t;


#define FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO  311


#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MIN  116
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX  116
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN  116
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA  95

#define FASTMAVLINK_MSG_ID_311_LEN_MIN  116
#define FASTMAVLINK_MSG_ID_311_LEN_MAX  116
#define FASTMAVLINK_MSG_ID_311_LEN  116
#define FASTMAVLINK_MSG_ID_311_CRCEXTRA  95

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_LEN  80
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_LEN  16

#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_FLAGS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_INFO_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UAVCAN_NODE_INFO packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit,
    fmav_status_t* _status)
{
    fmav_uavcan_node_info_t* _payload = (fmav_uavcan_node_info_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->sw_vcs_commit = sw_vcs_commit;
    _payload->hw_version_major = hw_version_major;
    _payload->hw_version_minor = hw_version_minor;
    _payload->sw_version_major = sw_version_major;
    _payload->sw_version_minor = sw_version_minor;
    memcpy(&(_payload->name), name, sizeof(char)*80);
    memcpy(&(_payload->hw_unique_id), hw_unique_id, sizeof(uint8_t)*16);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_info_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->name, _payload->hw_version_major, _payload->hw_version_minor, _payload->hw_unique_id, _payload->sw_version_major, _payload->sw_version_minor, _payload->sw_vcs_commit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit,
    fmav_status_t* _status)
{
    fmav_uavcan_node_info_t* _payload = (fmav_uavcan_node_info_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->sw_vcs_commit = sw_vcs_commit;
    _payload->hw_version_major = hw_version_major;
    _payload->hw_version_minor = hw_version_minor;
    _payload->sw_version_major = sw_version_major;
    _payload->sw_version_minor = sw_version_minor;
    memcpy(&(_payload->name), name, sizeof(char)*80);
    memcpy(&(_payload->hw_unique_id), hw_unique_id, sizeof(uint8_t)*16);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_INFO >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_INFO_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_info_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_info_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->name, _payload->hw_version_major, _payload->hw_version_minor, _payload->hw_unique_id, _payload->sw_version_major, _payload->sw_version_minor, _payload->sw_vcs_commit,
        _status);
}


//----------------------------------------
//-- Message UAVCAN_NODE_INFO unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavcan_node_info_decode(fmav_uavcan_node_info_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UAVCAN_NODE_INFO_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO  311

#define mavlink_uavcan_node_info_t  fmav_uavcan_node_info_t

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_LEN  116
#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_MIN_LEN  116
#define MAVLINK_MSG_ID_311_LEN  116
#define MAVLINK_MSG_ID_311_MIN_LEN  116

#define MAVLINK_MSG_ID_UAVCAN_NODE_INFO_CRC  95
#define MAVLINK_MSG_ID_311_CRC  95

#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_NAME_LEN 80
#define MAVLINK_MSG_UAVCAN_NODE_INFO_FIELD_HW_UNIQUE_ID_LEN 16


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavcan_node_info_pack(
        msg, sysid, compid,
        time_usec, uptime_sec, name, hw_version_major, hw_version_minor, hw_unique_id, sw_version_major, sw_version_minor, sw_vcs_commit,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_info_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, const char* name, uint8_t hw_version_major, uint8_t hw_version_minor, const uint8_t* hw_unique_id, uint8_t sw_version_major, uint8_t sw_version_minor, uint32_t sw_vcs_commit)
{
    return fmav_msg_uavcan_node_info_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, uptime_sec, name, hw_version_major, hw_version_minor, hw_unique_id, sw_version_major, sw_version_minor, sw_vcs_commit,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavcan_node_info_decode(const mavlink_message_t* msg, mavlink_uavcan_node_info_t* payload)
{
    fmav_msg_uavcan_node_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVCAN_NODE_INFO_H

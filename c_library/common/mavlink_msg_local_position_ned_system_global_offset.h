//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_H
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_H


//----------------------------------------
//-- Message LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_local_position_ned_system_global_offset_t {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
}) fmav_local_position_ned_system_global_offset_t;


#define FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET  89


#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MIN  28
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN  28
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRCEXTRA  231

#define FASTMAVLINK_MSG_ID_89_LEN_MIN  28
#define FASTMAVLINK_MSG_ID_89_LEN_MAX  28
#define FASTMAVLINK_MSG_ID_89_LEN  28
#define FASTMAVLINK_MSG_ID_89_CRCEXTRA  231



#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_FLAGS  0
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_system_global_offset_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw,
    fmav_status_t* _status)
{
    fmav_local_position_ned_system_global_offset_t* _payload = (fmav_local_position_ned_system_global_offset_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_system_global_offset_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_local_position_ned_system_global_offset_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_local_position_ned_system_global_offset_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_system_global_offset_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw,
    fmav_status_t* _status)
{
    fmav_local_position_ned_system_global_offset_t* _payload = (fmav_local_position_ned_system_global_offset_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_local_position_ned_system_global_offset_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_local_position_ned_system_global_offset_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_local_position_ned_system_global_offset_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->x, _payload->y, _payload->z, _payload->roll, _payload->pitch, _payload->yaw,
        _status);
}


//----------------------------------------
//-- Message LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_local_position_ned_system_global_offset_decode(fmav_local_position_ned_system_global_offset_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET  89

#define mavlink_local_position_ned_system_global_offset_t  fmav_local_position_ned_system_global_offset_t

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN  28
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_MIN_LEN  28
#define MAVLINK_MSG_ID_89_LEN  28
#define MAVLINK_MSG_ID_89_MIN_LEN  28

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC  231
#define MAVLINK_MSG_ID_89_CRC  231




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_local_position_ned_system_global_offset_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_local_position_ned_system_global_offset_pack(
        msg, sysid, compid,
        time_boot_ms, x, y, z, roll, pitch, yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_local_position_ned_system_global_offset_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw)
{
    return fmav_msg_local_position_ned_system_global_offset_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, x, y, z, roll, pitch, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_local_position_ned_system_global_offset_decode(const mavlink_message_t* msg, mavlink_local_position_ned_system_global_offset_t* payload)
{
    fmav_msg_local_position_ned_system_global_offset_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_H

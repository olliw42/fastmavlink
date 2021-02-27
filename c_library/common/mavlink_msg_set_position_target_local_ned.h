//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_position_target_local_ned_t {
    uint32_t time_boot_ms;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t coordinate_frame;
}) fmav_set_position_target_local_ned_t;


#define FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED  84


#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MIN  53
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN  53
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA  143

#define FASTMAVLINK_MSG_ID_84_LEN_MIN  53
#define FASTMAVLINK_MSG_ID_84_LEN_MAX  53
#define FASTMAVLINK_MSG_ID_84_LEN  53
#define FASTMAVLINK_MSG_ID_84_CRCEXTRA  143



#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FLAGS  3
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TARGET_SYSTEM_OFS  50
#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TARGET_COMPONENT_OFS  51

#define FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_84_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_84_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t* _payload = (fmav_set_position_target_local_ned_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->coordinate_frame = coordinate_frame;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_position_target_local_ned_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t* _payload = (fmav_set_position_target_local_ned_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->coordinate_frame = coordinate_frame;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->target_system, _payload->target_component, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_set_position_target_local_ned_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.afx = afx;
    _payload.afy = afy;
    _payload.afz = afz;
    _payload.yaw = yaw;
    _payload.yaw_rate = yaw_rate;
    _payload.type_mask = type_mask;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.coordinate_frame = coordinate_frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_position_target_local_ned_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_POSITION_TARGET_LOCAL_NED unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_position_target_local_ned_decode(fmav_set_position_target_local_ned_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED  84

#define mavlink_set_position_target_local_ned_t  fmav_set_position_target_local_ned_t

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN  53
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_MIN_LEN  53
#define MAVLINK_MSG_ID_84_LEN  53
#define MAVLINK_MSG_ID_84_MIN_LEN  53

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC  143
#define MAVLINK_MSG_ID_84_CRC  143




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_position_target_local_ned_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_position_target_local_ned_pack(
        msg, sysid, compid,
        time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_position_target_local_ned_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    return fmav_msg_set_position_target_local_ned_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_position_target_local_ned_decode(const mavlink_message_t* msg, mavlink_set_position_target_local_ned_t* payload)
{
    fmav_msg_set_position_target_local_ned_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_H

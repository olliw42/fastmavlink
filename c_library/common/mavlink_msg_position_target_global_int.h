//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_H
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_H


//----------------------------------------
//-- Message POSITION_TARGET_GLOBAL_INT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_position_target_global_int_t {
    uint32_t time_boot_ms;
    int32_t lat_int;
    int32_t lon_int;
    float alt;
    float vx;
    float vy;
    float vz;
    float afx;
    float afy;
    float afz;
    float yaw;
    float yaw_rate;
    uint16_t type_mask;
    uint8_t coordinate_frame;
}) fmav_position_target_global_int_t;


#define FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT  87


#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MIN  51
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX  51
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN  51
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_CRCEXTRA  150

#define FASTMAVLINK_MSG_ID_87_LEN_MIN  51
#define FASTMAVLINK_MSG_ID_87_LEN_MAX  51
#define FASTMAVLINK_MSG_ID_87_LEN  51
#define FASTMAVLINK_MSG_ID_87_CRCEXTRA  150



#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_FLAGS  0
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message POSITION_TARGET_GLOBAL_INT packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_global_int_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_position_target_global_int_t* _payload = (fmav_position_target_global_int_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_int = lat_int;
    _payload->lon_int = lon_int;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->coordinate_frame = coordinate_frame;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_global_int_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_position_target_global_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_position_target_global_int_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->coordinate_frame, _payload->type_mask, _payload->lat_int, _payload->lon_int, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_global_int_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_position_target_global_int_t* _payload = (fmav_position_target_global_int_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->lat_int = lat_int;
    _payload->lon_int = lon_int;
    _payload->alt = alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->afx = afx;
    _payload->afy = afy;
    _payload->afz = afz;
    _payload->yaw = yaw;
    _payload->yaw_rate = yaw_rate;
    _payload->type_mask = type_mask;
    _payload->coordinate_frame = coordinate_frame;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_global_int_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_position_target_global_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_position_target_global_int_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->coordinate_frame, _payload->type_mask, _payload->lat_int, _payload->lon_int, _payload->alt, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


//----------------------------------------
//-- Message POSITION_TARGET_GLOBAL_INT unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_position_target_global_int_decode(fmav_position_target_global_int_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT  87

#define mavlink_position_target_global_int_t  fmav_position_target_global_int_t

#define MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN  51
#define MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_MIN_LEN  51
#define MAVLINK_MSG_ID_87_LEN  51
#define MAVLINK_MSG_ID_87_MIN_LEN  51

#define MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_CRC  150
#define MAVLINK_MSG_ID_87_CRC  150




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_position_target_global_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_position_target_global_int_pack(
        msg, sysid, compid,
        time_boot_ms, coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_position_target_global_int_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    return fmav_msg_position_target_global_int_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_position_target_global_int_decode(const mavlink_message_t* msg, mavlink_position_target_global_int_t* payload)
{
    fmav_msg_position_target_global_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_POSITION_TARGET_GLOBAL_INT_H

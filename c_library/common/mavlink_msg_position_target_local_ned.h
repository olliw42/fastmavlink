//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_H
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_H


//----------------------------------------
//-- Message POSITION_TARGET_LOCAL_NED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_position_target_local_ned_t {
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
    uint8_t coordinate_frame;
}) fmav_position_target_local_ned_t;


#define FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED  85

#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX  51
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA  140

#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FLAGS  0
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FRAME_LEN_MAX  76



#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_X_OFS  4
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_Y_OFS  8
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_Z_OFS  12
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_VX_OFS  16
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_VY_OFS  20
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_VZ_OFS  24
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_AFX_OFS  28
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_AFY_OFS  32
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_AFZ_OFS  36
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_YAW_OFS  40
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_YAW_RATE_OFS  44
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_TYPE_MASK_OFS  48
#define FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_FIELD_COORDINATE_FRAME_OFS  50


//----------------------------------------
//-- Message POSITION_TARGET_LOCAL_NED packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_position_target_local_ned_t* _payload = (fmav_position_target_local_ned_t*)msg->payload;

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
    _payload->coordinate_frame = coordinate_frame;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_position_target_local_ned_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_position_target_local_ned_t* _payload = (fmav_position_target_local_ned_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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
    _payload->coordinate_frame = coordinate_frame;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_position_target_local_ned_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->coordinate_frame, _payload->type_mask, _payload->x, _payload->y, _payload->z, _payload->vx, _payload->vy, _payload->vz, _payload->afx, _payload->afy, _payload->afz, _payload->yaw, _payload->yaw_rate,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate,
    fmav_status_t* _status)
{
    fmav_position_target_local_ned_t _payload;

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
    _payload.coordinate_frame = coordinate_frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_position_target_local_ned_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message POSITION_TARGET_LOCAL_NED unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_position_target_local_ned_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_position_target_local_ned_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_position_target_local_ned_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_position_target_local_ned_decode(fmav_position_target_local_ned_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_position_target_local_ned_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_afx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_afy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_afz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_position_target_local_ned_get_field_yaw_rate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_position_target_local_ned_get_field_type_mask(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[48]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_position_target_local_ned_get_field_coordinate_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED  85

#define mavlink_position_target_local_ned_t  fmav_position_target_local_ned_t

#define MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN  51
#define MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_MIN_LEN  51
#define MAVLINK_MSG_ID_85_LEN  51
#define MAVLINK_MSG_ID_85_MIN_LEN  51

#define MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_CRC  140
#define MAVLINK_MSG_ID_85_CRC  140




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_position_target_local_ned_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_position_target_local_ned_pack(
        msg, sysid, compid,
        time_boot_ms, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_position_target_local_ned_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)
{
    return fmav_msg_position_target_local_ned_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_position_target_local_ned_decode(const mavlink_message_t* msg, mavlink_position_target_local_ned_t* payload)
{
    fmav_msg_position_target_local_ned_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_POSITION_TARGET_LOCAL_NED_H

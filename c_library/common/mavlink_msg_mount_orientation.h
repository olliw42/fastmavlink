//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOUNT_ORIENTATION_H
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_H


//----------------------------------------
//-- Message MOUNT_ORIENTATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mount_orientation_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float yaw_absolute;
}) fmav_mount_orientation_t;


#define FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION  265

#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX  20
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA  26

#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FLAGS  0
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FRAME_LEN_MAX  45



#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FIELD_ROLL_OFS  4
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FIELD_PITCH_OFS  8
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FIELD_YAW_OFS  12
#define FASTMAVLINK_MSG_MOUNT_ORIENTATION_FIELD_YAW_ABSOLUTE_OFS  16


//----------------------------------------
//-- Message MOUNT_ORIENTATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute,
    fmav_status_t* _status)
{
    fmav_mount_orientation_t* _payload = (fmav_mount_orientation_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->yaw_absolute = yaw_absolute;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_orientation_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_orientation_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->yaw_absolute,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute,
    fmav_status_t* _status)
{
    fmav_mount_orientation_t* _payload = (fmav_mount_orientation_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->yaw_absolute = yaw_absolute;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_orientation_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_orientation_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->yaw_absolute,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute,
    fmav_status_t* _status)
{
    fmav_mount_orientation_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.roll = roll;
    _payload.pitch = pitch;
    _payload.yaw = yaw;
    _payload.yaw_absolute = yaw_absolute;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_orientation_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_orientation_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MOUNT_ORIENTATION,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_ORIENTATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MOUNT_ORIENTATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_mount_orientation_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_mount_orientation_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mount_orientation_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mount_orientation_decode(fmav_mount_orientation_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOUNT_ORIENTATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_mount_orientation_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mount_orientation_get_field_roll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mount_orientation_get_field_pitch(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mount_orientation_get_field_yaw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mount_orientation_get_field_yaw_absolute(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION  265

#define mavlink_mount_orientation_t  fmav_mount_orientation_t

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN  20
#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN  16
#define MAVLINK_MSG_ID_265_LEN  20
#define MAVLINK_MSG_ID_265_MIN_LEN  16

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC  26
#define MAVLINK_MSG_ID_265_CRC  26




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_orientation_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mount_orientation_pack(
        msg, sysid, compid,
        time_boot_ms, roll, pitch, yaw, yaw_absolute,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_orientation_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
    return fmav_msg_mount_orientation_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, roll, pitch, yaw, yaw_absolute,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mount_orientation_decode(const mavlink_message_t* msg, mavlink_mount_orientation_t* payload)
{
    fmav_msg_mount_orientation_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOUNT_ORIENTATION_H

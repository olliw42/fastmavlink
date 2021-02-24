//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ATTITUDE_H
#define FASTMAVLINK_MSG_ATTITUDE_H


//----------------------------------------
//-- Message ATTITUDE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_attitude_t {
    uint32_t time_boot_ms;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
}) fmav_attitude_t;


#define FASTMAVLINK_MSG_ID_ATTITUDE  30


#define FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MIN  28
#define FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN  28
#define FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA  39

#define FASTMAVLINK_MSG_ID_30_LEN_MIN  28
#define FASTMAVLINK_MSG_ID_30_LEN_MAX  28
#define FASTMAVLINK_MSG_ID_30_LEN  28
#define FASTMAVLINK_MSG_ID_30_CRCEXTRA  39



#define FASTMAVLINK_MSG_ATTITUDE_FLAGS  0
#define FASTMAVLINK_MSG_ATTITUDE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ATTITUDE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ATTITUDE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed,
    fmav_status_t* _status)
{
    fmav_attitude_t* _payload = (fmav_attitude_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ATTITUDE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed,
    fmav_status_t* _status)
{
    fmav_attitude_t* _payload = (fmav_attitude_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->roll = roll;
    _payload->pitch = pitch;
    _payload->yaw = yaw;
    _payload->rollspeed = rollspeed;
    _payload->pitchspeed = pitchspeed;
    _payload->yawspeed = yawspeed;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ATTITUDE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ATTITUDE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ATTITUDE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_attitude_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_attitude_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_attitude_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->roll, _payload->pitch, _payload->yaw, _payload->rollspeed, _payload->pitchspeed, _payload->yawspeed,
        _status);
}


//----------------------------------------
//-- Message ATTITUDE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_attitude_decode(fmav_attitude_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ATTITUDE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ATTITUDE  30

#define mavlink_attitude_t  fmav_attitude_t

#define MAVLINK_MSG_ID_ATTITUDE_LEN  28
#define MAVLINK_MSG_ID_ATTITUDE_MIN_LEN  28
#define MAVLINK_MSG_ID_30_LEN  28
#define MAVLINK_MSG_ID_30_MIN_LEN  28

#define MAVLINK_MSG_ID_ATTITUDE_CRC  39
#define MAVLINK_MSG_ID_30_CRC  39




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_attitude_pack(
        msg, sysid, compid,
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_attitude_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    return fmav_msg_attitude_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* payload)
{
    fmav_msg_attitude_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ATTITUDE_H

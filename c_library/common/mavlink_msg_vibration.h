//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VIBRATION_H
#define FASTMAVLINK_MSG_VIBRATION_H


//----------------------------------------
//-- Message VIBRATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_vibration_t {
    uint64_t time_usec;
    float vibration_x;
    float vibration_y;
    float vibration_z;
    uint32_t clipping_0;
    uint32_t clipping_1;
    uint32_t clipping_2;
}) fmav_vibration_t;


#define FASTMAVLINK_MSG_ID_VIBRATION  241


#define FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MIN  32
#define FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX  32
#define FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN  32
#define FASTMAVLINK_MSG_VIBRATION_CRCEXTRA  90

#define FASTMAVLINK_MSG_ID_241_LEN_MIN  32
#define FASTMAVLINK_MSG_ID_241_LEN_MAX  32
#define FASTMAVLINK_MSG_ID_241_LEN  32
#define FASTMAVLINK_MSG_ID_241_CRCEXTRA  90



#define FASTMAVLINK_MSG_VIBRATION_FLAGS  0
#define FASTMAVLINK_MSG_VIBRATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VIBRATION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message VIBRATION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2,
    fmav_status_t* _status)
{
    fmav_vibration_t* _payload = (fmav_vibration_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->vibration_x = vibration_x;
    _payload->vibration_y = vibration_y;
    _payload->vibration_z = vibration_z;
    _payload->clipping_0 = clipping_0;
    _payload->clipping_1 = clipping_1;
    _payload->clipping_2 = clipping_2;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VIBRATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VIBRATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vibration_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->vibration_x, _payload->vibration_y, _payload->vibration_z, _payload->clipping_0, _payload->clipping_1, _payload->clipping_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2,
    fmav_status_t* _status)
{
    fmav_vibration_t* _payload = (fmav_vibration_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->vibration_x = vibration_x;
    _payload->vibration_y = vibration_y;
    _payload->vibration_z = vibration_z;
    _payload->clipping_0 = clipping_0;
    _payload->clipping_1 = clipping_1;
    _payload->clipping_2 = clipping_2;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VIBRATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VIBRATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VIBRATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIBRATION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_vibration_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_vibration_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_vibration_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->vibration_x, _payload->vibration_y, _payload->vibration_z, _payload->clipping_0, _payload->clipping_1, _payload->clipping_2,
        _status);
}


//----------------------------------------
//-- Message VIBRATION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_vibration_decode(fmav_vibration_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_VIBRATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VIBRATION  241

#define mavlink_vibration_t  fmav_vibration_t

#define MAVLINK_MSG_ID_VIBRATION_LEN  32
#define MAVLINK_MSG_ID_VIBRATION_MIN_LEN  32
#define MAVLINK_MSG_ID_241_LEN  32
#define MAVLINK_MSG_ID_241_MIN_LEN  32

#define MAVLINK_MSG_ID_VIBRATION_CRC  90
#define MAVLINK_MSG_ID_241_CRC  90




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vibration_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_vibration_pack(
        msg, sysid, compid,
        time_usec, vibration_x, vibration_y, vibration_z, clipping_0, clipping_1, clipping_2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_vibration_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float vibration_x, float vibration_y, float vibration_z, uint32_t clipping_0, uint32_t clipping_1, uint32_t clipping_2)
{
    return fmav_msg_vibration_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, vibration_x, vibration_y, vibration_z, clipping_0, clipping_1, clipping_2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_vibration_decode(const mavlink_message_t* msg, mavlink_vibration_t* payload)
{
    fmav_msg_vibration_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VIBRATION_H

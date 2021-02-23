//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RPM_H
#define FASTMAVLINK_MSG_RPM_H


//----------------------------------------
//-- Message RPM
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rpm_t {
    float rpm1;
    float rpm2;
}) fmav_rpm_t;


#define FASTMAVLINK_MSG_ID_RPM  226


#define FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MIN  8
#define FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_RPM_PAYLOAD_LEN  8
#define FASTMAVLINK_MSG_RPM_CRCEXTRA  207

#define FASTMAVLINK_MSG_ID_226_LEN_MIN  8
#define FASTMAVLINK_MSG_ID_226_LEN_MAX  8
#define FASTMAVLINK_MSG_ID_226_LEN  8
#define FASTMAVLINK_MSG_ID_226_CRCEXTRA  207



#define FASTMAVLINK_MSG_RPM_FLAGS  0
#define FASTMAVLINK_MSG_RPM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RPM_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RPM packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2,
    fmav_status_t* _status)
{
    fmav_rpm_t* _payload = (fmav_rpm_t*)msg->payload;

    _payload->rpm1 = rpm1;
    _payload->rpm2 = rpm2;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RPM;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RPM_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rpm_pack(
        msg, sysid, compid,
        _payload->rpm1, _payload->rpm2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2,
    fmav_status_t* _status)
{
    fmav_rpm_t* _payload = (fmav_rpm_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rpm1 = rpm1;
    _payload->rpm2 = rpm2;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RPM;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RPM >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RPM >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RPM_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rpm_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->rpm1, _payload->rpm2,
        _status);
}


//----------------------------------------
//-- Message RPM unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rpm_decode(fmav_rpm_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RPM  226

#define mavlink_rpm_t  fmav_rpm_t

#define MAVLINK_MSG_ID_RPM_LEN  8
#define MAVLINK_MSG_ID_RPM_MIN_LEN  8
#define MAVLINK_MSG_ID_226_LEN  8
#define MAVLINK_MSG_ID_226_MIN_LEN  8

#define MAVLINK_MSG_ID_RPM_CRC  207
#define MAVLINK_MSG_ID_226_CRC  207




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rpm_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float rpm1, float rpm2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rpm_pack(
        msg, sysid, compid,
        rpm1, rpm2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rpm_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2)
{
    return fmav_msg_rpm_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        rpm1, rpm2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rpm_decode(const mavlink_message_t* msg, mavlink_rpm_t* payload)
{
    fmav_msg_rpm_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RPM_H

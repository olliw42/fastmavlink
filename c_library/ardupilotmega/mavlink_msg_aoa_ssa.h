//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AOA_SSA_H
#define FASTMAVLINK_MSG_AOA_SSA_H


//----------------------------------------
//-- Message AOA_SSA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_aoa_ssa_t {
    uint64_t time_usec;
    float AOA;
    float SSA;
}) fmav_aoa_ssa_t;


#define FASTMAVLINK_MSG_ID_AOA_SSA  11020


#define FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MIN  16
#define FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN  16
#define FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA  205

#define FASTMAVLINK_MSG_ID_11020_LEN_MIN  16
#define FASTMAVLINK_MSG_ID_11020_LEN_MAX  16
#define FASTMAVLINK_MSG_ID_11020_LEN  16
#define FASTMAVLINK_MSG_ID_11020_CRCEXTRA  205



#define FASTMAVLINK_MSG_AOA_SSA_FLAGS  0
#define FASTMAVLINK_MSG_AOA_SSA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AOA_SSA_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message AOA_SSA packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA,
    fmav_status_t* _status)
{
    fmav_aoa_ssa_t* _payload = (fmav_aoa_ssa_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->AOA = AOA;
    _payload->SSA = SSA;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AOA_SSA;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aoa_ssa_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aoa_ssa_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->AOA, _payload->SSA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA,
    fmav_status_t* _status)
{
    fmav_aoa_ssa_t* _payload = (fmav_aoa_ssa_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->AOA = AOA;
    _payload->SSA = SSA;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AOA_SSA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AOA_SSA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AOA_SSA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aoa_ssa_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aoa_ssa_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->AOA, _payload->SSA,
        _status);
}


//----------------------------------------
//-- Message AOA_SSA unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_aoa_ssa_decode(fmav_aoa_ssa_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AOA_SSA  11020

#define mavlink_aoa_ssa_t  fmav_aoa_ssa_t

#define MAVLINK_MSG_ID_AOA_SSA_LEN  16
#define MAVLINK_MSG_ID_AOA_SSA_MIN_LEN  16
#define MAVLINK_MSG_ID_11020_LEN  16
#define MAVLINK_MSG_ID_11020_MIN_LEN  16

#define MAVLINK_MSG_ID_AOA_SSA_CRC  205
#define MAVLINK_MSG_ID_11020_CRC  205




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aoa_ssa_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float AOA, float SSA)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_aoa_ssa_pack(
        msg, sysid, compid,
        time_usec, AOA, SSA,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aoa_ssa_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA)
{
    return fmav_msg_aoa_ssa_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, AOA, SSA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_aoa_ssa_decode(const mavlink_message_t* msg, mavlink_aoa_ssa_t* payload)
{
    fmav_msg_aoa_ssa_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AOA_SSA_H

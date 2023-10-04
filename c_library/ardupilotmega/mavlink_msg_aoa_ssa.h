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

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_aoa_ssa_t {
    uint64_t time_usec;
    float AOA;
    float SSA;
}) fmav_aoa_ssa_t;


#define FASTMAVLINK_MSG_ID_AOA_SSA  11020

#define FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA  205

#define FASTMAVLINK_MSG_AOA_SSA_FLAGS  0
#define FASTMAVLINK_MSG_AOA_SSA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AOA_SSA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AOA_SSA_FRAME_LEN_MAX  41



#define FASTMAVLINK_MSG_AOA_SSA_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_AOA_SSA_FIELD_AOA_OFS  8
#define FASTMAVLINK_MSG_AOA_SSA_FIELD_SSA_OFS  12


//----------------------------------------
//-- Message AOA_SSA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA,
    fmav_status_t* _status)
{
    fmav_aoa_ssa_t* _payload = (fmav_aoa_ssa_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->AOA = AOA;
    _payload->SSA = SSA;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AOA_SSA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aoa_ssa_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aoa_ssa_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->AOA, _payload->SSA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA,
    fmav_status_t* _status)
{
    fmav_aoa_ssa_t* _payload = (fmav_aoa_ssa_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->AOA = AOA;
    _payload->SSA = SSA;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AOA_SSA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AOA_SSA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AOA_SSA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aoa_ssa_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aoa_ssa_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->AOA, _payload->SSA,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA,
    fmav_status_t* _status)
{
    fmav_aoa_ssa_t _payload;

    _payload.time_usec = time_usec;
    _payload.AOA = AOA;
    _payload.SSA = SSA;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AOA_SSA,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aoa_ssa_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_aoa_ssa_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AOA_SSA,
        FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AOA_SSA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AOA_SSA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_aoa_ssa_decode(fmav_aoa_ssa_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AOA_SSA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_aoa_ssa_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aoa_ssa_get_field_AOA(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aoa_ssa_get_field_SSA(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
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
    mavlink_message_t* _msg,
    uint64_t time_usec, float AOA, float SSA)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_aoa_ssa_pack(
        _msg, sysid, compid,
        time_usec, AOA, SSA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aoa_ssa_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_aoa_ssa_t* _payload)
{
    return mavlink_msg_aoa_ssa_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->AOA, _payload->SSA);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aoa_ssa_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float AOA, float SSA)
{
    return fmav_msg_aoa_ssa_pack_to_frame_buf(
        (uint8_t*)_buf,
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

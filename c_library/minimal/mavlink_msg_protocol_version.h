//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PROTOCOL_VERSION_H
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_H


//----------------------------------------
//-- Message PROTOCOL_VERSION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_protocol_version_t {
    uint16_t version;
    uint16_t min_version;
    uint16_t max_version;
    uint8_t spec_version_hash[8];
    uint8_t library_version_hash[8];
}) fmav_protocol_version_t;


#define FASTMAVLINK_MSG_ID_PROTOCOL_VERSION  300


#define FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MIN  22
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN  22
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_CRCEXTRA  217

#define FASTMAVLINK_MSG_ID_300_LEN_MIN  22
#define FASTMAVLINK_MSG_ID_300_LEN_MAX  22
#define FASTMAVLINK_MSG_ID_300_LEN  22
#define FASTMAVLINK_MSG_ID_300_CRCEXTRA  217

#define FASTMAVLINK_MSG_PROTOCOL_VERSION_FIELD_SPEC_VERSION_HASH_LEN  8
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_FIELD_LIBRARY_VERSION_HASH_LEN  8

#define FASTMAVLINK_MSG_PROTOCOL_VERSION_FLAGS  0
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PROTOCOL_VERSION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message PROTOCOL_VERSION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_protocol_version_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t* spec_version_hash, const uint8_t* library_version_hash,
    fmav_status_t* _status)
{
    fmav_protocol_version_t* _payload = (fmav_protocol_version_t*)msg->payload;

    _payload->version = version;
    _payload->min_version = min_version;
    _payload->max_version = max_version;
    memcpy(&(_payload->spec_version_hash), spec_version_hash, sizeof(uint8_t)*8);
    memcpy(&(_payload->library_version_hash), library_version_hash, sizeof(uint8_t)*8);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_PROTOCOL_VERSION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_PROTOCOL_VERSION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_protocol_version_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_protocol_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_protocol_version_pack(
        msg, sysid, compid,
        _payload->version, _payload->min_version, _payload->max_version, _payload->spec_version_hash, _payload->library_version_hash,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_protocol_version_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t* spec_version_hash, const uint8_t* library_version_hash,
    fmav_status_t* _status)
{
    fmav_protocol_version_t* _payload = (fmav_protocol_version_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->version = version;
    _payload->min_version = min_version;
    _payload->max_version = max_version;
    memcpy(&(_payload->spec_version_hash), spec_version_hash, sizeof(uint8_t)*8);
    memcpy(&(_payload->library_version_hash), library_version_hash, sizeof(uint8_t)*8);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PROTOCOL_VERSION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PROTOCOL_VERSION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PROTOCOL_VERSION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PROTOCOL_VERSION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_protocol_version_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_protocol_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_protocol_version_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->version, _payload->min_version, _payload->max_version, _payload->spec_version_hash, _payload->library_version_hash,
        _status);
}


//----------------------------------------
//-- Message PROTOCOL_VERSION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_protocol_version_decode(fmav_protocol_version_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_PROTOCOL_VERSION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PROTOCOL_VERSION  300

#define mavlink_protocol_version_t  fmav_protocol_version_t

#define MAVLINK_MSG_ID_PROTOCOL_VERSION_LEN  22
#define MAVLINK_MSG_ID_PROTOCOL_VERSION_MIN_LEN  22
#define MAVLINK_MSG_ID_300_LEN  22
#define MAVLINK_MSG_ID_300_MIN_LEN  22

#define MAVLINK_MSG_ID_PROTOCOL_VERSION_CRC  217
#define MAVLINK_MSG_ID_300_CRC  217

#define MAVLINK_MSG_PROTOCOL_VERSION_FIELD_SPEC_VERSION_HASH_LEN 8
#define MAVLINK_MSG_PROTOCOL_VERSION_FIELD_LIBRARY_VERSION_HASH_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_protocol_version_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t* spec_version_hash, const uint8_t* library_version_hash)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_protocol_version_pack(
        msg, sysid, compid,
        version, min_version, max_version, spec_version_hash, library_version_hash,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_protocol_version_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t version, uint16_t min_version, uint16_t max_version, const uint8_t* spec_version_hash, const uint8_t* library_version_hash)
{
    return fmav_msg_protocol_version_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        version, min_version, max_version, spec_version_hash, library_version_hash,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_protocol_version_decode(const mavlink_message_t* msg, mavlink_protocol_version_t* payload)
{
    fmav_msg_protocol_version_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PROTOCOL_VERSION_H

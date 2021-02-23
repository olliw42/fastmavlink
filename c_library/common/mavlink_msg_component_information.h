//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_COMPONENT_INFORMATION_H
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_H


//----------------------------------------
//-- Message COMPONENT_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_component_information_t {
    uint32_t time_boot_ms;
    uint32_t metadata_type;
    uint32_t metadata_uid;
    uint32_t translation_uid;
    char metadata_uri[70];
    char translation_uri[70];
}) fmav_component_information_t;


#define FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION  395


#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MIN  156
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX  156
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN  156
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA  163

#define FASTMAVLINK_MSG_ID_395_LEN_MIN  156
#define FASTMAVLINK_MSG_ID_395_LEN_MAX  156
#define FASTMAVLINK_MSG_ID_395_LEN  156
#define FASTMAVLINK_MSG_ID_395_CRCEXTRA  163

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN  70
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_LEN  70

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message COMPONENT_INFORMATION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t metadata_type, uint32_t metadata_uid, const char* metadata_uri, uint32_t translation_uid, const char* translation_uri,
    fmav_status_t* _status)
{
    fmav_component_information_t* _payload = (fmav_component_information_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->metadata_type = metadata_type;
    _payload->metadata_uid = metadata_uid;
    _payload->translation_uid = translation_uid;
    memcpy(&(_payload->metadata_uri), metadata_uri, sizeof(char)*70);
    memcpy(&(_payload->translation_uri), translation_uri, sizeof(char)*70);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->metadata_type, _payload->metadata_uid, _payload->metadata_uri, _payload->translation_uid, _payload->translation_uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t metadata_type, uint32_t metadata_uid, const char* metadata_uri, uint32_t translation_uid, const char* translation_uri,
    fmav_status_t* _status)
{
    fmav_component_information_t* _payload = (fmav_component_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->metadata_type = metadata_type;
    _payload->metadata_uid = metadata_uid;
    _payload->translation_uid = translation_uid;
    memcpy(&(_payload->metadata_uri), metadata_uri, sizeof(char)*70);
    memcpy(&(_payload->translation_uri), translation_uri, sizeof(char)*70);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_component_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->metadata_type, _payload->metadata_uid, _payload->metadata_uri, _payload->translation_uid, _payload->translation_uri,
        _status);
}


//----------------------------------------
//-- Message COMPONENT_INFORMATION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_information_decode(fmav_component_information_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION  395

#define mavlink_component_information_t  fmav_component_information_t

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_LEN  156
#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_MIN_LEN  156
#define MAVLINK_MSG_ID_395_LEN  156
#define MAVLINK_MSG_ID_395_MIN_LEN  156

#define MAVLINK_MSG_ID_COMPONENT_INFORMATION_CRC  163
#define MAVLINK_MSG_ID_395_CRC  163

#define MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN 70
#define MAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_LEN 70


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint32_t metadata_type, uint32_t metadata_uid, const char* metadata_uri, uint32_t translation_uid, const char* translation_uri)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_component_information_pack(
        msg, sysid, compid,
        time_boot_ms, metadata_type, metadata_uid, metadata_uri, translation_uid, translation_uri,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_component_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t metadata_type, uint32_t metadata_uid, const char* metadata_uri, uint32_t translation_uid, const char* translation_uri)
{
    return fmav_msg_component_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, metadata_type, metadata_uid, metadata_uri, translation_uid, translation_uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_component_information_decode(const mavlink_message_t* msg, mavlink_component_information_t* payload)
{
    fmav_msg_component_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_COMPONENT_INFORMATION_H

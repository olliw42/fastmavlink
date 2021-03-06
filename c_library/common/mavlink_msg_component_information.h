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

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX  156
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA  163

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FRAME_LEN_MAX  181

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_NUM  70 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_LEN  70 // length of array = number of bytes
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_NUM  70 // number of elements in array
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_LEN  70 // length of array = number of bytes

#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_TYPE_OFS  4
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_UID_OFS  8
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_UID_OFS  12
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_OFS  16
#define FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_OFS  86


//----------------------------------------
//-- Message COMPONENT_INFORMATION packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t metadata_type, uint32_t metadata_uid, const char* metadata_uri, uint32_t translation_uid, const char* translation_uri,
    fmav_status_t* _status)
{
    fmav_component_information_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.metadata_type = metadata_type;
    _payload.metadata_uid = metadata_uid;
    _payload.translation_uid = translation_uid;
    memcpy(&(_payload.metadata_uri), metadata_uri, sizeof(char)*70);
    memcpy(&(_payload.translation_uri), translation_uri, sizeof(char)*70);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_component_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_component_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_COMPONENT_INFORMATION,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_COMPONENT_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message COMPONENT_INFORMATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_component_information_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_component_information_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_information_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_component_information_decode(fmav_component_information_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_COMPONENT_INFORMATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_information_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_information_get_field_metadata_type(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_information_get_field_metadata_uid(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_component_information_get_field_translation_uid(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_component_information_get_field_metadata_uri_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_component_information_get_field_metadata_uri(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_METADATA_URI_NUM) return 0;
    return ((char*)&(msg->payload[16]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_component_information_get_field_translation_uri_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[86]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_component_information_get_field_translation_uri(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_COMPONENT_INFORMATION_FIELD_TRANSLATION_URI_NUM) return 0;
    return ((char*)&(msg->payload[86]))[index];
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

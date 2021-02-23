//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_STORAGE_INFORMATION_H
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_H


//----------------------------------------
//-- Message STORAGE_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_storage_information_t {
    uint32_t time_boot_ms;
    float total_capacity;
    float used_capacity;
    float available_capacity;
    float read_speed;
    float write_speed;
    uint8_t storage_id;
    uint8_t storage_count;
    uint8_t status;
    uint8_t type;
    char name[32];
}) fmav_storage_information_t;


#define FASTMAVLINK_MSG_ID_STORAGE_INFORMATION  261


#define FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MIN  27
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX  60
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN  60
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA  179

#define FASTMAVLINK_MSG_ID_261_LEN_MIN  27
#define FASTMAVLINK_MSG_ID_261_LEN_MAX  60
#define FASTMAVLINK_MSG_ID_261_LEN  60
#define FASTMAVLINK_MSG_ID_261_CRCEXTRA  179

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_LEN  32

#define FASTMAVLINK_MSG_STORAGE_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_STORAGE_INFORMATION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message STORAGE_INFORMATION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name,
    fmav_status_t* _status)
{
    fmav_storage_information_t* _payload = (fmav_storage_information_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->total_capacity = total_capacity;
    _payload->used_capacity = used_capacity;
    _payload->available_capacity = available_capacity;
    _payload->read_speed = read_speed;
    _payload->write_speed = write_speed;
    _payload->storage_id = storage_id;
    _payload->storage_count = storage_count;
    _payload->status = status;
    _payload->type = type;
    memcpy(&(_payload->name), name, sizeof(char)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_STORAGE_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storage_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storage_information_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->storage_id, _payload->storage_count, _payload->status, _payload->total_capacity, _payload->used_capacity, _payload->available_capacity, _payload->read_speed, _payload->write_speed, _payload->type, _payload->name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name,
    fmav_status_t* _status)
{
    fmav_storage_information_t* _payload = (fmav_storage_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->total_capacity = total_capacity;
    _payload->used_capacity = used_capacity;
    _payload->available_capacity = available_capacity;
    _payload->read_speed = read_speed;
    _payload->write_speed = write_speed;
    _payload->storage_id = storage_id;
    _payload->storage_count = storage_count;
    _payload->status = status;
    _payload->type = type;
    memcpy(&(_payload->name), name, sizeof(char)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_STORAGE_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_STORAGE_INFORMATION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_storage_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_storage_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_storage_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->storage_id, _payload->storage_count, _payload->status, _payload->total_capacity, _payload->used_capacity, _payload->available_capacity, _payload->read_speed, _payload->write_speed, _payload->type, _payload->name,
        _status);
}


//----------------------------------------
//-- Message STORAGE_INFORMATION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_storage_information_decode(fmav_storage_information_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_STORAGE_INFORMATION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_STORAGE_INFORMATION  261

#define mavlink_storage_information_t  fmav_storage_information_t

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN  60
#define MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN  27
#define MAVLINK_MSG_ID_261_LEN  60
#define MAVLINK_MSG_ID_261_MIN_LEN  27

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC  179
#define MAVLINK_MSG_ID_261_CRC  179

#define MAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storage_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_storage_information_pack(
        msg, sysid, compid,
        time_boot_ms, storage_id, storage_count, status, total_capacity, used_capacity, available_capacity, read_speed, write_speed, type, name,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_storage_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char* name)
{
    return fmav_msg_storage_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, storage_id, storage_count, status, total_capacity, used_capacity, available_capacity, read_speed, write_speed, type, name,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_storage_information_decode(const mavlink_message_t* msg, mavlink_storage_information_t* payload)
{
    fmav_msg_storage_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_STORAGE_INFORMATION_H

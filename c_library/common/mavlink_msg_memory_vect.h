//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MEMORY_VECT_H
#define FASTMAVLINK_MSG_MEMORY_VECT_H


//----------------------------------------
//-- Message MEMORY_VECT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_memory_vect_t {
    uint16_t address;
    uint8_t ver;
    uint8_t type;
    int8_t value[32];
}) fmav_memory_vect_t;


#define FASTMAVLINK_MSG_ID_MEMORY_VECT  249

#define FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX  36
#define FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA  204

#define FASTMAVLINK_MSG_MEMORY_VECT_FLAGS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MEMORY_VECT_FRAME_LEN_MAX  61

#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_ADDRESS_OFS  0
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VER_OFS  2
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_TYPE_OFS  3
#define FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_OFS  4


//----------------------------------------
//-- Message MEMORY_VECT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t* _payload = (fmav_memory_vect_t*)msg->payload;

    _payload->address = address;
    _payload->ver = ver;
    _payload->type = type;
    memcpy(&(_payload->value), value, sizeof(int8_t)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MEMORY_VECT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_memory_vect_pack(
        msg, sysid, compid,
        _payload->address, _payload->ver, _payload->type, _payload->value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t* _payload = (fmav_memory_vect_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->address = address;
    _payload->ver = ver;
    _payload->type = type;
    memcpy(&(_payload->value), value, sizeof(int8_t)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MEMORY_VECT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMORY_VECT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MEMORY_VECT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_memory_vect_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->address, _payload->ver, _payload->type, _payload->value,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value,
    fmav_status_t* _status)
{
    fmav_memory_vect_t _payload;

    _payload.address = address;
    _payload.ver = ver;
    _payload.type = type;
    memcpy(&(_payload.value), value, sizeof(int8_t)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MEMORY_VECT,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_memory_vect_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MEMORY_VECT,
        FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MEMORY_VECT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MEMORY_VECT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_memory_vect_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_memory_vect_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_memory_vect_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_memory_vect_decode(fmav_memory_vect_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MEMORY_VECT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_memory_vect_get_field_address(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_memory_vect_get_field_ver(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_memory_vect_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_memory_vect_get_field_value_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[4]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_memory_vect_get_field_value(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_NUM) return 0;
    return ((int8_t*)&(msg->payload[4]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MEMORY_VECT  249

#define mavlink_memory_vect_t  fmav_memory_vect_t

#define MAVLINK_MSG_ID_MEMORY_VECT_LEN  36
#define MAVLINK_MSG_ID_MEMORY_VECT_MIN_LEN  36
#define MAVLINK_MSG_ID_249_LEN  36
#define MAVLINK_MSG_ID_249_MIN_LEN  36

#define MAVLINK_MSG_ID_MEMORY_VECT_CRC  204
#define MAVLINK_MSG_ID_249_CRC  204

#define MAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_memory_vect_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_memory_vect_pack(
        msg, sysid, compid,
        address, ver, type, value,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_memory_vect_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t address, uint8_t ver, uint8_t type, const int8_t* value)
{
    return fmav_msg_memory_vect_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        address, ver, type, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_memory_vect_decode(const mavlink_message_t* msg, mavlink_memory_vect_t* payload)
{
    fmav_msg_memory_vect_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MEMORY_VECT_H

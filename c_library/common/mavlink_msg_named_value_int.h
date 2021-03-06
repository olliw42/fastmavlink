//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_NAMED_VALUE_INT_H
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_H


//----------------------------------------
//-- Message NAMED_VALUE_INT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_named_value_int_t {
    uint32_t time_boot_ms;
    int32_t value;
    char name[10];
}) fmav_named_value_int_t;


#define FASTMAVLINK_MSG_ID_NAMED_VALUE_INT  252

#define FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX  18
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA  44

#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FLAGS  0
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FRAME_LEN_MAX  43

#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN  10 // length of array = number of bytes

#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_VALUE_OFS  4
#define FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_OFS  8


//----------------------------------------
//-- Message NAMED_VALUE_INT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* name, int32_t value,
    fmav_status_t* _status)
{
    fmav_named_value_int_t* _payload = (fmav_named_value_int_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->value = value;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_NAMED_VALUE_INT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_named_value_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_named_value_int_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->name, _payload->value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* name, int32_t value,
    fmav_status_t* _status)
{
    fmav_named_value_int_t* _payload = (fmav_named_value_int_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->value = value;
    memcpy(&(_payload->name), name, sizeof(char)*10);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_NAMED_VALUE_INT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_NAMED_VALUE_INT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_NAMED_VALUE_INT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_named_value_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_named_value_int_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->name, _payload->value,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* name, int32_t value,
    fmav_status_t* _status)
{
    fmav_named_value_int_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.value = value;
    memcpy(&(_payload.name), name, sizeof(char)*10);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_NAMED_VALUE_INT,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_named_value_int_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_named_value_int_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_NAMED_VALUE_INT,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_NAMED_VALUE_INT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message NAMED_VALUE_INT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_named_value_int_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_named_value_int_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_named_value_int_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_named_value_int_decode(fmav_named_value_int_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_NAMED_VALUE_INT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_named_value_int_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_named_value_int_get_field_value(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_named_value_int_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[8]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_named_value_int_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[8]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_NAMED_VALUE_INT  252

#define mavlink_named_value_int_t  fmav_named_value_int_t

#define MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN  18
#define MAVLINK_MSG_ID_NAMED_VALUE_INT_MIN_LEN  18
#define MAVLINK_MSG_ID_252_LEN  18
#define MAVLINK_MSG_ID_252_MIN_LEN  18

#define MAVLINK_MSG_ID_NAMED_VALUE_INT_CRC  44
#define MAVLINK_MSG_ID_252_CRC  44

#define MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN 10


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_named_value_int_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, const char* name, int32_t value)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_named_value_int_pack(
        msg, sysid, compid,
        time_boot_ms, name, value,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_named_value_int_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, const char* name, int32_t value)
{
    return fmav_msg_named_value_int_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, name, value,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_named_value_int_decode(const mavlink_message_t* msg, mavlink_named_value_int_t* payload)
{
    fmav_msg_named_value_int_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_NAMED_VALUE_INT_H

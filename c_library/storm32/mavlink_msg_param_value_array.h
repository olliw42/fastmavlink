//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_H
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_H


//----------------------------------------
//-- Message PARAM_VALUE_ARRAY
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_param_value_array_t {
    uint16_t param_count;
    uint16_t param_index_first;
    uint16_t flags;
    uint8_t param_array_len;
    uint8_t packet_buf[248];
}) fmav_param_value_array_t;


#define FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY  60041

#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX  255
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA  191

#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FLAGS  0
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FRAME_LEN_MAX  280

#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PACKET_BUF_NUM  248 // number of elements in array
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PACKET_BUF_LEN  248 // length of array = number of bytes

#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PARAM_COUNT_OFS  0
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PARAM_INDEX_FIRST_OFS  2
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_FLAGS_OFS  4
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PARAM_ARRAY_LEN_OFS  6
#define FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PACKET_BUF_OFS  7


//----------------------------------------
//-- Message PARAM_VALUE_ARRAY pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t param_count, uint16_t param_index_first, uint8_t param_array_len, uint16_t flags, const uint8_t* packet_buf,
    fmav_status_t* _status)
{
    fmav_param_value_array_t* _payload = (fmav_param_value_array_t*)_msg->payload;

    _payload->param_count = param_count;
    _payload->param_index_first = param_index_first;
    _payload->flags = flags;
    _payload->param_array_len = param_array_len;
    memcpy(&(_payload->packet_buf), packet_buf, sizeof(uint8_t)*248);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_value_array_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_value_array_pack(
        _msg, sysid, compid,
        _payload->param_count, _payload->param_index_first, _payload->param_array_len, _payload->flags, _payload->packet_buf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t param_count, uint16_t param_index_first, uint8_t param_array_len, uint16_t flags, const uint8_t* packet_buf,
    fmav_status_t* _status)
{
    fmav_param_value_array_t* _payload = (fmav_param_value_array_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->param_count = param_count;
    _payload->param_index_first = param_index_first;
    _payload->flags = flags;
    _payload->param_array_len = param_array_len;
    memcpy(&(_payload->packet_buf), packet_buf, sizeof(uint8_t)*248);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_value_array_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_param_value_array_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->param_count, _payload->param_index_first, _payload->param_array_len, _payload->flags, _payload->packet_buf,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t param_count, uint16_t param_index_first, uint8_t param_array_len, uint16_t flags, const uint8_t* packet_buf,
    fmav_status_t* _status)
{
    fmav_param_value_array_t _payload;

    _payload.param_count = param_count;
    _payload.param_index_first = param_index_first;
    _payload.flags = flags;
    _payload.param_array_len = param_array_len;
    memcpy(&(_payload.packet_buf), packet_buf, sizeof(uint8_t)*248);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_param_value_array_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_PARAM_VALUE_ARRAY,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message PARAM_VALUE_ARRAY decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_param_value_array_decode(fmav_param_value_array_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_get_field_param_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_get_field_param_index_first(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_param_value_array_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_value_array_get_field_param_array_len(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_param_value_array_get_field_packet_buf_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[7]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_param_value_array_get_field_packet_buf(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PACKET_BUF_NUM) return 0;
    return ((uint8_t*)&(msg->payload[7]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_PARAM_VALUE_ARRAY  60041

#define mavlink_param_value_array_t  fmav_param_value_array_t

#define MAVLINK_MSG_ID_PARAM_VALUE_ARRAY_LEN  255
#define MAVLINK_MSG_ID_PARAM_VALUE_ARRAY_MIN_LEN  255
#define MAVLINK_MSG_ID_60041_LEN  255
#define MAVLINK_MSG_ID_60041_MIN_LEN  255

#define MAVLINK_MSG_ID_PARAM_VALUE_ARRAY_CRC  191
#define MAVLINK_MSG_ID_60041_CRC  191

#define MAVLINK_MSG_PARAM_VALUE_ARRAY_FIELD_PACKET_BUF_LEN 248


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_value_array_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint16_t param_count, uint16_t param_index_first, uint8_t param_array_len, uint16_t flags, const uint8_t* packet_buf)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_param_value_array_pack(
        _msg, sysid, compid,
        param_count, param_index_first, param_array_len, flags, packet_buf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_value_array_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_param_value_array_t* _payload)
{
    return mavlink_msg_param_value_array_pack(
        sysid,
        compid,
        _msg,
        _payload->param_count, _payload->param_index_first, _payload->param_array_len, _payload->flags, _payload->packet_buf);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_param_value_array_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t param_count, uint16_t param_index_first, uint8_t param_array_len, uint16_t flags, const uint8_t* packet_buf)
{
    return fmav_msg_param_value_array_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        param_count, param_index_first, param_array_len, flags, packet_buf,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_param_value_array_decode(const mavlink_message_t* msg, mavlink_param_value_array_t* payload)
{
    fmav_msg_param_value_array_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_PARAM_VALUE_ARRAY_H

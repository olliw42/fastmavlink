//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DATA_STREAM_H
#define FASTMAVLINK_MSG_DATA_STREAM_H


//----------------------------------------
//-- Message DATA_STREAM
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_data_stream_t {
    uint16_t message_rate;
    uint8_t stream_id;
    uint8_t on_off;
}) fmav_data_stream_t;


#define FASTMAVLINK_MSG_ID_DATA_STREAM  67

#define FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA  21

#define FASTMAVLINK_MSG_DATA_STREAM_FLAGS  0
#define FASTMAVLINK_MSG_DATA_STREAM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_DATA_STREAM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_DATA_STREAM_FRAME_LEN_MAX  29



#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_MESSAGE_RATE_OFS  0
#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_STREAM_ID_OFS  2
#define FASTMAVLINK_MSG_DATA_STREAM_FIELD_ON_OFF_OFS  3


//----------------------------------------
//-- Message DATA_STREAM packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t* _payload = (fmav_data_stream_t*)msg->payload;

    _payload->message_rate = message_rate;
    _payload->stream_id = stream_id;
    _payload->on_off = on_off;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DATA_STREAM;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_stream_pack(
        msg, sysid, compid,
        _payload->stream_id, _payload->message_rate, _payload->on_off,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t* _payload = (fmav_data_stream_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->message_rate = message_rate;
    _payload->stream_id = stream_id;
    _payload->on_off = on_off;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DATA_STREAM;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_STREAM >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DATA_STREAM >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_data_stream_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->stream_id, _payload->message_rate, _payload->on_off,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off,
    fmav_status_t* _status)
{
    fmav_data_stream_t _payload;

    _payload.message_rate = message_rate;
    _payload.stream_id = stream_id;
    _payload.on_off = on_off;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DATA_STREAM,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_data_stream_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DATA_STREAM,
        FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DATA_STREAM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DATA_STREAM unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_data_stream_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_data_stream_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_stream_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_data_stream_decode(fmav_data_stream_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DATA_STREAM_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_data_stream_get_field_message_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_stream_get_field_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_data_stream_get_field_on_off(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[3]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DATA_STREAM  67

#define mavlink_data_stream_t  fmav_data_stream_t

#define MAVLINK_MSG_ID_DATA_STREAM_LEN  4
#define MAVLINK_MSG_ID_DATA_STREAM_MIN_LEN  4
#define MAVLINK_MSG_ID_67_LEN  4
#define MAVLINK_MSG_ID_67_MIN_LEN  4

#define MAVLINK_MSG_ID_DATA_STREAM_CRC  21
#define MAVLINK_MSG_ID_67_CRC  21




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_stream_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_data_stream_pack(
        msg, sysid, compid,
        stream_id, message_rate, on_off,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_data_stream_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t message_rate, uint8_t on_off)
{
    return fmav_msg_data_stream_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        stream_id, message_rate, on_off,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_data_stream_decode(const mavlink_message_t* msg, mavlink_data_stream_t* payload)
{
    fmav_msg_data_stream_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DATA_STREAM_H

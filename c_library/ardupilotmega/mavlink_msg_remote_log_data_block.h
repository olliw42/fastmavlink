//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_H
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_H


//----------------------------------------
//-- Message REMOTE_LOG_DATA_BLOCK
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_remote_log_data_block_t {
    uint32_t seqno;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t data[200];
}) fmav_remote_log_data_block_t;


#define FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK  184

#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX  206
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA  159

#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FLAGS  3
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FRAME_LEN_MAX  231

#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_NUM  200 // number of elements in array
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN  200 // length of array = number of bytes

#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_SEQNO_OFS  0
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_OFS  6


//----------------------------------------
//-- Message REMOTE_LOG_DATA_BLOCK packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_remote_log_data_block_t* _payload = (fmav_remote_log_data_block_t*)msg->payload;

    _payload->seqno = seqno;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*200);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_remote_log_data_block_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_remote_log_data_block_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seqno, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_remote_log_data_block_t* _payload = (fmav_remote_log_data_block_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->seqno = seqno;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*200);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_remote_log_data_block_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_remote_log_data_block_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->seqno, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_remote_log_data_block_t _payload;

    _payload.seqno = seqno;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*200);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_remote_log_data_block_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_remote_log_data_block_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message REMOTE_LOG_DATA_BLOCK unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_remote_log_data_block_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_remote_log_data_block_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_remote_log_data_block_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_remote_log_data_block_decode(fmav_remote_log_data_block_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_remote_log_data_block_get_field_seqno(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_remote_log_data_block_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_remote_log_data_block_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_remote_log_data_block_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_remote_log_data_block_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[6]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK  184

#define mavlink_remote_log_data_block_t  fmav_remote_log_data_block_t

#define MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_LEN  206
#define MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_MIN_LEN  206
#define MAVLINK_MSG_ID_184_LEN  206
#define MAVLINK_MSG_ID_184_MIN_LEN  206

#define MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_CRC  159
#define MAVLINK_MSG_ID_184_CRC  159

#define MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN 200


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_remote_log_data_block_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_remote_log_data_block_pack(
        msg, sysid, compid,
        target_system, target_component, seqno, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_remote_log_data_block_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t seqno, const uint8_t* data)
{
    return fmav_msg_remote_log_data_block_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, seqno, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_remote_log_data_block_decode(const mavlink_message_t* msg, mavlink_remote_log_data_block_t* payload)
{
    fmav_msg_remote_log_data_block_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_H

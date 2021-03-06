//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOG_DATA_H
#define FASTMAVLINK_MSG_LOG_DATA_H


//----------------------------------------
//-- Message LOG_DATA
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_log_data_t {
    uint32_t ofs;
    uint16_t id;
    uint8_t count;
    uint8_t data[90];
}) fmav_log_data_t;


#define FASTMAVLINK_MSG_ID_LOG_DATA  120

#define FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX  97
#define FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA  134

#define FASTMAVLINK_MSG_LOG_DATA_FLAGS  0
#define FASTMAVLINK_MSG_LOG_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOG_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LOG_DATA_FRAME_LEN_MAX  122

#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_NUM  90 // number of elements in array
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN  90 // length of array = number of bytes

#define FASTMAVLINK_MSG_LOG_DATA_FIELD_OFS_OFS  0
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_ID_OFS  4
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_COUNT_OFS  6
#define FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_OFS  7


//----------------------------------------
//-- Message LOG_DATA packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t* _payload = (fmav_log_data_t*)msg->payload;

    _payload->ofs = ofs;
    _payload->id = id;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*90);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LOG_DATA;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_data_pack(
        msg, sysid, compid,
        _payload->id, _payload->ofs, _payload->count, _payload->data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t* _payload = (fmav_log_data_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ofs = ofs;
    _payload->id = id;
    _payload->count = count;
    memcpy(&(_payload->data), data, sizeof(uint8_t)*90);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOG_DATA;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_DATA >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOG_DATA >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_log_data_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->id, _payload->ofs, _payload->count, _payload->data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data,
    fmav_status_t* _status)
{
    fmav_log_data_t _payload;

    _payload.ofs = ofs;
    _payload.id = id;
    _payload.count = count;
    memcpy(&(_payload.data), data, sizeof(uint8_t)*90);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOG_DATA,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_log_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOG_DATA,
        FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOG_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOG_DATA unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_log_data_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_log_data_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_data_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_log_data_decode(fmav_log_data_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOG_DATA_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_log_data_get_field_ofs(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_log_data_get_field_id(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_data_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_log_data_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[7]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_log_data_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_LOG_DATA_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[7]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOG_DATA  120

#define mavlink_log_data_t  fmav_log_data_t

#define MAVLINK_MSG_ID_LOG_DATA_LEN  97
#define MAVLINK_MSG_ID_LOG_DATA_MIN_LEN  97
#define MAVLINK_MSG_ID_120_LEN  97
#define MAVLINK_MSG_ID_120_MIN_LEN  97

#define MAVLINK_MSG_ID_LOG_DATA_CRC  134
#define MAVLINK_MSG_ID_120_CRC  134

#define MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN 90


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_log_data_pack(
        msg, sysid, compid,
        id, ofs, count, data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_log_data_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t id, uint32_t ofs, uint8_t count, const uint8_t* data)
{
    return fmav_msg_log_data_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        id, ofs, count, data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_log_data_decode(const mavlink_message_t* msg, mavlink_log_data_t* payload)
{
    fmav_msg_log_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOG_DATA_H

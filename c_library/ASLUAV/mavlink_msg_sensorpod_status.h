//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENSORPOD_STATUS_H
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_H


//----------------------------------------
//-- Message SENSORPOD_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sensorpod_status_t {
    uint64_t timestamp;
    uint16_t free_space;
    uint8_t visensor_rate_1;
    uint8_t visensor_rate_2;
    uint8_t visensor_rate_3;
    uint8_t visensor_rate_4;
    uint8_t recording_nodes_count;
    uint8_t cpu_temp;
}) fmav_sensorpod_status_t;


#define FASTMAVLINK_MSG_ID_SENSORPOD_STATUS  8012

#define FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_CRCEXTRA  54

#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FRAME_LEN_MAX  41



#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_FREE_SPACE_OFS  8
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_VISENSOR_RATE_1_OFS  10
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_VISENSOR_RATE_2_OFS  11
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_VISENSOR_RATE_3_OFS  12
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_VISENSOR_RATE_4_OFS  13
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_RECORDING_NODES_COUNT_OFS  14
#define FASTMAVLINK_MSG_SENSORPOD_STATUS_FIELD_CPU_TEMP_OFS  15


//----------------------------------------
//-- Message SENSORPOD_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space,
    fmav_status_t* _status)
{
    fmav_sensorpod_status_t* _payload = (fmav_sensorpod_status_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->free_space = free_space;
    _payload->visensor_rate_1 = visensor_rate_1;
    _payload->visensor_rate_2 = visensor_rate_2;
    _payload->visensor_rate_3 = visensor_rate_3;
    _payload->visensor_rate_4 = visensor_rate_4;
    _payload->recording_nodes_count = recording_nodes_count;
    _payload->cpu_temp = cpu_temp;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENSORPOD_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENSORPOD_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensorpod_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensorpod_status_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->visensor_rate_1, _payload->visensor_rate_2, _payload->visensor_rate_3, _payload->visensor_rate_4, _payload->recording_nodes_count, _payload->cpu_temp, _payload->free_space,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space,
    fmav_status_t* _status)
{
    fmav_sensorpod_status_t* _payload = (fmav_sensorpod_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->free_space = free_space;
    _payload->visensor_rate_1 = visensor_rate_1;
    _payload->visensor_rate_2 = visensor_rate_2;
    _payload->visensor_rate_3 = visensor_rate_3;
    _payload->visensor_rate_4 = visensor_rate_4;
    _payload->recording_nodes_count = recording_nodes_count;
    _payload->cpu_temp = cpu_temp;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENSORPOD_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSORPOD_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSORPOD_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensorpod_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensorpod_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->visensor_rate_1, _payload->visensor_rate_2, _payload->visensor_rate_3, _payload->visensor_rate_4, _payload->recording_nodes_count, _payload->cpu_temp, _payload->free_space,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space,
    fmav_status_t* _status)
{
    fmav_sensorpod_status_t _payload;

    _payload.timestamp = timestamp;
    _payload.free_space = free_space;
    _payload.visensor_rate_1 = visensor_rate_1;
    _payload.visensor_rate_2 = visensor_rate_2;
    _payload.visensor_rate_3 = visensor_rate_3;
    _payload.visensor_rate_4 = visensor_rate_4;
    _payload.recording_nodes_count = recording_nodes_count;
    _payload.cpu_temp = cpu_temp;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENSORPOD_STATUS,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensorpod_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENSORPOD_STATUS,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSORPOD_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENSORPOD_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sensorpod_status_decode(fmav_sensorpod_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSORPOD_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_sensorpod_status_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensorpod_status_get_field_free_space(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_visensor_rate_1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_visensor_rate_2(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_visensor_rate_3(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_visensor_rate_4(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_recording_nodes_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensorpod_status_get_field_cpu_temp(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENSORPOD_STATUS  8012

#define mavlink_sensorpod_status_t  fmav_sensorpod_status_t

#define MAVLINK_MSG_ID_SENSORPOD_STATUS_LEN  16
#define MAVLINK_MSG_ID_SENSORPOD_STATUS_MIN_LEN  16
#define MAVLINK_MSG_ID_8012_LEN  16
#define MAVLINK_MSG_ID_8012_MIN_LEN  16

#define MAVLINK_MSG_ID_SENSORPOD_STATUS_CRC  54
#define MAVLINK_MSG_ID_8012_CRC  54




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensorpod_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sensorpod_status_pack(
        _msg, sysid, compid,
        timestamp, visensor_rate_1, visensor_rate_2, visensor_rate_3, visensor_rate_4, recording_nodes_count, cpu_temp, free_space,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensorpod_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sensorpod_status_t* _payload)
{
    return mavlink_msg_sensorpod_status_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->visensor_rate_1, _payload->visensor_rate_2, _payload->visensor_rate_3, _payload->visensor_rate_4, _payload->recording_nodes_count, _payload->cpu_temp, _payload->free_space);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensorpod_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t visensor_rate_1, uint8_t visensor_rate_2, uint8_t visensor_rate_3, uint8_t visensor_rate_4, uint8_t recording_nodes_count, uint8_t cpu_temp, uint16_t free_space)
{
    return fmav_msg_sensorpod_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, visensor_rate_1, visensor_rate_2, visensor_rate_3, visensor_rate_4, recording_nodes_count, cpu_temp, free_space,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sensorpod_status_decode(const mavlink_message_t* msg, mavlink_sensorpod_status_t* payload)
{
    fmav_msg_sensorpod_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENSORPOD_STATUS_H

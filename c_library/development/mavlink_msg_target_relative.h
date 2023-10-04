//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TARGET_RELATIVE_H
#define FASTMAVLINK_MSG_TARGET_RELATIVE_H


//----------------------------------------
//-- Message TARGET_RELATIVE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_target_relative_t {
    uint64_t timestamp;
    float x;
    float y;
    float z;
    float pos_std[3];
    float yaw_std;
    float q_target[4];
    float q_sensor[4];
    uint8_t id;
    uint8_t frame;
    uint8_t type;
}) fmav_target_relative_t;


#define FASTMAVLINK_MSG_ID_TARGET_RELATIVE  511

#define FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX  71
#define FASTMAVLINK_MSG_TARGET_RELATIVE_CRCEXTRA  28

#define FASTMAVLINK_MSG_TARGET_RELATIVE_FLAGS  0
#define FASTMAVLINK_MSG_TARGET_RELATIVE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TARGET_RELATIVE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TARGET_RELATIVE_FRAME_LEN_MAX  96

#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_POS_STD_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_POS_STD_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_TARGET_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_TARGET_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_SENSOR_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_SENSOR_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_X_OFS  8
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Y_OFS  12
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Z_OFS  16
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_POS_STD_OFS  20
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_YAW_STD_OFS  32
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_TARGET_OFS  36
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_SENSOR_OFS  52
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_ID_OFS  68
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_FRAME_OFS  69
#define FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_TYPE_OFS  70


//----------------------------------------
//-- Message TARGET_RELATIVE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t frame, float x, float y, float z, const float* pos_std, float yaw_std, const float* q_target, const float* q_sensor, uint8_t type,
    fmav_status_t* _status)
{
    fmav_target_relative_t* _payload = (fmav_target_relative_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->yaw_std = yaw_std;
    _payload->id = id;
    _payload->frame = frame;
    _payload->type = type;
    memcpy(&(_payload->pos_std), pos_std, sizeof(float)*3);
    memcpy(&(_payload->q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload->q_sensor), q_sensor, sizeof(float)*4);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TARGET_RELATIVE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TARGET_RELATIVE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_relative_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_target_relative_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->id, _payload->frame, _payload->x, _payload->y, _payload->z, _payload->pos_std, _payload->yaw_std, _payload->q_target, _payload->q_sensor, _payload->type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t frame, float x, float y, float z, const float* pos_std, float yaw_std, const float* q_target, const float* q_sensor, uint8_t type,
    fmav_status_t* _status)
{
    fmav_target_relative_t* _payload = (fmav_target_relative_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->yaw_std = yaw_std;
    _payload->id = id;
    _payload->frame = frame;
    _payload->type = type;
    memcpy(&(_payload->pos_std), pos_std, sizeof(float)*3);
    memcpy(&(_payload->q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload->q_sensor), q_sensor, sizeof(float)*4);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TARGET_RELATIVE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TARGET_RELATIVE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TARGET_RELATIVE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_RELATIVE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_relative_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_target_relative_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->id, _payload->frame, _payload->x, _payload->y, _payload->z, _payload->pos_std, _payload->yaw_std, _payload->q_target, _payload->q_sensor, _payload->type,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t frame, float x, float y, float z, const float* pos_std, float yaw_std, const float* q_target, const float* q_sensor, uint8_t type,
    fmav_status_t* _status)
{
    fmav_target_relative_t _payload;

    _payload.timestamp = timestamp;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.yaw_std = yaw_std;
    _payload.id = id;
    _payload.frame = frame;
    _payload.type = type;
    memcpy(&(_payload.pos_std), pos_std, sizeof(float)*3);
    memcpy(&(_payload.q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload.q_sensor), q_sensor, sizeof(float)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TARGET_RELATIVE,
        FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_RELATIVE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_relative_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_relative_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TARGET_RELATIVE,
        FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_RELATIVE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TARGET_RELATIVE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_target_relative_decode(fmav_target_relative_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TARGET_RELATIVE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_target_relative_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_yaw_std(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_target_relative_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[68]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_target_relative_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[69]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_target_relative_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[70]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_relative_get_field_pos_std_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_pos_std(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_POS_STD_NUM) return 0;
    return ((float*)&(msg->payload[20]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_relative_get_field_q_target_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_q_target(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_TARGET_NUM) return 0;
    return ((float*)&(msg->payload[36]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_relative_get_field_q_sensor_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[52]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_relative_get_field_q_sensor(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_SENSOR_NUM) return 0;
    return ((float*)&(msg->payload[52]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TARGET_RELATIVE  511

#define mavlink_target_relative_t  fmav_target_relative_t

#define MAVLINK_MSG_ID_TARGET_RELATIVE_LEN  71
#define MAVLINK_MSG_ID_TARGET_RELATIVE_MIN_LEN  71
#define MAVLINK_MSG_ID_511_LEN  71
#define MAVLINK_MSG_ID_511_MIN_LEN  71

#define MAVLINK_MSG_ID_TARGET_RELATIVE_CRC  28
#define MAVLINK_MSG_ID_511_CRC  28

#define MAVLINK_MSG_TARGET_RELATIVE_FIELD_POS_STD_LEN 3
#define MAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_TARGET_LEN 4
#define MAVLINK_MSG_TARGET_RELATIVE_FIELD_Q_SENSOR_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_relative_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t id, uint8_t frame, float x, float y, float z, const float* pos_std, float yaw_std, const float* q_target, const float* q_sensor, uint8_t type)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_target_relative_pack(
        _msg, sysid, compid,
        timestamp, id, frame, x, y, z, pos_std, yaw_std, q_target, q_sensor, type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_relative_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_target_relative_t* _payload)
{
    return mavlink_msg_target_relative_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->id, _payload->frame, _payload->x, _payload->y, _payload->z, _payload->pos_std, _payload->yaw_std, _payload->q_target, _payload->q_sensor, _payload->type);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_relative_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t frame, float x, float y, float z, const float* pos_std, float yaw_std, const float* q_target, const float* q_sensor, uint8_t type)
{
    return fmav_msg_target_relative_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, id, frame, x, y, z, pos_std, yaw_std, q_target, q_sensor, type,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_target_relative_decode(const mavlink_message_t* msg, mavlink_target_relative_t* payload)
{
    fmav_msg_target_relative_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TARGET_RELATIVE_H

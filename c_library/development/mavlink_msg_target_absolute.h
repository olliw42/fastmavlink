//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TARGET_ABSOLUTE_H
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_H


//----------------------------------------
//-- Message TARGET_ABSOLUTE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_target_absolute_t {
    uint64_t timestamp;
    int32_t lat;
    int32_t lon;
    float alt;
    float vel[3];
    float acc[3];
    float q_target[4];
    float rates[3];
    float position_std[2];
    float vel_std[3];
    float acc_std[3];
    uint8_t id;
    uint8_t sensor_capabilities;
}) fmav_target_absolute_t;


#define FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE  510

#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX  106
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_CRCEXTRA  245

#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FLAGS  0
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FRAME_LEN_MAX  131

#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_Q_TARGET_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_Q_TARGET_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_RATES_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_RATES_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_POSITION_STD_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_POSITION_STD_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_STD_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_STD_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_STD_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_STD_LEN  12 // length of array = number of bytes

#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_LAT_OFS  8
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_LON_OFS  12
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ALT_OFS  16
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_OFS  20
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_OFS  32
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_Q_TARGET_OFS  44
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_RATES_OFS  60
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_POSITION_STD_OFS  72
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_STD_OFS  80
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_STD_OFS  92
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ID_OFS  104
#define FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_SENSOR_CAPABILITIES_OFS  105


//----------------------------------------
//-- Message TARGET_ABSOLUTE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t sensor_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* q_target, const float* rates, const float* position_std, const float* vel_std, const float* acc_std,
    fmav_status_t* _status)
{
    fmav_target_absolute_t* _payload = (fmav_target_absolute_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->id = id;
    _payload->sensor_capabilities = sensor_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_std), position_std, sizeof(float)*2);
    memcpy(&(_payload->vel_std), vel_std, sizeof(float)*3);
    memcpy(&(_payload->acc_std), acc_std, sizeof(float)*3);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TARGET_ABSOLUTE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_absolute_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_target_absolute_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->id, _payload->sensor_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->q_target, _payload->rates, _payload->position_std, _payload->vel_std, _payload->acc_std,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t sensor_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* q_target, const float* rates, const float* position_std, const float* vel_std, const float* acc_std,
    fmav_status_t* _status)
{
    fmav_target_absolute_t* _payload = (fmav_target_absolute_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->id = id;
    _payload->sensor_capabilities = sensor_capabilities;
    memcpy(&(_payload->vel), vel, sizeof(float)*3);
    memcpy(&(_payload->acc), acc, sizeof(float)*3);
    memcpy(&(_payload->q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload->rates), rates, sizeof(float)*3);
    memcpy(&(_payload->position_std), position_std, sizeof(float)*2);
    memcpy(&(_payload->vel_std), vel_std, sizeof(float)*3);
    memcpy(&(_payload->acc_std), acc_std, sizeof(float)*3);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_absolute_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_target_absolute_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->id, _payload->sensor_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->q_target, _payload->rates, _payload->position_std, _payload->vel_std, _payload->acc_std,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t sensor_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* q_target, const float* rates, const float* position_std, const float* vel_std, const float* acc_std,
    fmav_status_t* _status)
{
    fmav_target_absolute_t _payload;

    _payload.timestamp = timestamp;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.id = id;
    _payload.sensor_capabilities = sensor_capabilities;
    memcpy(&(_payload.vel), vel, sizeof(float)*3);
    memcpy(&(_payload.acc), acc, sizeof(float)*3);
    memcpy(&(_payload.q_target), q_target, sizeof(float)*4);
    memcpy(&(_payload.rates), rates, sizeof(float)*3);
    memcpy(&(_payload.position_std), position_std, sizeof(float)*2);
    memcpy(&(_payload.vel_std), vel_std, sizeof(float)*3);
    memcpy(&(_payload.acc_std), acc_std, sizeof(float)*3);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_target_absolute_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_target_absolute_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TARGET_ABSOLUTE,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TARGET_ABSOLUTE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TARGET_ABSOLUTE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_target_absolute_decode(fmav_target_absolute_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TARGET_ABSOLUTE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_target_absolute_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_target_absolute_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_target_absolute_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_target_absolute_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[104]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_target_absolute_get_field_sensor_capabilities(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[105]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_vel_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_vel(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_NUM) return 0;
    return ((float*)&(msg->payload[20]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_acc_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[32]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_acc(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_NUM) return 0;
    return ((float*)&(msg->payload[32]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_q_target_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[44]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_q_target(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_Q_TARGET_NUM) return 0;
    return ((float*)&(msg->payload[44]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_rates_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[60]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_rates(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_RATES_NUM) return 0;
    return ((float*)&(msg->payload[60]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_position_std_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[72]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_position_std(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_POSITION_STD_NUM) return 0;
    return ((float*)&(msg->payload[72]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_vel_std_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[80]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_vel_std(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_STD_NUM) return 0;
    return ((float*)&(msg->payload[80]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_target_absolute_get_field_acc_std_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[92]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_target_absolute_get_field_acc_std(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_STD_NUM) return 0;
    return ((float*)&(msg->payload[92]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TARGET_ABSOLUTE  510

#define mavlink_target_absolute_t  fmav_target_absolute_t

#define MAVLINK_MSG_ID_TARGET_ABSOLUTE_LEN  106
#define MAVLINK_MSG_ID_TARGET_ABSOLUTE_MIN_LEN  106
#define MAVLINK_MSG_ID_510_LEN  106
#define MAVLINK_MSG_ID_510_MIN_LEN  106

#define MAVLINK_MSG_ID_TARGET_ABSOLUTE_CRC  245
#define MAVLINK_MSG_ID_510_CRC  245

#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_LEN 3
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_LEN 3
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_Q_TARGET_LEN 4
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_RATES_LEN 3
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_POSITION_STD_LEN 2
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_VEL_STD_LEN 3
#define MAVLINK_MSG_TARGET_ABSOLUTE_FIELD_ACC_STD_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_absolute_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t id, uint8_t sensor_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* q_target, const float* rates, const float* position_std, const float* vel_std, const float* acc_std)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_target_absolute_pack(
        _msg, sysid, compid,
        timestamp, id, sensor_capabilities, lat, lon, alt, vel, acc, q_target, rates, position_std, vel_std, acc_std,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_absolute_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_target_absolute_t* _payload)
{
    return mavlink_msg_target_absolute_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->id, _payload->sensor_capabilities, _payload->lat, _payload->lon, _payload->alt, _payload->vel, _payload->acc, _payload->q_target, _payload->rates, _payload->position_std, _payload->vel_std, _payload->acc_std);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_target_absolute_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t id, uint8_t sensor_capabilities, int32_t lat, int32_t lon, float alt, const float* vel, const float* acc, const float* q_target, const float* rates, const float* position_std, const float* vel_std, const float* acc_std)
{
    return fmav_msg_target_absolute_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, id, sensor_capabilities, lat, lon, alt, vel, acc, q_target, rates, position_std, vel_std, acc_std,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_target_absolute_decode(const mavlink_message_t* msg, mavlink_target_absolute_t* payload)
{
    fmav_msg_target_absolute_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TARGET_ABSOLUTE_H

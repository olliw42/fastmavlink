//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENSOR_OFFSETS_H
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_H


//----------------------------------------
//-- Message SENSOR_OFFSETS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sensor_offsets_t {
    float mag_declination;
    int32_t raw_press;
    int32_t raw_temp;
    float gyro_cal_x;
    float gyro_cal_y;
    float gyro_cal_z;
    float accel_cal_x;
    float accel_cal_y;
    float accel_cal_z;
    int16_t mag_ofs_x;
    int16_t mag_ofs_y;
    int16_t mag_ofs_z;
}) fmav_sensor_offsets_t;


#define FASTMAVLINK_MSG_ID_SENSOR_OFFSETS  150

#define FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA  134

#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FLAGS  0
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_MAG_DECLINATION_OFS  0
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_RAW_PRESS_OFS  4
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_RAW_TEMP_OFS  8
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_GYRO_CAL_X_OFS  12
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_GYRO_CAL_Y_OFS  16
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_GYRO_CAL_Z_OFS  20
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_ACCEL_CAL_X_OFS  24
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_ACCEL_CAL_Y_OFS  28
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_ACCEL_CAL_Z_OFS  32
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_MAG_OFS_X_OFS  36
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_MAG_OFS_Y_OFS  38
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FIELD_MAG_OFS_Z_OFS  40


//----------------------------------------
//-- Message SENSOR_OFFSETS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z,
    fmav_status_t* _status)
{
    fmav_sensor_offsets_t* _payload = (fmav_sensor_offsets_t*)_msg->payload;

    _payload->mag_declination = mag_declination;
    _payload->raw_press = raw_press;
    _payload->raw_temp = raw_temp;
    _payload->gyro_cal_x = gyro_cal_x;
    _payload->gyro_cal_y = gyro_cal_y;
    _payload->gyro_cal_z = gyro_cal_z;
    _payload->accel_cal_x = accel_cal_x;
    _payload->accel_cal_y = accel_cal_y;
    _payload->accel_cal_z = accel_cal_z;
    _payload->mag_ofs_x = mag_ofs_x;
    _payload->mag_ofs_y = mag_ofs_y;
    _payload->mag_ofs_z = mag_ofs_z;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENSOR_OFFSETS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_offsets_pack(
        _msg, sysid, compid,
        _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z, _payload->mag_declination, _payload->raw_press, _payload->raw_temp, _payload->gyro_cal_x, _payload->gyro_cal_y, _payload->gyro_cal_z, _payload->accel_cal_x, _payload->accel_cal_y, _payload->accel_cal_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z,
    fmav_status_t* _status)
{
    fmav_sensor_offsets_t* _payload = (fmav_sensor_offsets_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mag_declination = mag_declination;
    _payload->raw_press = raw_press;
    _payload->raw_temp = raw_temp;
    _payload->gyro_cal_x = gyro_cal_x;
    _payload->gyro_cal_y = gyro_cal_y;
    _payload->gyro_cal_z = gyro_cal_z;
    _payload->accel_cal_x = accel_cal_x;
    _payload->accel_cal_y = accel_cal_y;
    _payload->accel_cal_z = accel_cal_z;
    _payload->mag_ofs_x = mag_ofs_x;
    _payload->mag_ofs_y = mag_ofs_y;
    _payload->mag_ofs_z = mag_ofs_z;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_offsets_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z, _payload->mag_declination, _payload->raw_press, _payload->raw_temp, _payload->gyro_cal_x, _payload->gyro_cal_y, _payload->gyro_cal_z, _payload->accel_cal_x, _payload->accel_cal_y, _payload->accel_cal_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z,
    fmav_status_t* _status)
{
    fmav_sensor_offsets_t _payload;

    _payload.mag_declination = mag_declination;
    _payload.raw_press = raw_press;
    _payload.raw_temp = raw_temp;
    _payload.gyro_cal_x = gyro_cal_x;
    _payload.gyro_cal_y = gyro_cal_y;
    _payload.gyro_cal_z = gyro_cal_z;
    _payload.accel_cal_x = accel_cal_x;
    _payload.accel_cal_y = accel_cal_y;
    _payload.accel_cal_z = accel_cal_z;
    _payload.mag_ofs_x = mag_ofs_x;
    _payload.mag_ofs_y = mag_ofs_y;
    _payload.mag_ofs_z = mag_ofs_z;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENSOR_OFFSETS,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENSOR_OFFSETS,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENSOR_OFFSETS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sensor_offsets_decode(fmav_sensor_offsets_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_mag_declination(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_sensor_offsets_get_field_raw_press(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_sensor_offsets_get_field_raw_temp(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_gyro_cal_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_gyro_cal_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_gyro_cal_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_accel_cal_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_accel_cal_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_offsets_get_field_accel_cal_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_sensor_offsets_get_field_mag_ofs_x(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_sensor_offsets_get_field_mag_ofs_y(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_sensor_offsets_get_field_mag_ofs_z(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENSOR_OFFSETS  150

#define mavlink_sensor_offsets_t  fmav_sensor_offsets_t

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN  42
#define MAVLINK_MSG_ID_SENSOR_OFFSETS_MIN_LEN  42
#define MAVLINK_MSG_ID_150_LEN  42
#define MAVLINK_MSG_ID_150_MIN_LEN  42

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC  134
#define MAVLINK_MSG_ID_150_CRC  134




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_offsets_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sensor_offsets_pack(
        _msg, sysid, compid,
        mag_ofs_x, mag_ofs_y, mag_ofs_z, mag_declination, raw_press, raw_temp, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_offsets_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sensor_offsets_t* _payload)
{
    return mavlink_msg_sensor_offsets_pack(
        sysid,
        compid,
        _msg,
        _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z, _payload->mag_declination, _payload->raw_press, _payload->raw_temp, _payload->gyro_cal_x, _payload->gyro_cal_y, _payload->gyro_cal_z, _payload->accel_cal_x, _payload->accel_cal_y, _payload->accel_cal_z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_offsets_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
    return fmav_msg_sensor_offsets_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        mag_ofs_x, mag_ofs_y, mag_ofs_z, mag_declination, raw_press, raw_temp, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sensor_offsets_decode(const mavlink_message_t* msg, mavlink_sensor_offsets_t* payload)
{
    fmav_msg_sensor_offsets_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENSOR_OFFSETS_H

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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA  134

#define FASTMAVLINK_MSG_ID_150_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_150_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_150_LEN  42
#define FASTMAVLINK_MSG_ID_150_CRCEXTRA  134



#define FASTMAVLINK_MSG_SENSOR_OFFSETS_FLAGS  0
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENSOR_OFFSETS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SENSOR_OFFSETS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z,
    fmav_status_t* _status)
{
    fmav_sensor_offsets_t* _payload = (fmav_sensor_offsets_t*)msg->payload;

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


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SENSOR_OFFSETS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_offsets_pack(
        msg, sysid, compid,
        _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z, _payload->mag_declination, _payload->raw_press, _payload->raw_temp, _payload->gyro_cal_x, _payload->gyro_cal_y, _payload->gyro_cal_z, _payload->accel_cal_x, _payload->accel_cal_y, _payload->accel_cal_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z,
    fmav_status_t* _status)
{
    fmav_sensor_offsets_t* _payload = (fmav_sensor_offsets_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_OFFSETS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_OFFSETS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_offsets_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_offsets_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z, _payload->mag_declination, _payload->raw_press, _payload->raw_temp, _payload->gyro_cal_x, _payload->gyro_cal_y, _payload->gyro_cal_z, _payload->accel_cal_x, _payload->accel_cal_y, _payload->accel_cal_z,
        _status);
}


//----------------------------------------
//-- Message SENSOR_OFFSETS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sensor_offsets_decode(fmav_sensor_offsets_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SENSOR_OFFSETS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sensor_offsets_pack(
        msg, sysid, compid,
        mag_ofs_x, mag_ofs_y, mag_ofs_z, mag_declination, raw_press, raw_temp, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_offsets_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
    return fmav_msg_sensor_offsets_pack_to_frame_buf(
        (uint8_t*)buf,
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

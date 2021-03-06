//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_H
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_H


//----------------------------------------
//-- Message HIL_OPTICAL_FLOW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_optical_flow_t {
    uint64_t time_usec;
    uint32_t integration_time_us;
    float integrated_x;
    float integrated_y;
    float integrated_xgyro;
    float integrated_ygyro;
    float integrated_zgyro;
    uint32_t time_delta_distance_us;
    float distance;
    int16_t temperature;
    uint8_t sensor_id;
    uint8_t quality;
}) fmav_hil_optical_flow_t;


#define FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW  114

#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX  44
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA  237

#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FLAGS  0
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FRAME_LEN_MAX  69



#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATION_TIME_US_OFS  8
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATED_X_OFS  12
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATED_Y_OFS  16
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATED_XGYRO_OFS  20
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATED_YGYRO_OFS  24
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_INTEGRATED_ZGYRO_OFS  28
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_TIME_DELTA_DISTANCE_US_OFS  32
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_DISTANCE_OFS  36
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_TEMPERATURE_OFS  40
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_SENSOR_ID_OFS  42
#define FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_FIELD_QUALITY_OFS  43


//----------------------------------------
//-- Message HIL_OPTICAL_FLOW packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance,
    fmav_status_t* _status)
{
    fmav_hil_optical_flow_t* _payload = (fmav_hil_optical_flow_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->integration_time_us = integration_time_us;
    _payload->integrated_x = integrated_x;
    _payload->integrated_y = integrated_y;
    _payload->integrated_xgyro = integrated_xgyro;
    _payload->integrated_ygyro = integrated_ygyro;
    _payload->integrated_zgyro = integrated_zgyro;
    _payload->time_delta_distance_us = time_delta_distance_us;
    _payload->distance = distance;
    _payload->temperature = temperature;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_optical_flow_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->integration_time_us, _payload->integrated_x, _payload->integrated_y, _payload->integrated_xgyro, _payload->integrated_ygyro, _payload->integrated_zgyro, _payload->temperature, _payload->quality, _payload->time_delta_distance_us, _payload->distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance,
    fmav_status_t* _status)
{
    fmav_hil_optical_flow_t* _payload = (fmav_hil_optical_flow_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->integration_time_us = integration_time_us;
    _payload->integrated_x = integrated_x;
    _payload->integrated_y = integrated_y;
    _payload->integrated_xgyro = integrated_xgyro;
    _payload->integrated_ygyro = integrated_ygyro;
    _payload->integrated_zgyro = integrated_zgyro;
    _payload->time_delta_distance_us = time_delta_distance_us;
    _payload->distance = distance;
    _payload->temperature = temperature;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_optical_flow_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->integration_time_us, _payload->integrated_x, _payload->integrated_y, _payload->integrated_xgyro, _payload->integrated_ygyro, _payload->integrated_zgyro, _payload->temperature, _payload->quality, _payload->time_delta_distance_us, _payload->distance,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance,
    fmav_status_t* _status)
{
    fmav_hil_optical_flow_t _payload;

    _payload.time_usec = time_usec;
    _payload.integration_time_us = integration_time_us;
    _payload.integrated_x = integrated_x;
    _payload.integrated_y = integrated_y;
    _payload.integrated_xgyro = integrated_xgyro;
    _payload.integrated_ygyro = integrated_ygyro;
    _payload.integrated_zgyro = integrated_zgyro;
    _payload.time_delta_distance_us = time_delta_distance_us;
    _payload.distance = distance;
    _payload.temperature = temperature;
    _payload.sensor_id = sensor_id;
    _payload.quality = quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_optical_flow_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HIL_OPTICAL_FLOW,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HIL_OPTICAL_FLOW unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hil_optical_flow_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hil_optical_flow_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_optical_flow_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_optical_flow_decode(fmav_hil_optical_flow_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_hil_optical_flow_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_hil_optical_flow_get_field_integration_time_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_integrated_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_integrated_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_integrated_xgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_integrated_ygyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_integrated_zgyro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_hil_optical_flow_get_field_time_delta_distance_us(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_hil_optical_flow_get_field_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_hil_optical_flow_get_field_temperature(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_optical_flow_get_field_sensor_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hil_optical_flow_get_field_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[43]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW  114

#define mavlink_hil_optical_flow_t  fmav_hil_optical_flow_t

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN  44
#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN  44
#define MAVLINK_MSG_ID_114_LEN  44
#define MAVLINK_MSG_ID_114_MIN_LEN  44

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC  237
#define MAVLINK_MSG_ID_114_CRC  237




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_optical_flow_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_optical_flow_pack(
        msg, sysid, compid,
        time_usec, sensor_id, integration_time_us, integrated_x, integrated_y, integrated_xgyro, integrated_ygyro, integrated_zgyro, temperature, quality, time_delta_distance_us, distance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_optical_flow_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
    return fmav_msg_hil_optical_flow_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, sensor_id, integration_time_us, integrated_x, integrated_y, integrated_xgyro, integrated_ygyro, integrated_zgyro, temperature, quality, time_delta_distance_us, distance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_optical_flow_decode(const mavlink_message_t* msg, mavlink_hil_optical_flow_t* payload)
{
    fmav_msg_hil_optical_flow_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_OPTICAL_FLOW_H

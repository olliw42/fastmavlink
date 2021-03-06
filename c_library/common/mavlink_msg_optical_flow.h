//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPTICAL_FLOW_H
#define FASTMAVLINK_MSG_OPTICAL_FLOW_H


//----------------------------------------
//-- Message OPTICAL_FLOW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_optical_flow_t {
    uint64_t time_usec;
    float flow_comp_m_x;
    float flow_comp_m_y;
    float ground_distance;
    int16_t flow_x;
    int16_t flow_y;
    uint8_t sensor_id;
    uint8_t quality;
    float flow_rate_x;
    float flow_rate_y;
}) fmav_optical_flow_t;


#define FASTMAVLINK_MSG_ID_OPTICAL_FLOW  100

#define FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX  34
#define FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA  175

#define FASTMAVLINK_MSG_OPTICAL_FLOW_FLAGS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OPTICAL_FLOW_FRAME_LEN_MAX  59



#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_COMP_M_X_OFS  8
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_COMP_M_Y_OFS  12
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_GROUND_DISTANCE_OFS  16
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_X_OFS  20
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_Y_OFS  22
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_SENSOR_ID_OFS  24
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_QUALITY_OFS  25
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_RATE_X_OFS  26
#define FASTMAVLINK_MSG_OPTICAL_FLOW_FIELD_FLOW_RATE_Y_OFS  30


//----------------------------------------
//-- Message OPTICAL_FLOW packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t* _payload = (fmav_optical_flow_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->flow_comp_m_x = flow_comp_m_x;
    _payload->flow_comp_m_y = flow_comp_m_y;
    _payload->ground_distance = ground_distance;
    _payload->flow_x = flow_x;
    _payload->flow_y = flow_y;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;
    _payload->flow_rate_x = flow_rate_x;
    _payload->flow_rate_y = flow_rate_y;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OPTICAL_FLOW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_optical_flow_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->flow_x, _payload->flow_y, _payload->flow_comp_m_x, _payload->flow_comp_m_y, _payload->quality, _payload->ground_distance, _payload->flow_rate_x, _payload->flow_rate_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t* _payload = (fmav_optical_flow_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->flow_comp_m_x = flow_comp_m_x;
    _payload->flow_comp_m_y = flow_comp_m_y;
    _payload->ground_distance = ground_distance;
    _payload->flow_x = flow_x;
    _payload->flow_y = flow_y;
    _payload->sensor_id = sensor_id;
    _payload->quality = quality;
    _payload->flow_rate_x = flow_rate_x;
    _payload->flow_rate_y = flow_rate_y;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPTICAL_FLOW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_optical_flow_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->sensor_id, _payload->flow_x, _payload->flow_y, _payload->flow_comp_m_x, _payload->flow_comp_m_y, _payload->quality, _payload->ground_distance, _payload->flow_rate_x, _payload->flow_rate_y,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y,
    fmav_status_t* _status)
{
    fmav_optical_flow_t _payload;

    _payload.time_usec = time_usec;
    _payload.flow_comp_m_x = flow_comp_m_x;
    _payload.flow_comp_m_y = flow_comp_m_y;
    _payload.ground_distance = ground_distance;
    _payload.flow_x = flow_x;
    _payload.flow_y = flow_y;
    _payload.sensor_id = sensor_id;
    _payload.quality = quality;
    _payload.flow_rate_x = flow_rate_x;
    _payload.flow_rate_y = flow_rate_y;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPTICAL_FLOW,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_optical_flow_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_optical_flow_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPTICAL_FLOW,
        FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPTICAL_FLOW_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPTICAL_FLOW unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_optical_flow_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_optical_flow_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_optical_flow_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_optical_flow_decode(fmav_optical_flow_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPTICAL_FLOW_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_optical_flow_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_comp_m_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_comp_m_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_ground_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_optical_flow_get_field_flow_x(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_optical_flow_get_field_flow_y(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_optical_flow_get_field_sensor_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_optical_flow_get_field_quality(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[25]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_rate_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[26]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_optical_flow_get_field_flow_rate_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[30]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPTICAL_FLOW  100

#define mavlink_optical_flow_t  fmav_optical_flow_t

#define MAVLINK_MSG_ID_OPTICAL_FLOW_LEN  34
#define MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN  26
#define MAVLINK_MSG_ID_100_LEN  34
#define MAVLINK_MSG_ID_100_MIN_LEN  26

#define MAVLINK_MSG_ID_OPTICAL_FLOW_CRC  175
#define MAVLINK_MSG_ID_100_CRC  175




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_optical_flow_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_optical_flow_pack(
        msg, sysid, compid,
        time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance, flow_rate_x, flow_rate_y,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_optical_flow_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t sensor_id, int16_t flow_x, int16_t flow_y, float flow_comp_m_x, float flow_comp_m_y, uint8_t quality, float ground_distance, float flow_rate_x, float flow_rate_y)
{
    return fmav_msg_optical_flow_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance, flow_rate_x, flow_rate_y,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_optical_flow_decode(const mavlink_message_t* msg, mavlink_optical_flow_t* payload)
{
    fmav_msg_optical_flow_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPTICAL_FLOW_H

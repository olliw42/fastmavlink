//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RANGEFINDER_H
#define FASTMAVLINK_MSG_RANGEFINDER_H


//----------------------------------------
//-- Message RANGEFINDER
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rangefinder_t {
    float distance;
    float voltage;
}) fmav_rangefinder_t;


#define FASTMAVLINK_MSG_ID_RANGEFINDER  173

#define FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA  83

#define FASTMAVLINK_MSG_RANGEFINDER_FLAGS  0
#define FASTMAVLINK_MSG_RANGEFINDER_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RANGEFINDER_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RANGEFINDER_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_RANGEFINDER_FIELD_DISTANCE_OFS  0
#define FASTMAVLINK_MSG_RANGEFINDER_FIELD_VOLTAGE_OFS  4


//----------------------------------------
//-- Message RANGEFINDER packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float distance, float voltage,
    fmav_status_t* _status)
{
    fmav_rangefinder_t* _payload = (fmav_rangefinder_t*)msg->payload;

    _payload->distance = distance;
    _payload->voltage = voltage;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RANGEFINDER;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rangefinder_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rangefinder_pack(
        msg, sysid, compid,
        _payload->distance, _payload->voltage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float distance, float voltage,
    fmav_status_t* _status)
{
    fmav_rangefinder_t* _payload = (fmav_rangefinder_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->distance = distance;
    _payload->voltage = voltage;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RANGEFINDER;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RANGEFINDER >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RANGEFINDER >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rangefinder_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rangefinder_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->distance, _payload->voltage,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float distance, float voltage,
    fmav_status_t* _status)
{
    fmav_rangefinder_t _payload;

    _payload.distance = distance;
    _payload.voltage = voltage;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RANGEFINDER,
        FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rangefinder_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rangefinder_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RANGEFINDER,
        FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RANGEFINDER_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RANGEFINDER unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_rangefinder_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_rangefinder_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rangefinder_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rangefinder_decode(fmav_rangefinder_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RANGEFINDER_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rangefinder_get_field_distance(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rangefinder_get_field_voltage(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RANGEFINDER  173

#define mavlink_rangefinder_t  fmav_rangefinder_t

#define MAVLINK_MSG_ID_RANGEFINDER_LEN  8
#define MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN  8
#define MAVLINK_MSG_ID_173_LEN  8
#define MAVLINK_MSG_ID_173_MIN_LEN  8

#define MAVLINK_MSG_ID_RANGEFINDER_CRC  83
#define MAVLINK_MSG_ID_173_CRC  83




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rangefinder_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float distance, float voltage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rangefinder_pack(
        msg, sysid, compid,
        distance, voltage,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rangefinder_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float distance, float voltage)
{
    return fmav_msg_rangefinder_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        distance, voltage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rangefinder_decode(const mavlink_message_t* msg, mavlink_rangefinder_t* payload)
{
    fmav_msg_rangefinder_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RANGEFINDER_H

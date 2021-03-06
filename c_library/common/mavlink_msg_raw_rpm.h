//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RAW_RPM_H
#define FASTMAVLINK_MSG_RAW_RPM_H


//----------------------------------------
//-- Message RAW_RPM
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_raw_rpm_t {
    float frequency;
    uint8_t index;
}) fmav_raw_rpm_t;


#define FASTMAVLINK_MSG_ID_RAW_RPM  339

#define FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA  199

#define FASTMAVLINK_MSG_RAW_RPM_FLAGS  0
#define FASTMAVLINK_MSG_RAW_RPM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RAW_RPM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RAW_RPM_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_RAW_RPM_FIELD_FREQUENCY_OFS  0
#define FASTMAVLINK_MSG_RAW_RPM_FIELD_INDEX_OFS  4


//----------------------------------------
//-- Message RAW_RPM packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t* _payload = (fmav_raw_rpm_t*)msg->payload;

    _payload->frequency = frequency;
    _payload->index = index;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RAW_RPM;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_rpm_pack(
        msg, sysid, compid,
        _payload->index, _payload->frequency,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t* _payload = (fmav_raw_rpm_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->frequency = frequency;
    _payload->index = index;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RAW_RPM;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_RPM >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RAW_RPM >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_raw_rpm_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->index, _payload->frequency,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency,
    fmav_status_t* _status)
{
    fmav_raw_rpm_t _payload;

    _payload.frequency = frequency;
    _payload.index = index;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RAW_RPM,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_raw_rpm_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_raw_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RAW_RPM,
        FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RAW_RPM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RAW_RPM unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_raw_rpm_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_raw_rpm_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_rpm_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_raw_rpm_decode(fmav_raw_rpm_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RAW_RPM_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_raw_rpm_get_field_frequency(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_raw_rpm_get_field_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RAW_RPM  339

#define mavlink_raw_rpm_t  fmav_raw_rpm_t

#define MAVLINK_MSG_ID_RAW_RPM_LEN  5
#define MAVLINK_MSG_ID_RAW_RPM_MIN_LEN  5
#define MAVLINK_MSG_ID_339_LEN  5
#define MAVLINK_MSG_ID_339_MIN_LEN  5

#define MAVLINK_MSG_ID_RAW_RPM_CRC  199
#define MAVLINK_MSG_ID_339_CRC  199




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_rpm_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t index, float frequency)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_raw_rpm_pack(
        msg, sysid, compid,
        index, frequency,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_raw_rpm_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t index, float frequency)
{
    return fmav_msg_raw_rpm_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        index, frequency,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_raw_rpm_decode(const mavlink_message_t* msg, mavlink_raw_rpm_t* payload)
{
    fmav_msg_raw_rpm_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RAW_RPM_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ESTIMATOR_STATUS_H
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_H


//----------------------------------------
//-- Message ESTIMATOR_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_estimator_status_t {
    uint64_t time_usec;
    float vel_ratio;
    float pos_horiz_ratio;
    float pos_vert_ratio;
    float mag_ratio;
    float hagl_ratio;
    float tas_ratio;
    float pos_horiz_accuracy;
    float pos_vert_accuracy;
    uint16_t flags;
}) fmav_estimator_status_t;


#define FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS  230

#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA  163

#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FRAME_LEN_MAX  67



#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_VEL_RATIO_OFS  8
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_POS_HORIZ_RATIO_OFS  12
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_POS_VERT_RATIO_OFS  16
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_MAG_RATIO_OFS  20
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_HAGL_RATIO_OFS  24
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_TAS_RATIO_OFS  28
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_POS_HORIZ_ACCURACY_OFS  32
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_POS_VERT_ACCURACY_OFS  36
#define FASTMAVLINK_MSG_ESTIMATOR_STATUS_FIELD_FLAGS_OFS  40


//----------------------------------------
//-- Message ESTIMATOR_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy,
    fmav_status_t* _status)
{
    fmav_estimator_status_t* _payload = (fmav_estimator_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->vel_ratio = vel_ratio;
    _payload->pos_horiz_ratio = pos_horiz_ratio;
    _payload->pos_vert_ratio = pos_vert_ratio;
    _payload->mag_ratio = mag_ratio;
    _payload->hagl_ratio = hagl_ratio;
    _payload->tas_ratio = tas_ratio;
    _payload->pos_horiz_accuracy = pos_horiz_accuracy;
    _payload->pos_vert_accuracy = pos_vert_accuracy;
    _payload->flags = flags;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_estimator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_estimator_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->flags, _payload->vel_ratio, _payload->pos_horiz_ratio, _payload->pos_vert_ratio, _payload->mag_ratio, _payload->hagl_ratio, _payload->tas_ratio, _payload->pos_horiz_accuracy, _payload->pos_vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy,
    fmav_status_t* _status)
{
    fmav_estimator_status_t* _payload = (fmav_estimator_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->vel_ratio = vel_ratio;
    _payload->pos_horiz_ratio = pos_horiz_ratio;
    _payload->pos_vert_ratio = pos_vert_ratio;
    _payload->mag_ratio = mag_ratio;
    _payload->hagl_ratio = hagl_ratio;
    _payload->tas_ratio = tas_ratio;
    _payload->pos_horiz_accuracy = pos_horiz_accuracy;
    _payload->pos_vert_accuracy = pos_vert_accuracy;
    _payload->flags = flags;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_estimator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_estimator_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->flags, _payload->vel_ratio, _payload->pos_horiz_ratio, _payload->pos_vert_ratio, _payload->mag_ratio, _payload->hagl_ratio, _payload->tas_ratio, _payload->pos_horiz_accuracy, _payload->pos_vert_accuracy,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy,
    fmav_status_t* _status)
{
    fmav_estimator_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.vel_ratio = vel_ratio;
    _payload.pos_horiz_ratio = pos_horiz_ratio;
    _payload.pos_vert_ratio = pos_vert_ratio;
    _payload.mag_ratio = mag_ratio;
    _payload.hagl_ratio = hagl_ratio;
    _payload.tas_ratio = tas_ratio;
    _payload.pos_horiz_accuracy = pos_horiz_accuracy;
    _payload.pos_vert_accuracy = pos_vert_accuracy;
    _payload.flags = flags;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_estimator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ESTIMATOR_STATUS,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ESTIMATOR_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ESTIMATOR_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_estimator_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_estimator_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_estimator_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_estimator_status_decode(fmav_estimator_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ESTIMATOR_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_estimator_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_vel_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_pos_horiz_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_pos_vert_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_mag_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_hagl_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_tas_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_pos_horiz_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_estimator_status_get_field_pos_vert_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_estimator_status_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS  230

#define mavlink_estimator_status_t  fmav_estimator_status_t

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN  42
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN  42
#define MAVLINK_MSG_ID_230_LEN  42
#define MAVLINK_MSG_ID_230_MIN_LEN  42

#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_CRC  163
#define MAVLINK_MSG_ID_230_CRC  163




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_estimator_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_estimator_status_pack(
        msg, sysid, compid,
        time_usec, flags, vel_ratio, pos_horiz_ratio, pos_vert_ratio, mag_ratio, hagl_ratio, tas_ratio, pos_horiz_accuracy, pos_vert_accuracy,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_estimator_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t flags, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy)
{
    return fmav_msg_estimator_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, flags, vel_ratio, pos_horiz_ratio, pos_vert_ratio, mag_ratio, hagl_ratio, tas_ratio, pos_horiz_accuracy, pos_vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_estimator_status_decode(const mavlink_message_t* msg, mavlink_estimator_status_t* payload)
{
    fmav_msg_estimator_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ESTIMATOR_STATUS_H

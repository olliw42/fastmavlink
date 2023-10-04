//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MAG_CAL_PROGRESS_H
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_H


//----------------------------------------
//-- Message MAG_CAL_PROGRESS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mag_cal_progress_t {
    float direction_x;
    float direction_y;
    float direction_z;
    uint8_t compass_id;
    uint8_t cal_mask;
    uint8_t cal_status;
    uint8_t attempt;
    uint8_t completion_pct;
    uint8_t completion_mask[10];
}) fmav_mag_cal_progress_t;


#define FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS  191

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX  27
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA  92

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FLAGS  0
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FRAME_LEN_MAX  52

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_LEN  10 // length of array = number of bytes

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_DIRECTION_X_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_DIRECTION_Y_OFS  4
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_DIRECTION_Z_OFS  8
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPASS_ID_OFS  12
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_CAL_MASK_OFS  13
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_CAL_STATUS_OFS  14
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_ATTEMPT_OFS  15
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_PCT_OFS  16
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_OFS  17


//----------------------------------------
//-- Message MAG_CAL_PROGRESS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z,
    fmav_status_t* _status)
{
    fmav_mag_cal_progress_t* _payload = (fmav_mag_cal_progress_t*)_msg->payload;

    _payload->direction_x = direction_x;
    _payload->direction_y = direction_y;
    _payload->direction_z = direction_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->attempt = attempt;
    _payload->completion_pct = completion_pct;
    memcpy(&(_payload->completion_mask), completion_mask, sizeof(uint8_t)*10);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_progress_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_progress_pack(
        _msg, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->attempt, _payload->completion_pct, _payload->completion_mask, _payload->direction_x, _payload->direction_y, _payload->direction_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z,
    fmav_status_t* _status)
{
    fmav_mag_cal_progress_t* _payload = (fmav_mag_cal_progress_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->direction_x = direction_x;
    _payload->direction_y = direction_y;
    _payload->direction_z = direction_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->attempt = attempt;
    _payload->completion_pct = completion_pct;
    memcpy(&(_payload->completion_mask), completion_mask, sizeof(uint8_t)*10);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_progress_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_progress_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->attempt, _payload->completion_pct, _payload->completion_mask, _payload->direction_x, _payload->direction_y, _payload->direction_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z,
    fmav_status_t* _status)
{
    fmav_mag_cal_progress_t _payload;

    _payload.direction_x = direction_x;
    _payload.direction_y = direction_y;
    _payload.direction_z = direction_z;
    _payload.compass_id = compass_id;
    _payload.cal_mask = cal_mask;
    _payload.cal_status = cal_status;
    _payload.attempt = attempt;
    _payload.completion_pct = completion_pct;
    memcpy(&(_payload.completion_mask), completion_mask, sizeof(uint8_t)*10);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_progress_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MAG_CAL_PROGRESS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mag_cal_progress_decode(fmav_mag_cal_progress_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_progress_get_field_direction_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_progress_get_field_direction_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_mag_cal_progress_get_field_direction_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_compass_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_cal_mask(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_cal_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_attempt(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_completion_pct(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_mag_cal_progress_get_field_completion_mask_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[17]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mag_cal_progress_get_field_completion_mask(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_NUM) return 0;
    return ((uint8_t*)&(msg->payload[17]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS  191

#define mavlink_mag_cal_progress_t  fmav_mag_cal_progress_t

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_LEN  27
#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN  27
#define MAVLINK_MSG_ID_191_LEN  27
#define MAVLINK_MSG_ID_191_MIN_LEN  27

#define MAVLINK_MSG_ID_MAG_CAL_PROGRESS_CRC  92
#define MAVLINK_MSG_ID_191_CRC  92

#define MAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_LEN 10


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_progress_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mag_cal_progress_pack(
        _msg, sysid, compid,
        compass_id, cal_mask, cal_status, attempt, completion_pct, completion_mask, direction_x, direction_y, direction_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_progress_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mag_cal_progress_t* _payload)
{
    return mavlink_msg_mag_cal_progress_pack(
        sysid,
        compid,
        _msg,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->attempt, _payload->completion_pct, _payload->completion_mask, _payload->direction_x, _payload->direction_y, _payload->direction_z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_progress_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z)
{
    return fmav_msg_mag_cal_progress_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        compass_id, cal_mask, cal_status, attempt, completion_pct, completion_mask, direction_x, direction_y, direction_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mag_cal_progress_decode(const mavlink_message_t* msg, mavlink_mag_cal_progress_t* payload)
{
    fmav_msg_mag_cal_progress_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MAG_CAL_PROGRESS_H

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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MIN  27
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX  27
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN  27
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA  92

#define FASTMAVLINK_MSG_ID_191_LEN_MIN  27
#define FASTMAVLINK_MSG_ID_191_LEN_MAX  27
#define FASTMAVLINK_MSG_ID_191_LEN  27
#define FASTMAVLINK_MSG_ID_191_CRCEXTRA  92

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FIELD_COMPLETION_MASK_LEN  10

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FLAGS  0
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MAG_CAL_PROGRESS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_191_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_191_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message MAG_CAL_PROGRESS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z,
    fmav_status_t* _status)
{
    fmav_mag_cal_progress_t* _payload = (fmav_mag_cal_progress_t*)msg->payload;

    _payload->direction_x = direction_x;
    _payload->direction_y = direction_y;
    _payload->direction_z = direction_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->attempt = attempt;
    _payload->completion_pct = completion_pct;
    memcpy(&(_payload->completion_mask), completion_mask, sizeof(uint8_t)*10);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_progress_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_progress_pack(
        msg, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->attempt, _payload->completion_pct, _payload->completion_mask, _payload->direction_x, _payload->direction_y, _payload->direction_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z,
    fmav_status_t* _status)
{
    fmav_mag_cal_progress_t* _payload = (fmav_mag_cal_progress_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->direction_x = direction_x;
    _payload->direction_y = direction_y;
    _payload->direction_z = direction_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->attempt = attempt;
    _payload->completion_pct = completion_pct;
    memcpy(&(_payload->completion_mask), completion_mask, sizeof(uint8_t)*10);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_PROGRESS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_progress_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_progress_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_progress_pack_to_frame_buf(
        buf, sysid, compid,
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
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_PROGRESS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MAG_CAL_PROGRESS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mag_cal_progress_decode(fmav_mag_cal_progress_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MAG_CAL_PROGRESS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mag_cal_progress_pack(
        msg, sysid, compid,
        compass_id, cal_mask, cal_status, attempt, completion_pct, completion_mask, direction_x, direction_y, direction_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_progress_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t attempt, uint8_t completion_pct, const uint8_t* completion_mask, float direction_x, float direction_y, float direction_z)
{
    return fmav_msg_mag_cal_progress_pack_to_frame_buf(
        (uint8_t*)buf,
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

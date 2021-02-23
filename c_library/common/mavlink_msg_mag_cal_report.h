//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MAG_CAL_REPORT_H
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_H


//----------------------------------------
//-- Message MAG_CAL_REPORT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mag_cal_report_t {
    float fitness;
    float ofs_x;
    float ofs_y;
    float ofs_z;
    float diag_x;
    float diag_y;
    float diag_z;
    float offdiag_x;
    float offdiag_y;
    float offdiag_z;
    uint8_t compass_id;
    uint8_t cal_mask;
    uint8_t cal_status;
    uint8_t autosaved;
    float orientation_confidence;
    uint8_t old_orientation;
    uint8_t new_orientation;
    float scale_factor;
}) fmav_mag_cal_report_t;


#define FASTMAVLINK_MSG_ID_MAG_CAL_REPORT  192


#define FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MIN  44
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN  54
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA  36

#define FASTMAVLINK_MSG_ID_192_LEN_MIN  44
#define FASTMAVLINK_MSG_ID_192_LEN_MAX  54
#define FASTMAVLINK_MSG_ID_192_LEN  54
#define FASTMAVLINK_MSG_ID_192_CRCEXTRA  36



#define FASTMAVLINK_MSG_MAG_CAL_REPORT_FLAGS  0
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MAG_CAL_REPORT_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message MAG_CAL_REPORT packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor,
    fmav_status_t* _status)
{
    fmav_mag_cal_report_t* _payload = (fmav_mag_cal_report_t*)msg->payload;

    _payload->fitness = fitness;
    _payload->ofs_x = ofs_x;
    _payload->ofs_y = ofs_y;
    _payload->ofs_z = ofs_z;
    _payload->diag_x = diag_x;
    _payload->diag_y = diag_y;
    _payload->diag_z = diag_z;
    _payload->offdiag_x = offdiag_x;
    _payload->offdiag_y = offdiag_y;
    _payload->offdiag_z = offdiag_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->autosaved = autosaved;
    _payload->orientation_confidence = orientation_confidence;
    _payload->old_orientation = old_orientation;
    _payload->new_orientation = new_orientation;
    _payload->scale_factor = scale_factor;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MAG_CAL_REPORT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_report_pack(
        msg, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->autosaved, _payload->fitness, _payload->ofs_x, _payload->ofs_y, _payload->ofs_z, _payload->diag_x, _payload->diag_y, _payload->diag_z, _payload->offdiag_x, _payload->offdiag_y, _payload->offdiag_z, _payload->orientation_confidence, _payload->old_orientation, _payload->new_orientation, _payload->scale_factor,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor,
    fmav_status_t* _status)
{
    fmav_mag_cal_report_t* _payload = (fmav_mag_cal_report_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->fitness = fitness;
    _payload->ofs_x = ofs_x;
    _payload->ofs_y = ofs_y;
    _payload->ofs_z = ofs_z;
    _payload->diag_x = diag_x;
    _payload->diag_y = diag_y;
    _payload->diag_z = diag_z;
    _payload->offdiag_x = offdiag_x;
    _payload->offdiag_y = offdiag_y;
    _payload->offdiag_z = offdiag_z;
    _payload->compass_id = compass_id;
    _payload->cal_mask = cal_mask;
    _payload->cal_status = cal_status;
    _payload->autosaved = autosaved;
    _payload->orientation_confidence = orientation_confidence;
    _payload->old_orientation = old_orientation;
    _payload->new_orientation = new_orientation;
    _payload->scale_factor = scale_factor;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MAG_CAL_REPORT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MAG_CAL_REPORT_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mag_cal_report_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mag_cal_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mag_cal_report_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->compass_id, _payload->cal_mask, _payload->cal_status, _payload->autosaved, _payload->fitness, _payload->ofs_x, _payload->ofs_y, _payload->ofs_z, _payload->diag_x, _payload->diag_y, _payload->diag_z, _payload->offdiag_x, _payload->offdiag_y, _payload->offdiag_z, _payload->orientation_confidence, _payload->old_orientation, _payload->new_orientation, _payload->scale_factor,
        _status);
}


//----------------------------------------
//-- Message MAG_CAL_REPORT unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mag_cal_report_decode(fmav_mag_cal_report_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_MAG_CAL_REPORT_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MAG_CAL_REPORT  192

#define mavlink_mag_cal_report_t  fmav_mag_cal_report_t

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN  54
#define MAVLINK_MSG_ID_MAG_CAL_REPORT_MIN_LEN  44
#define MAVLINK_MSG_ID_192_LEN  54
#define MAVLINK_MSG_ID_192_MIN_LEN  44

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC  36
#define MAVLINK_MSG_ID_192_CRC  36




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mag_cal_report_pack(
        msg, sysid, compid,
        compass_id, cal_mask, cal_status, autosaved, fitness, ofs_x, ofs_y, ofs_z, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z, orientation_confidence, old_orientation, new_orientation, scale_factor,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mag_cal_report_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z, float orientation_confidence, uint8_t old_orientation, uint8_t new_orientation, float scale_factor)
{
    return fmav_msg_mag_cal_report_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        compass_id, cal_mask, cal_status, autosaved, fitness, ofs_x, ofs_y, ofs_z, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z, orientation_confidence, old_orientation, new_orientation, scale_factor,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mag_cal_report_decode(const mavlink_message_t* msg, mavlink_mag_cal_report_t* payload)
{
    fmav_msg_mag_cal_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MAG_CAL_REPORT_H

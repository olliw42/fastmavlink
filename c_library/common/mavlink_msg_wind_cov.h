//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_WIND_COV_H
#define FASTMAVLINK_MSG_WIND_COV_H


//----------------------------------------
//-- Message WIND_COV
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_wind_cov_t {
    uint64_t time_usec;
    float wind_x;
    float wind_y;
    float wind_z;
    float var_horiz;
    float var_vert;
    float wind_alt;
    float horiz_accuracy;
    float vert_accuracy;
}) fmav_wind_cov_t;


#define FASTMAVLINK_MSG_ID_WIND_COV  231


#define FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MIN  40
#define FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX  40
#define FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN  40
#define FASTMAVLINK_MSG_WIND_COV_CRCEXTRA  105

#define FASTMAVLINK_MSG_ID_231_LEN_MIN  40
#define FASTMAVLINK_MSG_ID_231_LEN_MAX  40
#define FASTMAVLINK_MSG_ID_231_LEN  40
#define FASTMAVLINK_MSG_ID_231_CRCEXTRA  105



#define FASTMAVLINK_MSG_WIND_COV_FLAGS  0
#define FASTMAVLINK_MSG_WIND_COV_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_WIND_COV_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message WIND_COV packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy,
    fmav_status_t* _status)
{
    fmav_wind_cov_t* _payload = (fmav_wind_cov_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_z = wind_z;
    _payload->var_horiz = var_horiz;
    _payload->var_vert = var_vert;
    _payload->wind_alt = wind_alt;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_WIND_COV;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_WIND_COV_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_cov_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->wind_x, _payload->wind_y, _payload->wind_z, _payload->var_horiz, _payload->var_vert, _payload->wind_alt, _payload->horiz_accuracy, _payload->vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy,
    fmav_status_t* _status)
{
    fmav_wind_cov_t* _payload = (fmav_wind_cov_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->wind_x = wind_x;
    _payload->wind_y = wind_y;
    _payload->wind_z = wind_z;
    _payload->var_horiz = var_horiz;
    _payload->var_vert = var_vert;
    _payload->wind_alt = wind_alt;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_WIND_COV;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND_COV >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_WIND_COV >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_WIND_COV_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_wind_cov_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_wind_cov_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_wind_cov_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->wind_x, _payload->wind_y, _payload->wind_z, _payload->var_horiz, _payload->var_vert, _payload->wind_alt, _payload->horiz_accuracy, _payload->vert_accuracy,
        _status);
}


//----------------------------------------
//-- Message WIND_COV unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_wind_cov_decode(fmav_wind_cov_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_WIND_COV_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_WIND_COV  231

#define mavlink_wind_cov_t  fmav_wind_cov_t

#define MAVLINK_MSG_ID_WIND_COV_LEN  40
#define MAVLINK_MSG_ID_WIND_COV_MIN_LEN  40
#define MAVLINK_MSG_ID_231_LEN  40
#define MAVLINK_MSG_ID_231_MIN_LEN  40

#define MAVLINK_MSG_ID_WIND_COV_CRC  105
#define MAVLINK_MSG_ID_231_CRC  105




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_cov_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_wind_cov_pack(
        msg, sysid, compid,
        time_usec, wind_x, wind_y, wind_z, var_horiz, var_vert, wind_alt, horiz_accuracy, vert_accuracy,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_wind_cov_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float wind_x, float wind_y, float wind_z, float var_horiz, float var_vert, float wind_alt, float horiz_accuracy, float vert_accuracy)
{
    return fmav_msg_wind_cov_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, wind_x, wind_y, wind_z, var_horiz, var_vert, wind_alt, horiz_accuracy, vert_accuracy,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_wind_cov_decode(const mavlink_message_t* msg, mavlink_wind_cov_t* payload)
{
    fmav_msg_wind_cov_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_WIND_COV_H

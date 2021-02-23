//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS2_RTK_H
#define FASTMAVLINK_MSG_GPS2_RTK_H


//----------------------------------------
//-- Message GPS2_RTK
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps2_rtk_t {
    uint32_t time_last_baseline_ms;
    uint32_t tow;
    int32_t baseline_a_mm;
    int32_t baseline_b_mm;
    int32_t baseline_c_mm;
    uint32_t accuracy;
    int32_t iar_num_hypotheses;
    uint16_t wn;
    uint8_t rtk_receiver_id;
    uint8_t rtk_health;
    uint8_t rtk_rate;
    uint8_t nsats;
    uint8_t baseline_coords_type;
}) fmav_gps2_rtk_t;


#define FASTMAVLINK_MSG_ID_GPS2_RTK  128


#define FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MIN  35
#define FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX  35
#define FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN  35
#define FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA  226

#define FASTMAVLINK_MSG_ID_128_LEN_MIN  35
#define FASTMAVLINK_MSG_ID_128_LEN_MAX  35
#define FASTMAVLINK_MSG_ID_128_LEN  35
#define FASTMAVLINK_MSG_ID_128_CRCEXTRA  226



#define FASTMAVLINK_MSG_GPS2_RTK_FLAGS  0
#define FASTMAVLINK_MSG_GPS2_RTK_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS2_RTK_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message GPS2_RTK packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses,
    fmav_status_t* _status)
{
    fmav_gps2_rtk_t* _payload = (fmav_gps2_rtk_t*)msg->payload;

    _payload->time_last_baseline_ms = time_last_baseline_ms;
    _payload->tow = tow;
    _payload->baseline_a_mm = baseline_a_mm;
    _payload->baseline_b_mm = baseline_b_mm;
    _payload->baseline_c_mm = baseline_c_mm;
    _payload->accuracy = accuracy;
    _payload->iar_num_hypotheses = iar_num_hypotheses;
    _payload->wn = wn;
    _payload->rtk_receiver_id = rtk_receiver_id;
    _payload->rtk_health = rtk_health;
    _payload->rtk_rate = rtk_rate;
    _payload->nsats = nsats;
    _payload->baseline_coords_type = baseline_coords_type;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS2_RTK;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_rtk_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_rtk_pack(
        msg, sysid, compid,
        _payload->time_last_baseline_ms, _payload->rtk_receiver_id, _payload->wn, _payload->tow, _payload->rtk_health, _payload->rtk_rate, _payload->nsats, _payload->baseline_coords_type, _payload->baseline_a_mm, _payload->baseline_b_mm, _payload->baseline_c_mm, _payload->accuracy, _payload->iar_num_hypotheses,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses,
    fmav_status_t* _status)
{
    fmav_gps2_rtk_t* _payload = (fmav_gps2_rtk_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_last_baseline_ms = time_last_baseline_ms;
    _payload->tow = tow;
    _payload->baseline_a_mm = baseline_a_mm;
    _payload->baseline_b_mm = baseline_b_mm;
    _payload->baseline_c_mm = baseline_c_mm;
    _payload->accuracy = accuracy;
    _payload->iar_num_hypotheses = iar_num_hypotheses;
    _payload->wn = wn;
    _payload->rtk_receiver_id = rtk_receiver_id;
    _payload->rtk_health = rtk_health;
    _payload->rtk_rate = rtk_rate;
    _payload->nsats = nsats;
    _payload->baseline_coords_type = baseline_coords_type;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS2_RTK;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RTK >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS2_RTK >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS2_RTK_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps2_rtk_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps2_rtk_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps2_rtk_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_last_baseline_ms, _payload->rtk_receiver_id, _payload->wn, _payload->tow, _payload->rtk_health, _payload->rtk_rate, _payload->nsats, _payload->baseline_coords_type, _payload->baseline_a_mm, _payload->baseline_b_mm, _payload->baseline_c_mm, _payload->accuracy, _payload->iar_num_hypotheses,
        _status);
}


//----------------------------------------
//-- Message GPS2_RTK unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps2_rtk_decode(fmav_gps2_rtk_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GPS2_RTK_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS2_RTK  128

#define mavlink_gps2_rtk_t  fmav_gps2_rtk_t

#define MAVLINK_MSG_ID_GPS2_RTK_LEN  35
#define MAVLINK_MSG_ID_GPS2_RTK_MIN_LEN  35
#define MAVLINK_MSG_ID_128_LEN  35
#define MAVLINK_MSG_ID_128_MIN_LEN  35

#define MAVLINK_MSG_ID_GPS2_RTK_CRC  226
#define MAVLINK_MSG_ID_128_CRC  226




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_rtk_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps2_rtk_pack(
        msg, sysid, compid,
        time_last_baseline_ms, rtk_receiver_id, wn, tow, rtk_health, rtk_rate, nsats, baseline_coords_type, baseline_a_mm, baseline_b_mm, baseline_c_mm, accuracy, iar_num_hypotheses,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps2_rtk_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
    return fmav_msg_gps2_rtk_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_last_baseline_ms, rtk_receiver_id, wn, tow, rtk_health, rtk_rate, nsats, baseline_coords_type, baseline_a_mm, baseline_b_mm, baseline_c_mm, accuracy, iar_num_hypotheses,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps2_rtk_decode(const mavlink_message_t* msg, mavlink_gps2_rtk_t* payload)
{
    fmav_msg_gps2_rtk_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS2_RTK_H

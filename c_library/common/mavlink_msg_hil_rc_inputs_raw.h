//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hil_rc_inputs_raw_t {
    uint64_t time_usec;
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint16_t chan9_raw;
    uint16_t chan10_raw;
    uint16_t chan11_raw;
    uint16_t chan12_raw;
    uint8_t rssi;
}) fmav_hil_rc_inputs_raw_t;


#define FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW  92


#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MIN  33
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN  33
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA  54

#define FASTMAVLINK_MSG_ID_92_LEN_MIN  33
#define FASTMAVLINK_MSG_ID_92_LEN_MAX  33
#define FASTMAVLINK_MSG_ID_92_LEN  33
#define FASTMAVLINK_MSG_ID_92_CRCEXTRA  54



#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_FLAGS  0
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_hil_rc_inputs_raw_t* _payload = (fmav_hil_rc_inputs_raw_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->rssi = rssi;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_rc_inputs_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_rc_inputs_raw_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi,
    fmav_status_t* _status)
{
    fmav_hil_rc_inputs_raw_t* _payload = (fmav_hil_rc_inputs_raw_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->rssi = rssi;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HIL_RC_INPUTS_RAW >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hil_rc_inputs_raw_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hil_rc_inputs_raw_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->rssi,
        _status);
}


//----------------------------------------
//-- Message HIL_RC_INPUTS_RAW unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hil_rc_inputs_raw_decode(fmav_hil_rc_inputs_raw_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW  92

#define mavlink_hil_rc_inputs_raw_t  fmav_hil_rc_inputs_raw_t

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN  33
#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_MIN_LEN  33
#define MAVLINK_MSG_ID_92_LEN  33
#define MAVLINK_MSG_ID_92_MIN_LEN  33

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC  54
#define MAVLINK_MSG_ID_92_CRC  54




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_rc_inputs_raw_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hil_rc_inputs_raw_pack(
        msg, sysid, compid,
        time_usec, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, rssi,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hil_rc_inputs_raw_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
    return fmav_msg_hil_rc_inputs_raw_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, rssi,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hil_rc_inputs_raw_decode(const mavlink_message_t* msg, mavlink_hil_rc_inputs_raw_t* payload)
{
    fmav_msg_hil_rc_inputs_raw_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HIL_RC_INPUTS_RAW_H

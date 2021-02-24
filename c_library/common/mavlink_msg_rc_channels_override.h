//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_H
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_H


//----------------------------------------
//-- Message RC_CHANNELS_OVERRIDE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rc_channels_override_t {
    uint16_t chan1_raw;
    uint16_t chan2_raw;
    uint16_t chan3_raw;
    uint16_t chan4_raw;
    uint16_t chan5_raw;
    uint16_t chan6_raw;
    uint16_t chan7_raw;
    uint16_t chan8_raw;
    uint8_t target_system;
    uint8_t target_component;
    uint16_t chan9_raw;
    uint16_t chan10_raw;
    uint16_t chan11_raw;
    uint16_t chan12_raw;
    uint16_t chan13_raw;
    uint16_t chan14_raw;
    uint16_t chan15_raw;
    uint16_t chan16_raw;
    uint16_t chan17_raw;
    uint16_t chan18_raw;
}) fmav_rc_channels_override_t;


#define FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE  70


#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MIN  18
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN  38
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_CRCEXTRA  124

#define FASTMAVLINK_MSG_ID_70_LEN_MIN  18
#define FASTMAVLINK_MSG_ID_70_LEN_MAX  38
#define FASTMAVLINK_MSG_ID_70_LEN  38
#define FASTMAVLINK_MSG_ID_70_CRCEXTRA  124



#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_FLAGS  3
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_TARGET_SYSTEM_OFS  16
#define FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_TARGET_COMPONENT_OFS  17


//----------------------------------------
//-- Message RC_CHANNELS_OVERRIDE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_override_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw,
    fmav_status_t* _status)
{
    fmav_rc_channels_override_t* _payload = (fmav_rc_channels_override_t*)msg->payload;

    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->chan13_raw = chan13_raw;
    _payload->chan14_raw = chan14_raw;
    _payload->chan15_raw = chan15_raw;
    _payload->chan16_raw = chan16_raw;
    _payload->chan17_raw = chan17_raw;
    _payload->chan18_raw = chan18_raw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_override_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_override_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_override_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->chan13_raw, _payload->chan14_raw, _payload->chan15_raw, _payload->chan16_raw, _payload->chan17_raw, _payload->chan18_raw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_override_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw,
    fmav_status_t* _status)
{
    fmav_rc_channels_override_t* _payload = (fmav_rc_channels_override_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->chan1_raw = chan1_raw;
    _payload->chan2_raw = chan2_raw;
    _payload->chan3_raw = chan3_raw;
    _payload->chan4_raw = chan4_raw;
    _payload->chan5_raw = chan5_raw;
    _payload->chan6_raw = chan6_raw;
    _payload->chan7_raw = chan7_raw;
    _payload->chan8_raw = chan8_raw;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->chan9_raw = chan9_raw;
    _payload->chan10_raw = chan10_raw;
    _payload->chan11_raw = chan11_raw;
    _payload->chan12_raw = chan12_raw;
    _payload->chan13_raw = chan13_raw;
    _payload->chan14_raw = chan14_raw;
    _payload->chan15_raw = chan15_raw;
    _payload->chan16_raw = chan16_raw;
    _payload->chan17_raw = chan17_raw;
    _payload->chan18_raw = chan18_raw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rc_channels_override_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rc_channels_override_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rc_channels_override_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->chan1_raw, _payload->chan2_raw, _payload->chan3_raw, _payload->chan4_raw, _payload->chan5_raw, _payload->chan6_raw, _payload->chan7_raw, _payload->chan8_raw, _payload->chan9_raw, _payload->chan10_raw, _payload->chan11_raw, _payload->chan12_raw, _payload->chan13_raw, _payload->chan14_raw, _payload->chan15_raw, _payload->chan16_raw, _payload->chan17_raw, _payload->chan18_raw,
        _status);
}


//----------------------------------------
//-- Message RC_CHANNELS_OVERRIDE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rc_channels_override_decode(fmav_rc_channels_override_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE  70

#define mavlink_rc_channels_override_t  fmav_rc_channels_override_t

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN  38
#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN  18
#define MAVLINK_MSG_ID_70_LEN  38
#define MAVLINK_MSG_ID_70_MIN_LEN  18

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC  124
#define MAVLINK_MSG_ID_70_CRC  124




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_override_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rc_channels_override_pack(
        msg, sysid, compid,
        target_system, target_component, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw, chan16_raw, chan17_raw, chan18_raw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rc_channels_override_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw)
{
    return fmav_msg_rc_channels_override_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw, chan16_raw, chan17_raw, chan18_raw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rc_channels_override_decode(const mavlink_message_t* msg, mavlink_rc_channels_override_t* payload)
{
    fmav_msg_rc_channels_override_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RC_CHANNELS_OVERRIDE_H

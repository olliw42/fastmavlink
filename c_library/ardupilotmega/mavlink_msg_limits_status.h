//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LIMITS_STATUS_H
#define FASTMAVLINK_MSG_LIMITS_STATUS_H


//----------------------------------------
//-- Message LIMITS_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_limits_status_t {
    uint32_t last_trigger;
    uint32_t last_action;
    uint32_t last_recovery;
    uint32_t last_clear;
    uint16_t breach_count;
    uint8_t limits_state;
    uint8_t mods_enabled;
    uint8_t mods_required;
    uint8_t mods_triggered;
}) fmav_limits_status_t;


#define FASTMAVLINK_MSG_ID_LIMITS_STATUS  167

#define FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX  22
#define FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA  144

#define FASTMAVLINK_MSG_LIMITS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_LIMITS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LIMITS_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LIMITS_STATUS_FRAME_LEN_MAX  47



#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_LAST_TRIGGER_OFS  0
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_LAST_ACTION_OFS  4
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_LAST_RECOVERY_OFS  8
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_LAST_CLEAR_OFS  12
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_BREACH_COUNT_OFS  16
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_LIMITS_STATE_OFS  18
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_MODS_ENABLED_OFS  19
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_MODS_REQUIRED_OFS  20
#define FASTMAVLINK_MSG_LIMITS_STATUS_FIELD_MODS_TRIGGERED_OFS  21


//----------------------------------------
//-- Message LIMITS_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered,
    fmav_status_t* _status)
{
    fmav_limits_status_t* _payload = (fmav_limits_status_t*)msg->payload;

    _payload->last_trigger = last_trigger;
    _payload->last_action = last_action;
    _payload->last_recovery = last_recovery;
    _payload->last_clear = last_clear;
    _payload->breach_count = breach_count;
    _payload->limits_state = limits_state;
    _payload->mods_enabled = mods_enabled;
    _payload->mods_required = mods_required;
    _payload->mods_triggered = mods_triggered;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_LIMITS_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_limits_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_limits_status_pack(
        msg, sysid, compid,
        _payload->limits_state, _payload->last_trigger, _payload->last_action, _payload->last_recovery, _payload->last_clear, _payload->breach_count, _payload->mods_enabled, _payload->mods_required, _payload->mods_triggered,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered,
    fmav_status_t* _status)
{
    fmav_limits_status_t* _payload = (fmav_limits_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->last_trigger = last_trigger;
    _payload->last_action = last_action;
    _payload->last_recovery = last_recovery;
    _payload->last_clear = last_clear;
    _payload->breach_count = breach_count;
    _payload->limits_state = limits_state;
    _payload->mods_enabled = mods_enabled;
    _payload->mods_required = mods_required;
    _payload->mods_triggered = mods_triggered;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LIMITS_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LIMITS_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LIMITS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_limits_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_limits_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->limits_state, _payload->last_trigger, _payload->last_action, _payload->last_recovery, _payload->last_clear, _payload->breach_count, _payload->mods_enabled, _payload->mods_required, _payload->mods_triggered,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered,
    fmav_status_t* _status)
{
    fmav_limits_status_t _payload;

    _payload.last_trigger = last_trigger;
    _payload.last_action = last_action;
    _payload.last_recovery = last_recovery;
    _payload.last_clear = last_clear;
    _payload.breach_count = breach_count;
    _payload.limits_state = limits_state;
    _payload.mods_enabled = mods_enabled;
    _payload.mods_required = mods_required;
    _payload.mods_triggered = mods_triggered;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LIMITS_STATUS,
        FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_limits_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LIMITS_STATUS,
        FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LIMITS_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LIMITS_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_limits_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_limits_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_limits_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_limits_status_decode(fmav_limits_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LIMITS_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_limits_status_get_field_last_trigger(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_limits_status_get_field_last_action(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_limits_status_get_field_last_recovery(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_limits_status_get_field_last_clear(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_limits_status_get_field_breach_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_limits_status_get_field_limits_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_limits_status_get_field_mods_enabled(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_limits_status_get_field_mods_required(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_limits_status_get_field_mods_triggered(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[21]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LIMITS_STATUS  167

#define mavlink_limits_status_t  fmav_limits_status_t

#define MAVLINK_MSG_ID_LIMITS_STATUS_LEN  22
#define MAVLINK_MSG_ID_LIMITS_STATUS_MIN_LEN  22
#define MAVLINK_MSG_ID_167_LEN  22
#define MAVLINK_MSG_ID_167_MIN_LEN  22

#define MAVLINK_MSG_ID_LIMITS_STATUS_CRC  144
#define MAVLINK_MSG_ID_167_CRC  144




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_limits_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_limits_status_pack(
        msg, sysid, compid,
        limits_state, last_trigger, last_action, last_recovery, last_clear, breach_count, mods_enabled, mods_required, mods_triggered,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_limits_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t limits_state, uint32_t last_trigger, uint32_t last_action, uint32_t last_recovery, uint32_t last_clear, uint16_t breach_count, uint8_t mods_enabled, uint8_t mods_required, uint8_t mods_triggered)
{
    return fmav_msg_limits_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        limits_state, last_trigger, last_action, last_recovery, last_clear, breach_count, mods_enabled, mods_required, mods_triggered,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_limits_status_decode(const mavlink_message_t* msg, mavlink_limits_status_t* payload)
{
    fmav_msg_limits_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LIMITS_STATUS_H

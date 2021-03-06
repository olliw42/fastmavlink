//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BUTTON_CHANGE_H
#define FASTMAVLINK_MSG_BUTTON_CHANGE_H


//----------------------------------------
//-- Message BUTTON_CHANGE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_button_change_t {
    uint32_t time_boot_ms;
    uint32_t last_change_ms;
    uint8_t state;
}) fmav_button_change_t;


#define FASTMAVLINK_MSG_ID_BUTTON_CHANGE  257

#define FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX  9
#define FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA  131

#define FASTMAVLINK_MSG_BUTTON_CHANGE_FLAGS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BUTTON_CHANGE_FRAME_LEN_MAX  34



#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_LAST_CHANGE_MS_OFS  4
#define FASTMAVLINK_MSG_BUTTON_CHANGE_FIELD_STATE_OFS  8


//----------------------------------------
//-- Message BUTTON_CHANGE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t* _payload = (fmav_button_change_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->last_change_ms = last_change_ms;
    _payload->state = state;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_BUTTON_CHANGE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_button_change_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->last_change_ms, _payload->state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t* _payload = (fmav_button_change_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->last_change_ms = last_change_ms;
    _payload->state = state;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BUTTON_CHANGE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_button_change_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->last_change_ms, _payload->state,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state,
    fmav_status_t* _status)
{
    fmav_button_change_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.last_change_ms = last_change_ms;
    _payload.state = state;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BUTTON_CHANGE,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_button_change_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_button_change_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BUTTON_CHANGE,
        FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BUTTON_CHANGE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BUTTON_CHANGE unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_button_change_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_button_change_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_button_change_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_button_change_decode(fmav_button_change_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_BUTTON_CHANGE_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_button_change_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_button_change_get_field_last_change_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_button_change_get_field_state(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BUTTON_CHANGE  257

#define mavlink_button_change_t  fmav_button_change_t

#define MAVLINK_MSG_ID_BUTTON_CHANGE_LEN  9
#define MAVLINK_MSG_ID_BUTTON_CHANGE_MIN_LEN  9
#define MAVLINK_MSG_ID_257_LEN  9
#define MAVLINK_MSG_ID_257_MIN_LEN  9

#define MAVLINK_MSG_ID_BUTTON_CHANGE_CRC  131
#define MAVLINK_MSG_ID_257_CRC  131




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_button_change_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_button_change_pack(
        msg, sysid, compid,
        time_boot_ms, last_change_ms, state,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_button_change_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t last_change_ms, uint8_t state)
{
    return fmav_msg_button_change_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, last_change_ms, state,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_button_change_decode(const mavlink_message_t* msg, mavlink_button_change_t* payload)
{
    fmav_msg_button_change_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BUTTON_CHANGE_H

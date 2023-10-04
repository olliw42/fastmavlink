//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ASLCTRL_DEBUG_H
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_H


//----------------------------------------
//-- Message ASLCTRL_DEBUG
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_aslctrl_debug_t {
    uint32_t i32_1;
    float f_1;
    float f_2;
    float f_3;
    float f_4;
    float f_5;
    float f_6;
    float f_7;
    float f_8;
    uint8_t i8_1;
    uint8_t i8_2;
}) fmav_aslctrl_debug_t;


#define FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG  8005

#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX  38
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_CRCEXTRA  251

#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FLAGS  0
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FRAME_LEN_MAX  63



#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_I32_1_OFS  0
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_1_OFS  4
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_2_OFS  8
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_3_OFS  12
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_4_OFS  16
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_5_OFS  20
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_6_OFS  24
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_7_OFS  28
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_F_8_OFS  32
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_I8_1_OFS  36
#define FASTMAVLINK_MSG_ASLCTRL_DEBUG_FIELD_I8_2_OFS  37


//----------------------------------------
//-- Message ASLCTRL_DEBUG pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8,
    fmav_status_t* _status)
{
    fmav_aslctrl_debug_t* _payload = (fmav_aslctrl_debug_t*)_msg->payload;

    _payload->i32_1 = i32_1;
    _payload->f_1 = f_1;
    _payload->f_2 = f_2;
    _payload->f_3 = f_3;
    _payload->f_4 = f_4;
    _payload->f_5 = f_5;
    _payload->f_6 = f_6;
    _payload->f_7 = f_7;
    _payload->f_8 = f_8;
    _payload->i8_1 = i8_1;
    _payload->i8_2 = i8_2;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ASLCTRL_DEBUG_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_debug_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aslctrl_debug_pack(
        _msg, sysid, compid,
        _payload->i32_1, _payload->i8_1, _payload->i8_2, _payload->f_1, _payload->f_2, _payload->f_3, _payload->f_4, _payload->f_5, _payload->f_6, _payload->f_7, _payload->f_8,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8,
    fmav_status_t* _status)
{
    fmav_aslctrl_debug_t* _payload = (fmav_aslctrl_debug_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->i32_1 = i32_1;
    _payload->f_1 = f_1;
    _payload->f_2 = f_2;
    _payload->f_3 = f_3;
    _payload->f_4 = f_4;
    _payload->f_5 = f_5;
    _payload->f_6 = f_6;
    _payload->f_7 = f_7;
    _payload->f_8 = f_8;
    _payload->i8_1 = i8_1;
    _payload->i8_2 = i8_2;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_debug_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aslctrl_debug_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->i32_1, _payload->i8_1, _payload->i8_2, _payload->f_1, _payload->f_2, _payload->f_3, _payload->f_4, _payload->f_5, _payload->f_6, _payload->f_7, _payload->f_8,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8,
    fmav_status_t* _status)
{
    fmav_aslctrl_debug_t _payload;

    _payload.i32_1 = i32_1;
    _payload.f_1 = f_1;
    _payload.f_2 = f_2;
    _payload.f_3 = f_3;
    _payload.f_4 = f_4;
    _payload.f_5 = f_5;
    _payload.f_6 = f_6;
    _payload.f_7 = f_7;
    _payload.f_8 = f_8;
    _payload.i8_1 = i8_1;
    _payload.i8_2 = i8_2;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_debug_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_debug_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ASLCTRL_DEBUG,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DEBUG_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ASLCTRL_DEBUG decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_aslctrl_debug_decode(fmav_aslctrl_debug_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLCTRL_DEBUG_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_aslctrl_debug_get_field_i32_1(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_5(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_6(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_7(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_debug_get_field_f_8(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_aslctrl_debug_get_field_i8_1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_aslctrl_debug_get_field_i8_2(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[37]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG  8005

#define mavlink_aslctrl_debug_t  fmav_aslctrl_debug_t

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_LEN  38
#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_MIN_LEN  38
#define MAVLINK_MSG_ID_8005_LEN  38
#define MAVLINK_MSG_ID_8005_MIN_LEN  38

#define MAVLINK_MSG_ID_ASLCTRL_DEBUG_CRC  251
#define MAVLINK_MSG_ID_8005_CRC  251




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_debug_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_aslctrl_debug_pack(
        _msg, sysid, compid,
        i32_1, i8_1, i8_2, f_1, f_2, f_3, f_4, f_5, f_6, f_7, f_8,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_debug_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_aslctrl_debug_t* _payload)
{
    return mavlink_msg_aslctrl_debug_pack(
        sysid,
        compid,
        _msg,
        _payload->i32_1, _payload->i8_1, _payload->i8_2, _payload->f_1, _payload->f_2, _payload->f_3, _payload->f_4, _payload->f_5, _payload->f_6, _payload->f_7, _payload->f_8);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_debug_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t i32_1, uint8_t i8_1, uint8_t i8_2, float f_1, float f_2, float f_3, float f_4, float f_5, float f_6, float f_7, float f_8)
{
    return fmav_msg_aslctrl_debug_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        i32_1, i8_1, i8_2, f_1, f_2, f_3, f_4, f_5, f_6, f_7, f_8,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_aslctrl_debug_decode(const mavlink_message_t* msg, mavlink_aslctrl_debug_t* payload)
{
    fmav_msg_aslctrl_debug_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ASLCTRL_DEBUG_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_8_H
#define FASTMAVLINK_MSG_ARRAY_TEST_8_H


//----------------------------------------
//-- Message ARRAY_TEST_8
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_8_t {
    double ar_d[2];
    uint32_t v3;
    uint16_t ar_u16[2];
}) fmav_array_test_8_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_8  17158

#define FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA  106

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_8_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_8_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FRAME_LEN_MAX  49

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_LEN  4 // length of array = number of bytes

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_V3_OFS  16
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_OFS  20


//----------------------------------------
//-- Message ARRAY_TEST_8 pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16,
    fmav_status_t* _status)
{
    fmav_array_test_8_t* _payload = (fmav_array_test_8_t*)_msg->payload;

    _payload->v3 = v3;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_8;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_8_pack(
        _msg, sysid, compid,
        _payload->v3, _payload->ar_d, _payload->ar_u16,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16,
    fmav_status_t* _status)
{
    fmav_array_test_8_t* _payload = (fmav_array_test_8_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v3 = v3;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8 >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8 >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_8_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->v3, _payload->ar_d, _payload->ar_u16,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16,
    fmav_status_t* _status)
{
    fmav_array_test_8_t _payload;

    _payload.v3 = v3;
    memcpy(&(_payload.ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload.ar_u16), ar_u16, sizeof(uint16_t)*2);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_8,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_8,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ARRAY_TEST_8 decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_8_decode(fmav_array_test_8_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_array_test_8_get_field_v3(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR double* fmav_msg_array_test_8_get_field_ar_d_ptr(const fmav_message_t* msg)
{
    return (double*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR double fmav_msg_array_test_8_get_field_ar_d(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_NUM) return 0;
    return ((double*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_array_test_8_get_field_ar_u16_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_get_field_ar_u16(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_NUM) return 0;
    return ((uint16_t*)&(msg->payload[20]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_8  17158

#define mavlink_array_test_8_t  fmav_array_test_8_t

#define MAVLINK_MSG_ID_ARRAY_TEST_8_LEN  24
#define MAVLINK_MSG_ID_ARRAY_TEST_8_MIN_LEN  24
#define MAVLINK_MSG_ID_17158_LEN  24
#define MAVLINK_MSG_ID_17158_MIN_LEN  24

#define MAVLINK_MSG_ID_ARRAY_TEST_8_CRC  106
#define MAVLINK_MSG_ID_17158_CRC  106

#define MAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_LEN 2


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_8_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_8_pack(
        _msg, sysid, compid,
        v3, ar_d, ar_u16,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_8_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_array_test_8_t* _payload)
{
    return mavlink_msg_array_test_8_pack(
        sysid,
        compid,
        _msg,
        _payload->v3, _payload->ar_d, _payload->ar_u16);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_8_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16)
{
    return fmav_msg_array_test_8_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        v3, ar_d, ar_u16,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_8_decode(const mavlink_message_t* msg, mavlink_array_test_8_t* payload)
{
    fmav_msg_array_test_8_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_8_H

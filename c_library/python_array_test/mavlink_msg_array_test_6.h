//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_6_H
#define FASTMAVLINK_MSG_ARRAY_TEST_6_H


//----------------------------------------
//-- Message ARRAY_TEST_6
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_6_t {
    double ar_d[2];
    uint32_t v3;
    uint32_t ar_u32[2];
    int32_t ar_i32[2];
    float ar_f[2];
    uint16_t v2;
    uint16_t ar_u16[2];
    int16_t ar_i16[2];
    uint8_t v1;
    uint8_t ar_u8[2];
    int8_t ar_i8[2];
    char ar_c[32];
}) fmav_array_test_6_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_6  17156

#define FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX  91
#define FASTMAVLINK_MSG_ARRAY_TEST_6_CRCEXTRA  14

#define FASTMAVLINK_MSG_ARRAY_TEST_6_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_6_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_6_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ARRAY_TEST_6_FRAME_LEN_MAX  116

#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_D_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_D_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U32_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U32_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I32_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I32_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_F_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_F_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U16_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U16_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I16_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I16_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U8_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U8_LEN  2 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I8_NUM  2 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I8_LEN  2 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_C_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_C_LEN  32 // length of array = number of bytes

#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_D_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_V3_OFS  16
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U32_OFS  20
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I32_OFS  28
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_F_OFS  36
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_V2_OFS  44
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U16_OFS  46
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I16_OFS  50
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_V1_OFS  54
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U8_OFS  55
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I8_OFS  57
#define FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_C_OFS  59


//----------------------------------------
//-- Message ARRAY_TEST_6 pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, uint16_t v2, uint32_t v3, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c, const double* ar_d, const float* ar_f,
    fmav_status_t* _status)
{
    fmav_array_test_6_t* _payload = (fmav_array_test_6_t*)_msg->payload;

    _payload->v3 = v3;
    _payload->v2 = v2;
    _payload->v1 = v1;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*2);
    memcpy(&(_payload->ar_i32), ar_i32, sizeof(int32_t)*2);
    memcpy(&(_payload->ar_f), ar_f, sizeof(float)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);
    memcpy(&(_payload->ar_i16), ar_i16, sizeof(int16_t)*2);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*2);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*2);
    memcpy(&(_payload->ar_c), ar_c, sizeof(char)*32);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_6;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_6_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_6_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_6_pack(
        _msg, sysid, compid,
        _payload->v1, _payload->v2, _payload->v3, _payload->ar_u32, _payload->ar_i32, _payload->ar_u16, _payload->ar_i16, _payload->ar_u8, _payload->ar_i8, _payload->ar_c, _payload->ar_d, _payload->ar_f,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, uint16_t v2, uint32_t v3, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c, const double* ar_d, const float* ar_f,
    fmav_status_t* _status)
{
    fmav_array_test_6_t* _payload = (fmav_array_test_6_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v3 = v3;
    _payload->v2 = v2;
    _payload->v1 = v1;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*2);
    memcpy(&(_payload->ar_i32), ar_i32, sizeof(int32_t)*2);
    memcpy(&(_payload->ar_f), ar_f, sizeof(float)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);
    memcpy(&(_payload->ar_i16), ar_i16, sizeof(int16_t)*2);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*2);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*2);
    memcpy(&(_payload->ar_c), ar_c, sizeof(char)*32);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_6;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_6 >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_6 >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_6_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_6_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_6_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->v1, _payload->v2, _payload->v3, _payload->ar_u32, _payload->ar_i32, _payload->ar_u16, _payload->ar_i16, _payload->ar_u8, _payload->ar_i8, _payload->ar_c, _payload->ar_d, _payload->ar_f,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, uint16_t v2, uint32_t v3, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c, const double* ar_d, const float* ar_f,
    fmav_status_t* _status)
{
    fmav_array_test_6_t _payload;

    _payload.v3 = v3;
    _payload.v2 = v2;
    _payload.v1 = v1;
    memcpy(&(_payload.ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload.ar_u32), ar_u32, sizeof(uint32_t)*2);
    memcpy(&(_payload.ar_i32), ar_i32, sizeof(int32_t)*2);
    memcpy(&(_payload.ar_f), ar_f, sizeof(float)*2);
    memcpy(&(_payload.ar_u16), ar_u16, sizeof(uint16_t)*2);
    memcpy(&(_payload.ar_i16), ar_i16, sizeof(int16_t)*2);
    memcpy(&(_payload.ar_u8), ar_u8, sizeof(uint8_t)*2);
    memcpy(&(_payload.ar_i8), ar_i8, sizeof(int8_t)*2);
    memcpy(&(_payload.ar_c), ar_c, sizeof(char)*32);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_6,
        FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_6_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_6_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_6,
        FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_6_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ARRAY_TEST_6 decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_6_decode(fmav_array_test_6_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ARRAY_TEST_6_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_array_test_6_get_field_v3(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_get_field_v2(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_array_test_6_get_field_v1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[54]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR double* fmav_msg_array_test_6_get_field_ar_d_ptr(const fmav_message_t* msg)
{
    return (double*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR double fmav_msg_array_test_6_get_field_ar_d(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_D_NUM) return 0;
    return ((double*)&(msg->payload[0]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_array_test_6_get_field_ar_u32_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_array_test_6_get_field_ar_u32(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U32_NUM) return 0;
    return ((uint32_t*)&(msg->payload[20]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t* fmav_msg_array_test_6_get_field_ar_i32_ptr(const fmav_message_t* msg)
{
    return (int32_t*)&(msg->payload[28]);
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_array_test_6_get_field_ar_i32(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I32_NUM) return 0;
    return ((int32_t*)&(msg->payload[28]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_array_test_6_get_field_ar_f_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_array_test_6_get_field_ar_f(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_F_NUM) return 0;
    return ((float*)&(msg->payload[36]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_array_test_6_get_field_ar_u16_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[46]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_6_get_field_ar_u16(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U16_NUM) return 0;
    return ((uint16_t*)&(msg->payload[46]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_array_test_6_get_field_ar_i16_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[50]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_array_test_6_get_field_ar_i16(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I16_NUM) return 0;
    return ((int16_t*)&(msg->payload[50]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_array_test_6_get_field_ar_u8_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[55]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_array_test_6_get_field_ar_u8(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U8_NUM) return 0;
    return ((uint8_t*)&(msg->payload[55]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_array_test_6_get_field_ar_i8_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[57]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_array_test_6_get_field_ar_i8(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I8_NUM) return 0;
    return ((int8_t*)&(msg->payload[57]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_array_test_6_get_field_ar_c_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[59]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_array_test_6_get_field_ar_c(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_C_NUM) return 0;
    return ((char*)&(msg->payload[59]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_6  17156

#define mavlink_array_test_6_t  fmav_array_test_6_t

#define MAVLINK_MSG_ID_ARRAY_TEST_6_LEN  91
#define MAVLINK_MSG_ID_ARRAY_TEST_6_MIN_LEN  91
#define MAVLINK_MSG_ID_17156_LEN  91
#define MAVLINK_MSG_ID_17156_MIN_LEN  91

#define MAVLINK_MSG_ID_ARRAY_TEST_6_CRC  14
#define MAVLINK_MSG_ID_17156_CRC  14

#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_D_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_F_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_U8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_I8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_6_FIELD_AR_C_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_6_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t v1, uint16_t v2, uint32_t v3, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c, const double* ar_d, const float* ar_f)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_6_pack(
        _msg, sysid, compid,
        v1, v2, v3, ar_u32, ar_i32, ar_u16, ar_i16, ar_u8, ar_i8, ar_c, ar_d, ar_f,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_6_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_array_test_6_t* _payload)
{
    return mavlink_msg_array_test_6_pack(
        sysid,
        compid,
        _msg,
        _payload->v1, _payload->v2, _payload->v3, _payload->ar_u32, _payload->ar_i32, _payload->ar_u16, _payload->ar_i16, _payload->ar_u8, _payload->ar_i8, _payload->ar_c, _payload->ar_d, _payload->ar_f);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_6_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, uint16_t v2, uint32_t v3, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c, const double* ar_d, const float* ar_f)
{
    return fmav_msg_array_test_6_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        v1, v2, v3, ar_u32, ar_i32, ar_u16, ar_i16, ar_u8, ar_i8, ar_c, ar_d, ar_f,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_6_decode(const mavlink_message_t* msg, mavlink_array_test_6_t* payload)
{
    fmav_msg_array_test_6_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_6_H

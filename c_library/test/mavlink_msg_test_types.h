//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_TEST_TYPES_H
#define FASTMAVLINK_MSG_TEST_TYPES_H


//----------------------------------------
//-- Message TEST_TYPES
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_test_types_t {
    uint64_t u64;
    int64_t s64;
    double d;
    uint64_t u64_array[3];
    int64_t s64_array[3];
    double d_array[3];
    uint32_t u32;
    int32_t s32;
    float f;
    uint32_t u32_array[3];
    int32_t s32_array[3];
    float f_array[3];
    uint16_t u16;
    int16_t s16;
    uint16_t u16_array[3];
    int16_t s16_array[3];
    char c;
    char s[10];
    uint8_t u8;
    int8_t s8;
    uint8_t u8_array[3];
    int8_t s8_array[3];
}) fmav_test_types_t;


#define FASTMAVLINK_MSG_ID_TEST_TYPES  17000

#define FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX  179
#define FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA  103

#define FASTMAVLINK_MSG_TEST_TYPES_FLAGS  0
#define FASTMAVLINK_MSG_TEST_TYPES_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TEST_TYPES_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TEST_TYPES_FRAME_LEN_MAX  204

#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_LEN  12 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S_LEN  10 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_LEN  3 // length of array = number of bytes
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_NUM  3 // number of elements in array
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_LEN  3 // length of array = number of bytes

#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_OFS  0
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_OFS  8
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_OFS  16
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_OFS  24
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_OFS  48
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_OFS  72
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_OFS  96
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_OFS  100
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_OFS  104
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_OFS  108
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_OFS  120
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_OFS  132
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_OFS  144
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_OFS  146
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_OFS  148
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_OFS  154
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_C_OFS  160
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S_OFS  161
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_OFS  171
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_OFS  172
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_OFS  173
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_OFS  176


//----------------------------------------
//-- Message TEST_TYPES pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array,
    fmav_status_t* _status)
{
    fmav_test_types_t* _payload = (fmav_test_types_t*)_msg->payload;

    _payload->u64 = u64;
    _payload->s64 = s64;
    _payload->d = d;
    _payload->u32 = u32;
    _payload->s32 = s32;
    _payload->f = f;
    _payload->u16 = u16;
    _payload->s16 = s16;
    _payload->c = c;
    _payload->u8 = u8;
    _payload->s8 = s8;
    memcpy(&(_payload->u64_array), u64_array, sizeof(uint64_t)*3);
    memcpy(&(_payload->s64_array), s64_array, sizeof(int64_t)*3);
    memcpy(&(_payload->d_array), d_array, sizeof(double)*3);
    memcpy(&(_payload->u32_array), u32_array, sizeof(uint32_t)*3);
    memcpy(&(_payload->s32_array), s32_array, sizeof(int32_t)*3);
    memcpy(&(_payload->f_array), f_array, sizeof(float)*3);
    memcpy(&(_payload->u16_array), u16_array, sizeof(uint16_t)*3);
    memcpy(&(_payload->s16_array), s16_array, sizeof(int16_t)*3);
    memcpy(&(_payload->s), s, sizeof(char)*10);
    memcpy(&(_payload->u8_array), u8_array, sizeof(uint8_t)*3);
    memcpy(&(_payload->s8_array), s8_array, sizeof(int8_t)*3);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_TEST_TYPES;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_test_types_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_test_types_pack(
        _msg, sysid, compid,
        _payload->c, _payload->s, _payload->u8, _payload->u16, _payload->u32, _payload->u64, _payload->s8, _payload->s16, _payload->s32, _payload->s64, _payload->f, _payload->d, _payload->u8_array, _payload->u16_array, _payload->u32_array, _payload->u64_array, _payload->s8_array, _payload->s16_array, _payload->s32_array, _payload->s64_array, _payload->f_array, _payload->d_array,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array,
    fmav_status_t* _status)
{
    fmav_test_types_t* _payload = (fmav_test_types_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->u64 = u64;
    _payload->s64 = s64;
    _payload->d = d;
    _payload->u32 = u32;
    _payload->s32 = s32;
    _payload->f = f;
    _payload->u16 = u16;
    _payload->s16 = s16;
    _payload->c = c;
    _payload->u8 = u8;
    _payload->s8 = s8;
    memcpy(&(_payload->u64_array), u64_array, sizeof(uint64_t)*3);
    memcpy(&(_payload->s64_array), s64_array, sizeof(int64_t)*3);
    memcpy(&(_payload->d_array), d_array, sizeof(double)*3);
    memcpy(&(_payload->u32_array), u32_array, sizeof(uint32_t)*3);
    memcpy(&(_payload->s32_array), s32_array, sizeof(int32_t)*3);
    memcpy(&(_payload->f_array), f_array, sizeof(float)*3);
    memcpy(&(_payload->u16_array), u16_array, sizeof(uint16_t)*3);
    memcpy(&(_payload->s16_array), s16_array, sizeof(int16_t)*3);
    memcpy(&(_payload->s), s, sizeof(char)*10);
    memcpy(&(_payload->u8_array), u8_array, sizeof(uint8_t)*3);
    memcpy(&(_payload->s8_array), s8_array, sizeof(int8_t)*3);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TEST_TYPES;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TEST_TYPES >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TEST_TYPES >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_test_types_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_test_types_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->c, _payload->s, _payload->u8, _payload->u16, _payload->u32, _payload->u64, _payload->s8, _payload->s16, _payload->s32, _payload->s64, _payload->f, _payload->d, _payload->u8_array, _payload->u16_array, _payload->u32_array, _payload->u64_array, _payload->s8_array, _payload->s16_array, _payload->s32_array, _payload->s64_array, _payload->f_array, _payload->d_array,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array,
    fmav_status_t* _status)
{
    fmav_test_types_t _payload;

    _payload.u64 = u64;
    _payload.s64 = s64;
    _payload.d = d;
    _payload.u32 = u32;
    _payload.s32 = s32;
    _payload.f = f;
    _payload.u16 = u16;
    _payload.s16 = s16;
    _payload.c = c;
    _payload.u8 = u8;
    _payload.s8 = s8;
    memcpy(&(_payload.u64_array), u64_array, sizeof(uint64_t)*3);
    memcpy(&(_payload.s64_array), s64_array, sizeof(int64_t)*3);
    memcpy(&(_payload.d_array), d_array, sizeof(double)*3);
    memcpy(&(_payload.u32_array), u32_array, sizeof(uint32_t)*3);
    memcpy(&(_payload.s32_array), s32_array, sizeof(int32_t)*3);
    memcpy(&(_payload.f_array), f_array, sizeof(float)*3);
    memcpy(&(_payload.u16_array), u16_array, sizeof(uint16_t)*3);
    memcpy(&(_payload.s16_array), s16_array, sizeof(int16_t)*3);
    memcpy(&(_payload.s), s, sizeof(char)*10);
    memcpy(&(_payload.u8_array), u8_array, sizeof(uint8_t)*3);
    memcpy(&(_payload.s8_array), s8_array, sizeof(int8_t)*3);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_TEST_TYPES,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_test_types_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_TEST_TYPES,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TEST_TYPES decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_test_types_decode(fmav_test_types_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_test_types_get_field_u64(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int64_t fmav_msg_test_types_get_field_s64(const fmav_message_t* msg)
{
    int64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR double fmav_msg_test_types_get_field_d(const fmav_message_t* msg)
{
    double r;
    memcpy(&r, &(msg->payload[16]), sizeof(double));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_test_types_get_field_u32(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[96]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_test_types_get_field_s32(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[100]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_test_types_get_field_f(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[104]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_get_field_u16(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[144]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_test_types_get_field_s16(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[146]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_test_types_get_field_c(const fmav_message_t* msg)
{
    char r;
    memcpy(&r, &(msg->payload[160]), sizeof(char));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_test_types_get_field_u8(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[171]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_test_types_get_field_s8(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[172]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t* fmav_msg_test_types_get_field_u64_array_ptr(const fmav_message_t* msg)
{
    return (uint64_t*)&(msg->payload[24]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_test_types_get_field_u64_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_NUM) return 0;
    return ((uint64_t*)&(msg->payload[24]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int64_t* fmav_msg_test_types_get_field_s64_array_ptr(const fmav_message_t* msg)
{
    return (int64_t*)&(msg->payload[48]);
}


FASTMAVLINK_FUNCTION_DECORATOR int64_t fmav_msg_test_types_get_field_s64_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_NUM) return 0;
    return ((int64_t*)&(msg->payload[48]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR double* fmav_msg_test_types_get_field_d_array_ptr(const fmav_message_t* msg)
{
    return (double*)&(msg->payload[72]);
}


FASTMAVLINK_FUNCTION_DECORATOR double fmav_msg_test_types_get_field_d_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_NUM) return 0;
    return ((double*)&(msg->payload[72]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_test_types_get_field_u32_array_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[108]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_test_types_get_field_u32_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_NUM) return 0;
    return ((uint32_t*)&(msg->payload[108]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t* fmav_msg_test_types_get_field_s32_array_ptr(const fmav_message_t* msg)
{
    return (int32_t*)&(msg->payload[120]);
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_test_types_get_field_s32_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_NUM) return 0;
    return ((int32_t*)&(msg->payload[120]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR float* fmav_msg_test_types_get_field_f_array_ptr(const fmav_message_t* msg)
{
    return (float*)&(msg->payload[132]);
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_test_types_get_field_f_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_NUM) return 0;
    return ((float*)&(msg->payload[132]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_test_types_get_field_u16_array_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[148]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_get_field_u16_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_NUM) return 0;
    return ((uint16_t*)&(msg->payload[148]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_test_types_get_field_s16_array_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[154]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_test_types_get_field_s16_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_NUM) return 0;
    return ((int16_t*)&(msg->payload[154]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_test_types_get_field_s_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[161]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_test_types_get_field_s(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_S_NUM) return 0;
    return ((char*)&(msg->payload[161]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_test_types_get_field_u8_array_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[173]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_test_types_get_field_u8_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_NUM) return 0;
    return ((uint8_t*)&(msg->payload[173]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_test_types_get_field_s8_array_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[176]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_test_types_get_field_s8_array(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_NUM) return 0;
    return ((int8_t*)&(msg->payload[176]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_TEST_TYPES  17000

#define mavlink_test_types_t  fmav_test_types_t

#define MAVLINK_MSG_ID_TEST_TYPES_LEN  179
#define MAVLINK_MSG_ID_TEST_TYPES_MIN_LEN  179
#define MAVLINK_MSG_ID_17000_LEN  179
#define MAVLINK_MSG_ID_17000_MIN_LEN  179

#define MAVLINK_MSG_ID_TEST_TYPES_CRC  103
#define MAVLINK_MSG_ID_17000_CRC  103

#define MAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_S_LEN 10
#define MAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_LEN 3
#define MAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_LEN 3


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_test_types_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_test_types_pack(
        _msg, sysid, compid,
        c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_test_types_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_test_types_t* _payload)
{
    return mavlink_msg_test_types_pack(
        sysid,
        compid,
        _msg,
        _payload->c, _payload->s, _payload->u8, _payload->u16, _payload->u32, _payload->u64, _payload->s8, _payload->s16, _payload->s32, _payload->s64, _payload->f, _payload->d, _payload->u8_array, _payload->u16_array, _payload->u32_array, _payload->u64_array, _payload->s8_array, _payload->s16_array, _payload->s32_array, _payload->s64_array, _payload->f_array, _payload->d_array);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_test_types_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array)
{
    return fmav_msg_test_types_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_test_types_decode(const mavlink_message_t* msg, mavlink_test_types_t* payload)
{
    fmav_msg_test_types_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_TEST_TYPES_H

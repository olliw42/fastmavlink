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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MIN  179
#define FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX  179
#define FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN  179
#define FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA  103

#define FASTMAVLINK_MSG_ID_17000_LEN_MIN  179
#define FASTMAVLINK_MSG_ID_17000_LEN_MAX  179
#define FASTMAVLINK_MSG_ID_17000_LEN  179
#define FASTMAVLINK_MSG_ID_17000_CRCEXTRA  103

#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U64_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S64_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_D_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U32_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S32_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_F_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U16_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S16_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S_LEN  10
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_U8_ARRAY_LEN  3
#define FASTMAVLINK_MSG_TEST_TYPES_FIELD_S8_ARRAY_LEN  3

#define FASTMAVLINK_MSG_TEST_TYPES_FLAGS  0
#define FASTMAVLINK_MSG_TEST_TYPES_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_TEST_TYPES_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_TEST_TYPES_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_17000_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_17000_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message TEST_TYPES packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array,
    fmav_status_t* _status)
{
    fmav_test_types_t* _payload = (fmav_test_types_t*)msg->payload;

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

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_TEST_TYPES;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_test_types_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_test_types_pack(
        msg, sysid, compid,
        _payload->c, _payload->s, _payload->u8, _payload->u16, _payload->u32, _payload->u64, _payload->s8, _payload->s16, _payload->s32, _payload->s64, _payload->f, _payload->d, _payload->u8_array, _payload->u16_array, _payload->u32_array, _payload->u64_array, _payload->s8_array, _payload->s16_array, _payload->s32_array, _payload->s64_array, _payload->f_array, _payload->d_array,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array,
    fmav_status_t* _status)
{
    fmav_test_types_t* _payload = (fmav_test_types_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_TEST_TYPES;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_TEST_TYPES >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_TEST_TYPES >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_test_types_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_test_types_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_test_types_pack_to_frame_buf(
        buf, sysid, compid,
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
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_TEST_TYPES_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message TEST_TYPES unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_test_types_decode(fmav_test_types_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_TEST_TYPES_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_test_types_pack(
        msg, sysid, compid,
        c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_test_types_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    char c, const char* s, uint8_t u8, uint16_t u16, uint32_t u32, uint64_t u64, int8_t s8, int16_t s16, int32_t s32, int64_t s64, float f, double d, const uint8_t* u8_array, const uint16_t* u16_array, const uint32_t* u32_array, const uint64_t* u64_array, const int8_t* s8_array, const int16_t* s16_array, const int32_t* s32_array, const int64_t* s64_array, const float* f_array, const double* d_array)
{
    return fmav_msg_test_types_pack_to_frame_buf(
        (uint8_t*)buf,
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

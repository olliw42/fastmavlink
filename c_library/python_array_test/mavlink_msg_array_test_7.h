//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_7_H
#define FASTMAVLINK_MSG_ARRAY_TEST_7_H


//----------------------------------------
//-- Message ARRAY_TEST_7
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_7_t {
    double ar_d[2];
    float ar_f[2];
    uint32_t ar_u32[2];
    int32_t ar_i32[2];
    uint16_t ar_u16[2];
    int16_t ar_i16[2];
    uint8_t ar_u8[2];
    int8_t ar_i8[2];
    char ar_c[32];
}) fmav_array_test_7_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_7  17157


#define FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MIN  84
#define FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX  84
#define FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN  84
#define FASTMAVLINK_MSG_ARRAY_TEST_7_CRCEXTRA  187

#define FASTMAVLINK_MSG_ID_17157_LEN_MIN  84
#define FASTMAVLINK_MSG_ID_17157_LEN_MAX  84
#define FASTMAVLINK_MSG_ID_17157_LEN  84
#define FASTMAVLINK_MSG_ID_17157_CRCEXTRA  187

#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_D_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_F_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U32_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I32_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U16_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I16_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U8_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I8_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_C_LEN  32

#define FASTMAVLINK_MSG_ARRAY_TEST_7_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_7_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_7_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ARRAY_TEST_7 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_7_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const double* ar_d, const float* ar_f, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c,
    fmav_status_t* _status)
{
    fmav_array_test_7_t* _payload = (fmav_array_test_7_t*)msg->payload;


    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_f), ar_f, sizeof(float)*2);
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*2);
    memcpy(&(_payload->ar_i32), ar_i32, sizeof(int32_t)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);
    memcpy(&(_payload->ar_i16), ar_i16, sizeof(int16_t)*2);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*2);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*2);
    memcpy(&(_payload->ar_c), ar_c, sizeof(char)*32);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_7;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_7_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_7_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_7_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_7_pack(
        msg, sysid, compid,
        _payload->ar_d, _payload->ar_f, _payload->ar_u32, _payload->ar_i32, _payload->ar_u16, _payload->ar_i16, _payload->ar_u8, _payload->ar_i8, _payload->ar_c,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_7_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const double* ar_d, const float* ar_f, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c,
    fmav_status_t* _status)
{
    fmav_array_test_7_t* _payload = (fmav_array_test_7_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_f), ar_f, sizeof(float)*2);
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*2);
    memcpy(&(_payload->ar_i32), ar_i32, sizeof(int32_t)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);
    memcpy(&(_payload->ar_i16), ar_i16, sizeof(int16_t)*2);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*2);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*2);
    memcpy(&(_payload->ar_c), ar_c, sizeof(char)*32);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_7;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_7 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_7 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_7_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_7_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_7_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_7_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ar_d, _payload->ar_f, _payload->ar_u32, _payload->ar_i32, _payload->ar_u16, _payload->ar_i16, _payload->ar_u8, _payload->ar_i8, _payload->ar_c,
        _status);
}


//----------------------------------------
//-- Message ARRAY_TEST_7 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_7_decode(fmav_array_test_7_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_7_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_7  17157

#define mavlink_array_test_7_t  fmav_array_test_7_t

#define MAVLINK_MSG_ID_ARRAY_TEST_7_LEN  84
#define MAVLINK_MSG_ID_ARRAY_TEST_7_MIN_LEN  84
#define MAVLINK_MSG_ID_17157_LEN  84
#define MAVLINK_MSG_ID_17157_MIN_LEN  84

#define MAVLINK_MSG_ID_ARRAY_TEST_7_CRC  187
#define MAVLINK_MSG_ID_17157_CRC  187

#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_D_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_F_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I32_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I16_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_U8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_I8_LEN 2
#define MAVLINK_MSG_ARRAY_TEST_7_FIELD_AR_C_LEN 32


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_7_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const double* ar_d, const float* ar_f, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_7_pack(
        msg, sysid, compid,
        ar_d, ar_f, ar_u32, ar_i32, ar_u16, ar_i16, ar_u8, ar_i8, ar_c,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_7_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const double* ar_d, const float* ar_f, const uint32_t* ar_u32, const int32_t* ar_i32, const uint16_t* ar_u16, const int16_t* ar_i16, const uint8_t* ar_u8, const int8_t* ar_i8, const char* ar_c)
{
    return fmav_msg_array_test_7_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ar_d, ar_f, ar_u32, ar_i32, ar_u16, ar_i16, ar_u8, ar_i8, ar_c,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_7_decode(const mavlink_message_t* msg, mavlink_array_test_7_t* payload)
{
    fmav_msg_array_test_7_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_7_H

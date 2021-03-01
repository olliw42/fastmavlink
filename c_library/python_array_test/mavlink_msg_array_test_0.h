//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_0_H
#define FASTMAVLINK_MSG_ARRAY_TEST_0_H


//----------------------------------------
//-- Message ARRAY_TEST_0
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_0_t {
    uint32_t ar_u32[4];
    uint16_t ar_u16[4];
    uint8_t v1;
    int8_t ar_i8[4];
    uint8_t ar_u8[4];
}) fmav_array_test_0_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_0  17150

#define FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MIN  33
#define FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_ARRAY_TEST_0_CRCEXTRA  26

#define FASTMAVLINK_MSG_ARRAY_TEST_0_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_0_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_0_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ARRAY_TEST_0_FRAME_LEN_MAX  58

#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_LEN  4 // length of array = number of bytes

#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_OFS  16
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_V1_OFS  24
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_OFS  25
#define FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_OFS  29


//----------------------------------------
//-- Message ARRAY_TEST_0 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, const int8_t* ar_i8, const uint8_t* ar_u8, const uint16_t* ar_u16, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_0_t* _payload = (fmav_array_test_0_t*)msg->payload;

    _payload->v1 = v1;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*4);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*4);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_0;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_0_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_0_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_0_pack(
        msg, sysid, compid,
        _payload->v1, _payload->ar_i8, _payload->ar_u8, _payload->ar_u16, _payload->ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, const int8_t* ar_i8, const uint8_t* ar_u8, const uint16_t* ar_u16, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_0_t* _payload = (fmav_array_test_0_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v1 = v1;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*4);
    memcpy(&(_payload->ar_i8), ar_i8, sizeof(int8_t)*4);
    memcpy(&(_payload->ar_u8), ar_u8, sizeof(uint8_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_0;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_0 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_0 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_0_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_0_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_0_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->v1, _payload->ar_i8, _payload->ar_u8, _payload->ar_u16, _payload->ar_u32,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, const int8_t* ar_i8, const uint8_t* ar_u8, const uint16_t* ar_u16, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_0_t _payload;

    _payload.v1 = v1;
    memcpy(&(_payload.ar_u32), ar_u32, sizeof(uint32_t)*4);
    memcpy(&(_payload.ar_u16), ar_u16, sizeof(uint16_t)*4);
    memcpy(&(_payload.ar_i8), ar_i8, sizeof(int8_t)*4);
    memcpy(&(_payload.ar_u8), ar_u8, sizeof(uint8_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_0,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_0_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_0_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_0,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_0_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ARRAY_TEST_0 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_0_decode(fmav_array_test_0_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_0_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_array_test_0_get_field_v1(const fmav_message_t* msg)
{
    uint8_t r; 
    memcpy(&r, &(msg->payload[24]), sizeof(uint8_t)); 
    return r;     
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_array_test_0_get_field_ar_u32_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_array_test_0_get_field_ar_u32(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_NUM) return 0;
    return ((uint32_t*)&(msg->payload[0]))[index];     
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t* fmav_msg_array_test_0_get_field_ar_u16_ptr(const fmav_message_t* msg)
{
    return (uint16_t*)&(msg->payload[16]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_0_get_field_ar_u16(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_NUM) return 0;
    return ((uint16_t*)&(msg->payload[16]))[index];     
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_array_test_0_get_field_ar_i8_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[25]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_array_test_0_get_field_ar_i8(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_NUM) return 0;
    return ((int8_t*)&(msg->payload[25]))[index];     
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_array_test_0_get_field_ar_u8_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[29]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_array_test_0_get_field_ar_u8(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_NUM) return 0;
    return ((uint8_t*)&(msg->payload[29]))[index];     
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_0  17150

#define mavlink_array_test_0_t  fmav_array_test_0_t

#define MAVLINK_MSG_ID_ARRAY_TEST_0_LEN  33
#define MAVLINK_MSG_ID_ARRAY_TEST_0_MIN_LEN  33
#define MAVLINK_MSG_ID_17150_LEN  33
#define MAVLINK_MSG_ID_17150_MIN_LEN  33

#define MAVLINK_MSG_ID_ARRAY_TEST_0_CRC  26
#define MAVLINK_MSG_ID_17150_CRC  26

#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U32_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U16_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_I8_LEN 4
#define MAVLINK_MSG_ARRAY_TEST_0_FIELD_AR_U8_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_0_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t v1, const int8_t* ar_i8, const uint8_t* ar_u8, const uint16_t* ar_u16, const uint32_t* ar_u32)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_0_pack(
        msg, sysid, compid,
        v1, ar_i8, ar_u8, ar_u16, ar_u32,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_0_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v1, const int8_t* ar_i8, const uint8_t* ar_u8, const uint16_t* ar_u16, const uint32_t* ar_u32)
{
    return fmav_msg_array_test_0_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        v1, ar_i8, ar_u8, ar_u16, ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_0_decode(const mavlink_message_t* msg, mavlink_array_test_0_t* payload)
{
    fmav_msg_array_test_0_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_0_H

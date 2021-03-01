//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_3_H
#define FASTMAVLINK_MSG_ARRAY_TEST_3_H


//----------------------------------------
//-- Message ARRAY_TEST_3
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_3_t {
    uint32_t ar_u32[4];
    uint8_t v;
}) fmav_array_test_3_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_3  17153

#define FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MIN  17
#define FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_ARRAY_TEST_3_CRCEXTRA  19

#define FASTMAVLINK_MSG_ARRAY_TEST_3_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_3_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_3_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ARRAY_TEST_3_FRAME_LEN_MAX  42

#define FASTMAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_LEN  16 // length of array = number of bytes

#define FASTMAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_3_FIELD_V_OFS  16


//----------------------------------------
//-- Message ARRAY_TEST_3 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_3_t* _payload = (fmav_array_test_3_t*)msg->payload;

    _payload->v = v;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_3;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_3_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_3_pack(
        msg, sysid, compid,
        _payload->v, _payload->ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_3_t* _payload = (fmav_array_test_3_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v = v;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_3;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_3 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_3 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_3_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->v, _payload->ar_u32,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t v, const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_3_t _payload;

    _payload.v = v;
    memcpy(&(_payload.ar_u32), ar_u32, sizeof(uint32_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_3,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_3_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_3_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_3_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_3,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_3_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ARRAY_TEST_3 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_3_decode(fmav_array_test_3_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_3_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_array_test_3_get_field_v(const fmav_message_t* msg)
{
    uint8_t r; 
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t)); 
    return r;     
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_array_test_3_get_field_ar_u32_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[0]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_array_test_3_get_field_ar_u32(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_NUM) return 0;
    return ((uint32_t*)&(msg->payload[0]))[index];     
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_3  17153

#define mavlink_array_test_3_t  fmav_array_test_3_t

#define MAVLINK_MSG_ID_ARRAY_TEST_3_LEN  17
#define MAVLINK_MSG_ID_ARRAY_TEST_3_MIN_LEN  17
#define MAVLINK_MSG_ID_17153_LEN  17
#define MAVLINK_MSG_ID_17153_MIN_LEN  17

#define MAVLINK_MSG_ID_ARRAY_TEST_3_CRC  19
#define MAVLINK_MSG_ID_17153_CRC  19

#define MAVLINK_MSG_ARRAY_TEST_3_FIELD_AR_U32_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_3_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t v, const uint32_t* ar_u32)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_3_pack(
        msg, sysid, compid,
        v, ar_u32,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_3_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t v, const uint32_t* ar_u32)
{
    return fmav_msg_array_test_3_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        v, ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_3_decode(const mavlink_message_t* msg, mavlink_array_test_3_t* payload)
{
    fmav_msg_array_test_3_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_3_H

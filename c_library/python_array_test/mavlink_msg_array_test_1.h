//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_1_H
#define FASTMAVLINK_MSG_ARRAY_TEST_1_H


//----------------------------------------
//-- Message ARRAY_TEST_1
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_1_t {
    uint32_t ar_u32[4];
}) fmav_array_test_1_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_1  17151


#define FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MIN  16
#define FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX  16
#define FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN  16
#define FASTMAVLINK_MSG_ARRAY_TEST_1_CRCEXTRA  72

#define FASTMAVLINK_MSG_ID_17151_LEN_MIN  16
#define FASTMAVLINK_MSG_ID_17151_LEN_MAX  16
#define FASTMAVLINK_MSG_ID_17151_LEN  16
#define FASTMAVLINK_MSG_ID_17151_CRCEXTRA  72

#define FASTMAVLINK_MSG_ARRAY_TEST_1_FIELD_AR_U32_LEN  4

#define FASTMAVLINK_MSG_ARRAY_TEST_1_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_1_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_1_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ARRAY_TEST_1_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_17151_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_17151_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message ARRAY_TEST_1 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_1_t* _payload = (fmav_array_test_1_t*)msg->payload;


    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_1;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_1_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_1_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_1_pack(
        msg, sysid, compid,
        _payload->ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_1_t* _payload = (fmav_array_test_1_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);


    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_1;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_1 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_1 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_1_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_1_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_1_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ar_u32,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32,
    fmav_status_t* _status)
{
    fmav_array_test_1_t _payload;


    memcpy(&(_payload.ar_u32), ar_u32, sizeof(uint32_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_1,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_1_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_1_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_1_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ARRAY_TEST_1,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_1_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ARRAY_TEST_1 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_1_decode(fmav_array_test_1_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_1_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_1  17151

#define mavlink_array_test_1_t  fmav_array_test_1_t

#define MAVLINK_MSG_ID_ARRAY_TEST_1_LEN  16
#define MAVLINK_MSG_ID_ARRAY_TEST_1_MIN_LEN  16
#define MAVLINK_MSG_ID_17151_LEN  16
#define MAVLINK_MSG_ID_17151_MIN_LEN  16

#define MAVLINK_MSG_ID_ARRAY_TEST_1_CRC  72
#define MAVLINK_MSG_ID_17151_CRC  72

#define MAVLINK_MSG_ARRAY_TEST_1_FIELD_AR_U32_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_1_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const uint32_t* ar_u32)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_1_pack(
        msg, sysid, compid,
        ar_u32,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_1_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32)
{
    return fmav_msg_array_test_1_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ar_u32,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_1_decode(const mavlink_message_t* msg, mavlink_array_test_1_t* payload)
{
    fmav_msg_array_test_1_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_1_H

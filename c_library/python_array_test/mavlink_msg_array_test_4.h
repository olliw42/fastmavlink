//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ARRAY_TEST_4_H
#define FASTMAVLINK_MSG_ARRAY_TEST_4_H


//----------------------------------------
//-- Message ARRAY_TEST_4
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_4_t {
    uint32_t ar_u32[4];
    uint8_t v;
}) fmav_array_test_4_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_4  17154


#define FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MIN  17
#define FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN  17
#define FASTMAVLINK_MSG_ARRAY_TEST_4_CRCEXTRA  89

#define FASTMAVLINK_MSG_ID_17154_LEN_MIN  17
#define FASTMAVLINK_MSG_ID_17154_LEN_MAX  17
#define FASTMAVLINK_MSG_ID_17154_LEN  17
#define FASTMAVLINK_MSG_ID_17154_CRCEXTRA  89

#define FASTMAVLINK_MSG_ARRAY_TEST_4_FIELD_AR_U32_LEN  4

#define FASTMAVLINK_MSG_ARRAY_TEST_4_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_4_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_4_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ARRAY_TEST_4 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_4_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32, uint8_t v,
    fmav_status_t* _status)
{
    fmav_array_test_4_t* _payload = (fmav_array_test_4_t*)msg->payload;

    _payload->v = v;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_4;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_4_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_4_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_4_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_4_pack(
        msg, sysid, compid,
        _payload->ar_u32, _payload->v,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_4_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32, uint8_t v,
    fmav_status_t* _status)
{
    fmav_array_test_4_t* _payload = (fmav_array_test_4_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v = v;
    memcpy(&(_payload->ar_u32), ar_u32, sizeof(uint32_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_4;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_4 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_4 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_4_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_4_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_4_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_4_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->ar_u32, _payload->v,
        _status);
}


//----------------------------------------
//-- Message ARRAY_TEST_4 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_4_decode(fmav_array_test_4_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_4_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ARRAY_TEST_4  17154

#define mavlink_array_test_4_t  fmav_array_test_4_t

#define MAVLINK_MSG_ID_ARRAY_TEST_4_LEN  17
#define MAVLINK_MSG_ID_ARRAY_TEST_4_MIN_LEN  17
#define MAVLINK_MSG_ID_17154_LEN  17
#define MAVLINK_MSG_ID_17154_MIN_LEN  17

#define MAVLINK_MSG_ID_ARRAY_TEST_4_CRC  89
#define MAVLINK_MSG_ID_17154_CRC  89

#define MAVLINK_MSG_ARRAY_TEST_4_FIELD_AR_U32_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_4_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    const uint32_t* ar_u32, uint8_t v)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_4_pack(
        msg, sysid, compid,
        ar_u32, v,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_4_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    const uint32_t* ar_u32, uint8_t v)
{
    return fmav_msg_array_test_4_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        ar_u32, v,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_array_test_4_decode(const mavlink_message_t* msg, mavlink_array_test_4_t* payload)
{
    fmav_msg_array_test_4_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ARRAY_TEST_4_H

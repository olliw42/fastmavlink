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

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_array_test_8_t {
    double ar_d[2];
    uint32_t v3;
    uint16_t ar_u16[2];
}) fmav_array_test_8_t;


#define FASTMAVLINK_MSG_ID_ARRAY_TEST_8  17158


#define FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MIN  24
#define FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX  24
#define FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN  24
#define FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA  106

#define FASTMAVLINK_MSG_ID_17158_LEN_MIN  24
#define FASTMAVLINK_MSG_ID_17158_LEN_MAX  24
#define FASTMAVLINK_MSG_ID_17158_LEN  24
#define FASTMAVLINK_MSG_ID_17158_CRCEXTRA  106

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_D_LEN  2
#define FASTMAVLINK_MSG_ARRAY_TEST_8_FIELD_AR_U16_LEN  2

#define FASTMAVLINK_MSG_ARRAY_TEST_8_FLAGS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_8_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ARRAY_TEST_8_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ARRAY_TEST_8 packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16,
    fmav_status_t* _status)
{
    fmav_array_test_8_t* _payload = (fmav_array_test_8_t*)msg->payload;

    _payload->v3 = v3;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ARRAY_TEST_8;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_8_pack(
        msg, sysid, compid,
        _payload->v3, _payload->ar_d, _payload->ar_u16,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16,
    fmav_status_t* _status)
{
    fmav_array_test_8_t* _payload = (fmav_array_test_8_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->v3 = v3;
    memcpy(&(_payload->ar_d), ar_d, sizeof(double)*2);
    memcpy(&(_payload->ar_u16), ar_u16, sizeof(uint16_t)*2);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8 >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ARRAY_TEST_8 >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ARRAY_TEST_8_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_array_test_8_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_array_test_8_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_array_test_8_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->v3, _payload->ar_d, _payload->ar_u16,
        _status);
}


//----------------------------------------
//-- Message ARRAY_TEST_8 unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_array_test_8_decode(fmav_array_test_8_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ARRAY_TEST_8_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_array_test_8_pack(
        msg, sysid, compid,
        v3, ar_d, ar_u16,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_array_test_8_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t v3, const double* ar_d, const uint16_t* ar_u16)
{
    return fmav_msg_array_test_8_pack_to_frame_buf(
        (uint8_t*)buf,
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

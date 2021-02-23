//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AUTOPILOT_VERSION_H
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_H


//----------------------------------------
//-- Message AUTOPILOT_VERSION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_autopilot_version_t {
    uint64_t capabilities;
    uint64_t uid;
    uint32_t flight_sw_version;
    uint32_t middleware_sw_version;
    uint32_t os_sw_version;
    uint32_t board_version;
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t flight_custom_version[8];
    uint8_t middleware_custom_version[8];
    uint8_t os_custom_version[8];
    uint8_t uid2[18];
}) fmav_autopilot_version_t;


#define FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION  148


#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MIN  60
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX  78
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN  78
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA  178

#define FASTMAVLINK_MSG_ID_148_LEN_MIN  60
#define FASTMAVLINK_MSG_ID_148_LEN_MAX  78
#define FASTMAVLINK_MSG_ID_148_LEN  78
#define FASTMAVLINK_MSG_ID_148_CRCEXTRA  178

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN  8
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN  8
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN  8
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN  18

#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_FLAGS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AUTOPILOT_VERSION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message AUTOPILOT_VERSION packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
    fmav_status_t* _status)
{
    fmav_autopilot_version_t* _payload = (fmav_autopilot_version_t*)msg->payload;

    _payload->capabilities = capabilities;
    _payload->uid = uid;
    _payload->flight_sw_version = flight_sw_version;
    _payload->middleware_sw_version = middleware_sw_version;
    _payload->os_sw_version = os_sw_version;
    _payload->board_version = board_version;
    _payload->vendor_id = vendor_id;
    _payload->product_id = product_id;
    memcpy(&(_payload->flight_custom_version), flight_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->middleware_custom_version), middleware_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->os_custom_version), os_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->uid2), uid2, sizeof(uint8_t)*18);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_pack(
        msg, sysid, compid,
        _payload->capabilities, _payload->flight_sw_version, _payload->middleware_sw_version, _payload->os_sw_version, _payload->board_version, _payload->flight_custom_version, _payload->middleware_custom_version, _payload->os_custom_version, _payload->vendor_id, _payload->product_id, _payload->uid, _payload->uid2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2,
    fmav_status_t* _status)
{
    fmav_autopilot_version_t* _payload = (fmav_autopilot_version_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->capabilities = capabilities;
    _payload->uid = uid;
    _payload->flight_sw_version = flight_sw_version;
    _payload->middleware_sw_version = middleware_sw_version;
    _payload->os_sw_version = os_sw_version;
    _payload->board_version = board_version;
    _payload->vendor_id = vendor_id;
    _payload->product_id = product_id;
    memcpy(&(_payload->flight_custom_version), flight_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->middleware_custom_version), middleware_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->os_custom_version), os_custom_version, sizeof(uint8_t)*8);
    memcpy(&(_payload->uid2), uid2, sizeof(uint8_t)*18);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AUTOPILOT_VERSION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AUTOPILOT_VERSION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_autopilot_version_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_autopilot_version_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_autopilot_version_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->capabilities, _payload->flight_sw_version, _payload->middleware_sw_version, _payload->os_sw_version, _payload->board_version, _payload->flight_custom_version, _payload->middleware_custom_version, _payload->os_custom_version, _payload->vendor_id, _payload->product_id, _payload->uid, _payload->uid2,
        _status);
}


//----------------------------------------
//-- Message AUTOPILOT_VERSION unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_autopilot_version_decode(fmav_autopilot_version_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AUTOPILOT_VERSION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION  148

#define mavlink_autopilot_version_t  fmav_autopilot_version_t

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_LEN  78
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN  60
#define MAVLINK_MSG_ID_148_LEN  78
#define MAVLINK_MSG_ID_148_MIN_LEN  60

#define MAVLINK_MSG_ID_AUTOPILOT_VERSION_CRC  178
#define MAVLINK_MSG_ID_148_CRC  178

#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN 8
#define MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN 18


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_autopilot_version_pack(
        msg, sysid, compid,
        capabilities, flight_sw_version, middleware_sw_version, os_sw_version, board_version, flight_custom_version, middleware_custom_version, os_custom_version, vendor_id, product_id, uid, uid2,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_autopilot_version_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t* flight_custom_version, const uint8_t* middleware_custom_version, const uint8_t* os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t* uid2)
{
    return fmav_msg_autopilot_version_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        capabilities, flight_sw_version, middleware_sw_version, os_sw_version, board_version, flight_custom_version, middleware_custom_version, os_custom_version, vendor_id, product_id, uid, uid2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_autopilot_version_decode(const mavlink_message_t* msg, mavlink_autopilot_version_t* payload)
{
    fmav_msg_autopilot_version_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AUTOPILOT_VERSION_H

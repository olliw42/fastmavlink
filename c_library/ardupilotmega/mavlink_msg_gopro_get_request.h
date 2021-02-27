//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GOPRO_GET_REQUEST_H
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_H


//----------------------------------------
//-- Message GOPRO_GET_REQUEST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gopro_get_request_t {
    uint8_t target_system;
    uint8_t target_component;
    uint8_t cmd_id;
}) fmav_gopro_get_request_t;


#define FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST  216


#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MIN  3
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN  3
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA  50

#define FASTMAVLINK_MSG_ID_216_LEN_MIN  3
#define FASTMAVLINK_MSG_ID_216_LEN_MAX  3
#define FASTMAVLINK_MSG_ID_216_LEN  3
#define FASTMAVLINK_MSG_ID_216_CRCEXTRA  50



#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_FLAGS  3
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_TARGET_COMPONENT_OFS  1

#define FASTMAVLINK_MSG_GOPRO_GET_REQUEST_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_216_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_216_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message GOPRO_GET_REQUEST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id,
    fmav_status_t* _status)
{
    fmav_gopro_get_request_t* _payload = (fmav_gopro_get_request_t*)msg->payload;

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_get_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_get_request_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->cmd_id,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id,
    fmav_status_t* _status)
{
    fmav_gopro_get_request_t* _payload = (fmav_gopro_get_request_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->cmd_id = cmd_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_get_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_get_request_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->cmd_id,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id,
    fmav_status_t* _status)
{
    fmav_gopro_get_request_t _payload;

    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.cmd_id = cmd_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_get_request_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_get_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GOPRO_GET_REQUEST,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_GET_REQUEST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GOPRO_GET_REQUEST unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_get_request_decode(fmav_gopro_get_request_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GOPRO_GET_REQUEST_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST  216

#define mavlink_gopro_get_request_t  fmav_gopro_get_request_t

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_LEN  3
#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN  3
#define MAVLINK_MSG_ID_216_LEN  3
#define MAVLINK_MSG_ID_216_MIN_LEN  3

#define MAVLINK_MSG_ID_GOPRO_GET_REQUEST_CRC  50
#define MAVLINK_MSG_ID_216_CRC  50




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_get_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gopro_get_request_pack(
        msg, sysid, compid,
        target_system, target_component, cmd_id,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_get_request_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t cmd_id)
{
    return fmav_msg_gopro_get_request_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, cmd_id,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gopro_get_request_decode(const mavlink_message_t* msg, mavlink_gopro_get_request_t* payload)
{
    fmav_msg_gopro_get_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GOPRO_GET_REQUEST_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_H
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_H


//----------------------------------------
//-- Message GOPRO_SET_RESPONSE
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gopro_set_response_t {
    uint8_t cmd_id;
    uint8_t status;
}) fmav_gopro_set_response_t;


#define FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE  219


#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MIN  2
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN  2
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_CRCEXTRA  162

#define FASTMAVLINK_MSG_ID_219_LEN_MIN  2
#define FASTMAVLINK_MSG_ID_219_LEN_MAX  2
#define FASTMAVLINK_MSG_ID_219_LEN  2
#define FASTMAVLINK_MSG_ID_219_CRCEXTRA  162



#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_FLAGS  0
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message GOPRO_SET_RESPONSE packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_response_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t cmd_id, uint8_t status,
    fmav_status_t* _status)
{
    fmav_gopro_set_response_t* _payload = (fmav_gopro_set_response_t*)msg->payload;

    _payload->cmd_id = cmd_id;
    _payload->status = status;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_response_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_response_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_response_pack(
        msg, sysid, compid,
        _payload->cmd_id, _payload->status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_response_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t cmd_id, uint8_t status,
    fmav_status_t* _status)
{
    fmav_gopro_set_response_t* _payload = (fmav_gopro_set_response_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->cmd_id = cmd_id;
    _payload->status = status;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GOPRO_SET_RESPONSE >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gopro_set_response_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gopro_set_response_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gopro_set_response_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->cmd_id, _payload->status,
        _status);
}


//----------------------------------------
//-- Message GOPRO_SET_RESPONSE unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gopro_set_response_decode(fmav_gopro_set_response_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GOPRO_SET_RESPONSE  219

#define mavlink_gopro_set_response_t  fmav_gopro_set_response_t

#define MAVLINK_MSG_ID_GOPRO_SET_RESPONSE_LEN  2
#define MAVLINK_MSG_ID_GOPRO_SET_RESPONSE_MIN_LEN  2
#define MAVLINK_MSG_ID_219_LEN  2
#define MAVLINK_MSG_ID_219_MIN_LEN  2

#define MAVLINK_MSG_ID_GOPRO_SET_RESPONSE_CRC  162
#define MAVLINK_MSG_ID_219_CRC  162




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_response_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t cmd_id, uint8_t status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gopro_set_response_pack(
        msg, sysid, compid,
        cmd_id, status,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gopro_set_response_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t cmd_id, uint8_t status)
{
    return fmav_msg_gopro_set_response_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        cmd_id, status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gopro_set_response_decode(const mavlink_message_t* msg, mavlink_gopro_set_response_t* payload)
{
    fmav_msg_gopro_set_response_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GOPRO_SET_RESPONSE_H

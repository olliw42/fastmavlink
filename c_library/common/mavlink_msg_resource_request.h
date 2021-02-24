//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RESOURCE_REQUEST_H
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_H


//----------------------------------------
//-- Message RESOURCE_REQUEST
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_resource_request_t {
    uint8_t request_id;
    uint8_t uri_type;
    uint8_t uri[120];
    uint8_t transfer_type;
    uint8_t storage[120];
}) fmav_resource_request_t;


#define FASTMAVLINK_MSG_ID_RESOURCE_REQUEST  142


#define FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MIN  243
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX  243
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN  243
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA  72

#define FASTMAVLINK_MSG_ID_142_LEN_MIN  243
#define FASTMAVLINK_MSG_ID_142_LEN_MAX  243
#define FASTMAVLINK_MSG_ID_142_LEN  243
#define FASTMAVLINK_MSG_ID_142_CRCEXTRA  72

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_LEN  120
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_LEN  120

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FLAGS  0
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message RESOURCE_REQUEST packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t request_id, uint8_t uri_type, const uint8_t* uri, uint8_t transfer_type, const uint8_t* storage,
    fmav_status_t* _status)
{
    fmav_resource_request_t* _payload = (fmav_resource_request_t*)msg->payload;

    _payload->request_id = request_id;
    _payload->uri_type = uri_type;
    _payload->transfer_type = transfer_type;
    memcpy(&(_payload->uri), uri, sizeof(uint8_t)*120);
    memcpy(&(_payload->storage), storage, sizeof(uint8_t)*120);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_RESOURCE_REQUEST;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_resource_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_resource_request_pack(
        msg, sysid, compid,
        _payload->request_id, _payload->uri_type, _payload->uri, _payload->transfer_type, _payload->storage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t request_id, uint8_t uri_type, const uint8_t* uri, uint8_t transfer_type, const uint8_t* storage,
    fmav_status_t* _status)
{
    fmav_resource_request_t* _payload = (fmav_resource_request_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->request_id = request_id;
    _payload->uri_type = uri_type;
    _payload->transfer_type = transfer_type;
    memcpy(&(_payload->uri), uri, sizeof(uint8_t)*120);
    memcpy(&(_payload->storage), storage, sizeof(uint8_t)*120);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RESOURCE_REQUEST;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RESOURCE_REQUEST >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RESOURCE_REQUEST >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_resource_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_resource_request_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->request_id, _payload->uri_type, _payload->uri, _payload->transfer_type, _payload->storage,
        _status);
}


//----------------------------------------
//-- Message RESOURCE_REQUEST unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_resource_request_decode(fmav_resource_request_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RESOURCE_REQUEST  142

#define mavlink_resource_request_t  fmav_resource_request_t

#define MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN  243
#define MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN  243
#define MAVLINK_MSG_ID_142_LEN  243
#define MAVLINK_MSG_ID_142_MIN_LEN  243

#define MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC  72
#define MAVLINK_MSG_ID_142_CRC  72

#define MAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_LEN 120
#define MAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_LEN 120


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_resource_request_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t request_id, uint8_t uri_type, const uint8_t* uri, uint8_t transfer_type, const uint8_t* storage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_resource_request_pack(
        msg, sysid, compid,
        request_id, uri_type, uri, transfer_type, storage,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_resource_request_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t request_id, uint8_t uri_type, const uint8_t* uri, uint8_t transfer_type, const uint8_t* storage)
{
    return fmav_msg_resource_request_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        request_id, uri_type, uri, transfer_type, storage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_resource_request_decode(const mavlink_message_t* msg, mavlink_resource_request_t* payload)
{
    fmav_msg_resource_request_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RESOURCE_REQUEST_H

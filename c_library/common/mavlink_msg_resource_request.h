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

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX  243
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA  72

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FLAGS  0
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FRAME_LEN_MAX  268

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_NUM  120 // number of elements in array
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_LEN  120 // length of array = number of bytes
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_NUM  120 // number of elements in array
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_LEN  120 // length of array = number of bytes

#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_REQUEST_ID_OFS  0
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_TYPE_OFS  1
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_OFS  2
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_TRANSFER_TYPE_OFS  122
#define FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_OFS  123


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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t request_id, uint8_t uri_type, const uint8_t* uri, uint8_t transfer_type, const uint8_t* storage,
    fmav_status_t* _status)
{
    fmav_resource_request_t _payload;

    _payload.request_id = request_id;
    _payload.uri_type = uri_type;
    _payload.transfer_type = transfer_type;
    memcpy(&(_payload.uri), uri, sizeof(uint8_t)*120);
    memcpy(&(_payload.storage), storage, sizeof(uint8_t)*120);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RESOURCE_REQUEST,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_resource_request_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_resource_request_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RESOURCE_REQUEST,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RESOURCE_REQUEST_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RESOURCE_REQUEST unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_resource_request_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_resource_request_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_resource_request_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_resource_request_decode(fmav_resource_request_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RESOURCE_REQUEST_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_resource_request_get_field_request_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_resource_request_get_field_uri_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[1]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_resource_request_get_field_transfer_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[122]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_resource_request_get_field_uri_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[2]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_resource_request_get_field_uri(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_NUM) return 0;
    return ((uint8_t*)&(msg->payload[2]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_resource_request_get_field_storage_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[123]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_resource_request_get_field_storage(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_NUM) return 0;
    return ((uint8_t*)&(msg->payload[123]))[index];
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

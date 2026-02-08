//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_GET
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_get_t {
    uint32_t ReqMessageId;
}) fmav_uavionix_adsb_get_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET  10006

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX  4
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_CRCEXTRA  193

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_FRAME_LEN_MAX  29



#define FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_FIELD_REQMESSAGEID_OFS  0


//----------------------------------------
//-- Message UAVIONIX_ADSB_GET pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ReqMessageId,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_get_t* _payload = (fmav_uavionix_adsb_get_t*)_msg->payload;

    _payload->ReqMessageId = ReqMessageId;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_get_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_get_pack(
        _msg, sysid, compid,
        _payload->ReqMessageId,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ReqMessageId,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_get_t* _payload = (fmav_uavionix_adsb_get_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ReqMessageId = ReqMessageId;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_get_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_get_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->ReqMessageId,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t ReqMessageId,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_get_t _payload;

    _payload.ReqMessageId = ReqMessageId;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_get_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_get_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_GET,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_GET decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_get_decode(fmav_uavionix_adsb_get_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavionix_adsb_get_get_field_ReqMessageId(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET  10006

#define mavlink_uavionix_adsb_get_t  fmav_uavionix_adsb_get_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_LEN  4
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_MIN_LEN  4
#define MAVLINK_MSG_ID_10006_LEN  4
#define MAVLINK_MSG_ID_10006_MIN_LEN  4

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_GET_CRC  193
#define MAVLINK_MSG_ID_10006_CRC  193




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_get_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t ReqMessageId)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_get_pack(
        _msg, sysid, compid,
        ReqMessageId,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_get_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_get_t* _payload)
{
    return mavlink_msg_uavionix_adsb_get_pack(
        sysid,
        compid,
        _msg,
        _payload->ReqMessageId);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_get_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t ReqMessageId)
{
    return fmav_msg_uavionix_adsb_get_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        ReqMessageId,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_get_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_get_t* payload)
{
    fmav_msg_uavionix_adsb_get_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_GET_H

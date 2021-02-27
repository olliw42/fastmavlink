//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CELLULAR_STATUS_H
#define FASTMAVLINK_MSG_CELLULAR_STATUS_H


//----------------------------------------
//-- Message CELLULAR_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_cellular_status_t {
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    uint8_t status;
    uint8_t failure_reason;
    uint8_t type;
    uint8_t quality;
}) fmav_cellular_status_t;


#define FASTMAVLINK_MSG_ID_CELLULAR_STATUS  334


#define FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MIN  10
#define FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX  10
#define FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN  10
#define FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA  72

#define FASTMAVLINK_MSG_ID_334_LEN_MIN  10
#define FASTMAVLINK_MSG_ID_334_LEN_MAX  10
#define FASTMAVLINK_MSG_ID_334_LEN  10
#define FASTMAVLINK_MSG_ID_334_CRCEXTRA  72



#define FASTMAVLINK_MSG_CELLULAR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CELLULAR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CELLULAR_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CELLULAR_STATUS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_334_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_334_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message CELLULAR_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t* _payload = (fmav_cellular_status_t*)msg->payload;

    _payload->mcc = mcc;
    _payload->mnc = mnc;
    _payload->lac = lac;
    _payload->status = status;
    _payload->failure_reason = failure_reason;
    _payload->type = type;
    _payload->quality = quality;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CELLULAR_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_status_pack(
        msg, sysid, compid,
        _payload->status, _payload->failure_reason, _payload->type, _payload->quality, _payload->mcc, _payload->mnc, _payload->lac,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t* _payload = (fmav_cellular_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mcc = mcc;
    _payload->mnc = mnc;
    _payload->lac = lac;
    _payload->status = status;
    _payload->failure_reason = failure_reason;
    _payload->type = type;
    _payload->quality = quality;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CELLULAR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_cellular_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->status, _payload->failure_reason, _payload->type, _payload->quality, _payload->mcc, _payload->mnc, _payload->lac,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac,
    fmav_status_t* _status)
{
    fmav_cellular_status_t _payload;

    _payload.mcc = mcc;
    _payload.mnc = mnc;
    _payload.lac = lac;
    _payload.status = status;
    _payload.failure_reason = failure_reason;
    _payload.type = type;
    _payload.quality = quality;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_STATUS,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_cellular_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_cellular_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CELLULAR_STATUS,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CELLULAR_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CELLULAR_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_cellular_status_decode(fmav_cellular_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CELLULAR_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CELLULAR_STATUS  334

#define mavlink_cellular_status_t  fmav_cellular_status_t

#define MAVLINK_MSG_ID_CELLULAR_STATUS_LEN  10
#define MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN  10
#define MAVLINK_MSG_ID_334_LEN  10
#define MAVLINK_MSG_ID_334_MIN_LEN  10

#define MAVLINK_MSG_ID_CELLULAR_STATUS_CRC  72
#define MAVLINK_MSG_ID_334_CRC  72




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_cellular_status_pack(
        msg, sysid, compid,
        status, failure_reason, type, quality, mcc, mnc, lac,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_cellular_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t status, uint8_t failure_reason, uint8_t type, uint8_t quality, uint16_t mcc, uint16_t mnc, uint16_t lac)
{
    return fmav_msg_cellular_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        status, failure_reason, type, quality, mcc, mnc, lac,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_cellular_status_decode(const mavlink_message_t* msg, mavlink_cellular_status_t* payload)
{
    fmav_msg_cellular_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CELLULAR_STATUS_H

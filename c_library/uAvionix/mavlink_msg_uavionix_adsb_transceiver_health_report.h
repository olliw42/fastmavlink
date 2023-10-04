//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_H
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_H


//----------------------------------------
//-- Message UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavionix_adsb_transceiver_health_report_t {
    uint8_t rfHealth;
}) fmav_uavionix_adsb_transceiver_health_report_t;


#define FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT  10003

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX  1
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA  4

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_FLAGS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_FRAME_LEN_MAX  26



#define FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_FIELD_RFHEALTH_OFS  0


//----------------------------------------
//-- Message UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_transceiver_health_report_t* _payload = (fmav_uavionix_adsb_transceiver_health_report_t*)_msg->payload;

    _payload->rfHealth = rfHealth;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_transceiver_health_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack(
        _msg, sysid, compid,
        _payload->rfHealth,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_transceiver_health_report_t* _payload = (fmav_uavionix_adsb_transceiver_health_report_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rfHealth = rfHealth;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_transceiver_health_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->rfHealth,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_transceiver_health_report_t _payload;

    _payload.rfHealth = rfHealth;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_transceiver_health_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_transceiver_health_report_decode(fmav_uavionix_adsb_transceiver_health_report_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavionix_adsb_transceiver_health_report_get_field_rfHealth(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT  10003

#define mavlink_uavionix_adsb_transceiver_health_report_t  fmav_uavionix_adsb_transceiver_health_report_t

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN  1
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN  1
#define MAVLINK_MSG_ID_10003_LEN  1
#define MAVLINK_MSG_ID_10003_MIN_LEN  1

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC  4
#define MAVLINK_MSG_ID_10003_CRC  4




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t rfHealth)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack(
        _msg, sysid, compid,
        rfHealth,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_uavionix_adsb_transceiver_health_report_t* _payload)
{
    return mavlink_msg_uavionix_adsb_transceiver_health_report_pack(
        sysid,
        compid,
        _msg,
        _payload->rfHealth);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        rfHealth,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavionix_adsb_transceiver_health_report_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_transceiver_health_report_t* payload)
{
    fmav_msg_uavionix_adsb_transceiver_health_report_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_H

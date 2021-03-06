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

// fields are ordered, as they are on the wire
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
//-- Message UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_transceiver_health_report_t* _payload = (fmav_uavionix_adsb_transceiver_health_report_t*)msg->payload;

    _payload->rfHealth = rfHealth;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_transceiver_health_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack(
        msg, sysid, compid,
        _payload->rfHealth,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth,
    fmav_status_t* _status)
{
    fmav_uavionix_adsb_transceiver_health_report_t* _payload = (fmav_uavionix_adsb_transceiver_health_report_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rfHealth = rfHealth;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavionix_adsb_transceiver_health_report_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavionix_adsb_transceiver_health_report_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
        buf, sysid, compid,
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
//-- Message UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_uavionix_adsb_transceiver_health_report_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_uavionix_adsb_transceiver_health_report_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_transceiver_health_report_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavionix_adsb_transceiver_health_report_decode(fmav_uavionix_adsb_transceiver_health_report_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_PAYLOAD_LEN_MAX);
    }
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
    mavlink_message_t* msg,
    uint8_t rfHealth)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack(
        msg, sysid, compid,
        rfHealth,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rfHealth)
{
    return fmav_msg_uavionix_adsb_transceiver_health_report_pack_to_frame_buf(
        (uint8_t*)buf,
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

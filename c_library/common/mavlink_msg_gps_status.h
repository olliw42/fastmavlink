//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_STATUS_H
#define FASTMAVLINK_MSG_GPS_STATUS_H


//----------------------------------------
//-- Message GPS_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_status_t {
    uint8_t satellites_visible;
    uint8_t satellite_prn[20];
    uint8_t satellite_used[20];
    uint8_t satellite_elevation[20];
    uint8_t satellite_azimuth[20];
    uint8_t satellite_snr[20];
}) fmav_gps_status_t;


#define FASTMAVLINK_MSG_ID_GPS_STATUS  25


#define FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MIN  101
#define FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX  101
#define FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN  101
#define FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA  23

#define FASTMAVLINK_MSG_ID_25_LEN_MIN  101
#define FASTMAVLINK_MSG_ID_25_LEN_MAX  101
#define FASTMAVLINK_MSG_ID_25_LEN  101
#define FASTMAVLINK_MSG_ID_25_CRCEXTRA  23

#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN  20
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN  20
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN  20
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN  20
#define FASTMAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN  20

#define FASTMAVLINK_MSG_GPS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GPS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message GPS_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr,
    fmav_status_t* _status)
{
    fmav_gps_status_t* _payload = (fmav_gps_status_t*)msg->payload;

    _payload->satellites_visible = satellites_visible;
    memcpy(&(_payload->satellite_prn), satellite_prn, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_used), satellite_used, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_elevation), satellite_elevation, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_azimuth), satellite_azimuth, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_snr), satellite_snr, sizeof(uint8_t)*20);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_status_pack(
        msg, sysid, compid,
        _payload->satellites_visible, _payload->satellite_prn, _payload->satellite_used, _payload->satellite_elevation, _payload->satellite_azimuth, _payload->satellite_snr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr,
    fmav_status_t* _status)
{
    fmav_gps_status_t* _payload = (fmav_gps_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->satellites_visible = satellites_visible;
    memcpy(&(_payload->satellite_prn), satellite_prn, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_used), satellite_used, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_elevation), satellite_elevation, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_azimuth), satellite_azimuth, sizeof(uint8_t)*20);
    memcpy(&(_payload->satellite_snr), satellite_snr, sizeof(uint8_t)*20);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->satellites_visible, _payload->satellite_prn, _payload->satellite_used, _payload->satellite_elevation, _payload->satellite_azimuth, _payload->satellite_snr,
        _status);
}


//----------------------------------------
//-- Message GPS_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_status_decode(fmav_gps_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GPS_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_STATUS  25

#define mavlink_gps_status_t  fmav_gps_status_t

#define MAVLINK_MSG_ID_GPS_STATUS_LEN  101
#define MAVLINK_MSG_ID_GPS_STATUS_MIN_LEN  101
#define MAVLINK_MSG_ID_25_LEN  101
#define MAVLINK_MSG_ID_25_MIN_LEN  101

#define MAVLINK_MSG_ID_GPS_STATUS_CRC  23
#define MAVLINK_MSG_ID_25_CRC  23

#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN 20


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_status_pack(
        msg, sysid, compid,
        satellites_visible, satellite_prn, satellite_used, satellite_elevation, satellite_azimuth, satellite_snr,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t satellites_visible, const uint8_t* satellite_prn, const uint8_t* satellite_used, const uint8_t* satellite_elevation, const uint8_t* satellite_azimuth, const uint8_t* satellite_snr)
{
    return fmav_msg_gps_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        satellites_visible, satellite_prn, satellite_used, satellite_elevation, satellite_azimuth, satellite_snr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_status_decode(const mavlink_message_t* msg, mavlink_gps_status_t* payload)
{
    fmav_msg_gps_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_STATUS_H

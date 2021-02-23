//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_tracking_geo_status_t {
    int32_t lat;
    int32_t lon;
    float alt;
    float h_acc;
    float v_acc;
    float vel_n;
    float vel_e;
    float vel_d;
    float vel_acc;
    float dist;
    float hdg;
    float hdg_acc;
    uint8_t tracking_status;
}) fmav_camera_tracking_geo_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS  276


#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MIN  49
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX  49
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN  49
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA  18

#define FASTMAVLINK_MSG_ID_276_LEN_MIN  49
#define FASTMAVLINK_MSG_ID_276_LEN_MAX  49
#define FASTMAVLINK_MSG_ID_276_LEN  49
#define FASTMAVLINK_MSG_ID_276_CRCEXTRA  18



#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc,
    fmav_status_t* _status)
{
    fmav_camera_tracking_geo_status_t* _payload = (fmav_camera_tracking_geo_status_t*)msg->payload;

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_n = vel_n;
    _payload->vel_e = vel_e;
    _payload->vel_d = vel_d;
    _payload->vel_acc = vel_acc;
    _payload->dist = dist;
    _payload->hdg = hdg;
    _payload->hdg_acc = hdg_acc;
    _payload->tracking_status = tracking_status;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_geo_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_geo_status_pack(
        msg, sysid, compid,
        _payload->tracking_status, _payload->lat, _payload->lon, _payload->alt, _payload->h_acc, _payload->v_acc, _payload->vel_n, _payload->vel_e, _payload->vel_d, _payload->vel_acc, _payload->dist, _payload->hdg, _payload->hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc,
    fmav_status_t* _status)
{
    fmav_camera_tracking_geo_status_t* _payload = (fmav_camera_tracking_geo_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_n = vel_n;
    _payload->vel_e = vel_e;
    _payload->vel_d = vel_d;
    _payload->vel_acc = vel_acc;
    _payload->dist = dist;
    _payload->hdg = hdg;
    _payload->hdg_acc = hdg_acc;
    _payload->tracking_status = tracking_status;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_tracking_geo_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_tracking_geo_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->tracking_status, _payload->lat, _payload->lon, _payload->alt, _payload->h_acc, _payload->v_acc, _payload->vel_n, _payload->vel_e, _payload->vel_d, _payload->vel_acc, _payload->dist, _payload->hdg, _payload->hdg_acc,
        _status);
}


//----------------------------------------
//-- Message CAMERA_TRACKING_GEO_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_tracking_geo_status_decode(fmav_camera_tracking_geo_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS  276

#define mavlink_camera_tracking_geo_status_t  fmav_camera_tracking_geo_status_t

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN  49
#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN  49
#define MAVLINK_MSG_ID_276_LEN  49
#define MAVLINK_MSG_ID_276_MIN_LEN  49

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC  18
#define MAVLINK_MSG_ID_276_CRC  18




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_geo_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_tracking_geo_status_pack(
        msg, sysid, compid,
        tracking_status, lat, lon, alt, h_acc, v_acc, vel_n, vel_e, vel_d, vel_acc, dist, hdg, hdg_acc,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_tracking_geo_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
    return fmav_msg_camera_tracking_geo_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        tracking_status, lat, lon, alt, h_acc, v_acc, vel_n, vel_e, vel_d, vel_acc, dist, hdg, hdg_acc,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_tracking_geo_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_geo_status_t* payload)
{
    fmav_msg_camera_tracking_geo_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_TRACKING_GEO_STATUS_H

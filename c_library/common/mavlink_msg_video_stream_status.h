//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_video_stream_status_t {
    float framerate;
    uint32_t bitrate;
    uint16_t flags;
    uint16_t resolution_h;
    uint16_t resolution_v;
    uint16_t rotation;
    uint16_t hfov;
    uint8_t stream_id;
}) fmav_video_stream_status_t;


#define FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS  270


#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MIN  19
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN  19
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA  59

#define FASTMAVLINK_MSG_ID_270_LEN_MIN  19
#define FASTMAVLINK_MSG_ID_270_LEN_MAX  19
#define FASTMAVLINK_MSG_ID_270_LEN  19
#define FASTMAVLINK_MSG_ID_270_CRCEXTRA  59



#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov,
    fmav_status_t* _status)
{
    fmav_video_stream_status_t* _payload = (fmav_video_stream_status_t*)msg->payload;

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_status_pack(
        msg, sysid, compid,
        _payload->stream_id, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov,
    fmav_status_t* _status)
{
    fmav_video_stream_status_t* _payload = (fmav_video_stream_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->stream_id, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov,
        _status);
}


//----------------------------------------
//-- Message VIDEO_STREAM_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_video_stream_status_decode(fmav_video_stream_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS  270

#define mavlink_video_stream_status_t  fmav_video_stream_status_t

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_LEN  19
#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_MIN_LEN  19
#define MAVLINK_MSG_ID_270_LEN  19
#define MAVLINK_MSG_ID_270_MIN_LEN  19

#define MAVLINK_MSG_ID_VIDEO_STREAM_STATUS_CRC  59
#define MAVLINK_MSG_ID_270_CRC  59




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_video_stream_status_pack(
        msg, sysid, compid,
        stream_id, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov)
{
    return fmav_msg_video_stream_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        stream_id, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_video_stream_status_decode(const mavlink_message_t* msg, mavlink_video_stream_status_t* payload)
{
    fmav_msg_video_stream_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VIDEO_STREAM_STATUS_H

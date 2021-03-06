//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_H
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_H


//----------------------------------------
//-- Message VIDEO_STREAM_INFORMATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_video_stream_information_t {
    float framerate;
    uint32_t bitrate;
    uint16_t flags;
    uint16_t resolution_h;
    uint16_t resolution_v;
    uint16_t rotation;
    uint16_t hfov;
    uint8_t stream_id;
    uint8_t count;
    uint8_t type;
    char name[32];
    char uri[160];
}) fmav_video_stream_information_t;


#define FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION  269

#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX  213
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA  109

#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FLAGS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FRAME_LEN_MAX  238

#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_NUM  32 // number of elements in array
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_LEN  32 // length of array = number of bytes
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_NUM  160 // number of elements in array
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_LEN  160 // length of array = number of bytes

#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_FRAMERATE_OFS  0
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_BITRATE_OFS  4
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_FLAGS_OFS  8
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_RESOLUTION_H_OFS  10
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_RESOLUTION_V_OFS  12
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_ROTATION_OFS  14
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_HFOV_OFS  16
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_STREAM_ID_OFS  18
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_COUNT_OFS  19
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_TYPE_OFS  20
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_OFS  21
#define FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_OFS  53


//----------------------------------------
//-- Message VIDEO_STREAM_INFORMATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint8_t count, uint8_t type, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov, const char* name, const char* uri,
    fmav_status_t* _status)
{
    fmav_video_stream_information_t* _payload = (fmav_video_stream_information_t*)msg->payload;

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;
    _payload->count = count;
    _payload->type = type;
    memcpy(&(_payload->name), name, sizeof(char)*32);
    memcpy(&(_payload->uri), uri, sizeof(char)*160);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_information_pack(
        msg, sysid, compid,
        _payload->stream_id, _payload->count, _payload->type, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov, _payload->name, _payload->uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint8_t count, uint8_t type, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov, const char* name, const char* uri,
    fmav_status_t* _status)
{
    fmav_video_stream_information_t* _payload = (fmav_video_stream_information_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->framerate = framerate;
    _payload->bitrate = bitrate;
    _payload->flags = flags;
    _payload->resolution_h = resolution_h;
    _payload->resolution_v = resolution_v;
    _payload->rotation = rotation;
    _payload->hfov = hfov;
    _payload->stream_id = stream_id;
    _payload->count = count;
    _payload->type = type;
    memcpy(&(_payload->name), name, sizeof(char)*32);
    memcpy(&(_payload->uri), uri, sizeof(char)*160);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_video_stream_information_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->stream_id, _payload->count, _payload->type, _payload->flags, _payload->framerate, _payload->resolution_h, _payload->resolution_v, _payload->bitrate, _payload->rotation, _payload->hfov, _payload->name, _payload->uri,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint8_t count, uint8_t type, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov, const char* name, const char* uri,
    fmav_status_t* _status)
{
    fmav_video_stream_information_t _payload;

    _payload.framerate = framerate;
    _payload.bitrate = bitrate;
    _payload.flags = flags;
    _payload.resolution_h = resolution_h;
    _payload.resolution_v = resolution_v;
    _payload.rotation = rotation;
    _payload.hfov = hfov;
    _payload.stream_id = stream_id;
    _payload.count = count;
    _payload.type = type;
    memcpy(&(_payload.name), name, sizeof(char)*32);
    memcpy(&(_payload.uri), uri, sizeof(char)*160);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_video_stream_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message VIDEO_STREAM_INFORMATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_video_stream_information_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_video_stream_information_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_video_stream_information_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_video_stream_information_decode(fmav_video_stream_information_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_video_stream_information_get_field_framerate(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_video_stream_information_get_field_bitrate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_get_field_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_get_field_resolution_h(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_get_field_resolution_v(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_get_field_rotation(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_video_stream_information_get_field_hfov(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_video_stream_information_get_field_stream_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_video_stream_information_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_video_stream_information_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_video_stream_information_get_field_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[21]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_video_stream_information_get_field_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_NUM) return 0;
    return ((char*)&(msg->payload[21]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_video_stream_information_get_field_uri_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[53]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_video_stream_information_get_field_uri(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_NUM) return 0;
    return ((char*)&(msg->payload[53]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION  269

#define mavlink_video_stream_information_t  fmav_video_stream_information_t

#define MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION_LEN  213
#define MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION_MIN_LEN  213
#define MAVLINK_MSG_ID_269_LEN  213
#define MAVLINK_MSG_ID_269_MIN_LEN  213

#define MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION_CRC  109
#define MAVLINK_MSG_ID_269_CRC  109

#define MAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_NAME_LEN 32
#define MAVLINK_MSG_VIDEO_STREAM_INFORMATION_FIELD_URI_LEN 160


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t stream_id, uint8_t count, uint8_t type, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov, const char* name, const char* uri)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_video_stream_information_pack(
        msg, sysid, compid,
        stream_id, count, type, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov, name, uri,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_video_stream_information_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t stream_id, uint8_t count, uint8_t type, uint16_t flags, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, uint16_t hfov, const char* name, const char* uri)
{
    return fmav_msg_video_stream_information_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        stream_id, count, type, flags, framerate, resolution_h, resolution_v, bitrate, rotation, hfov, name, uri,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_video_stream_information_decode(const mavlink_message_t* msg, mavlink_video_stream_information_t* payload)
{
    fmav_msg_video_stream_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_VIDEO_STREAM_INFORMATION_H

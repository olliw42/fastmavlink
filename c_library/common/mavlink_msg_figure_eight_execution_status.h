//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_H
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_H


//----------------------------------------
//-- Message FIGURE_EIGHT_EXECUTION_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_figure_eight_execution_status_t {
    uint64_t time_usec;
    float major_radius;
    float minor_radius;
    float orientation;
    int32_t x;
    int32_t y;
    float z;
    uint8_t frame;
}) fmav_figure_eight_execution_status_t;


#define FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS  361

#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX  33
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_CRCEXTRA  93

#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FRAME_LEN_MAX  58



#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_MAJOR_RADIUS_OFS  8
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_MINOR_RADIUS_OFS  12
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_ORIENTATION_OFS  16
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_X_OFS  20
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_Y_OFS  24
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_Z_OFS  28
#define FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_FIELD_FRAME_OFS  32


//----------------------------------------
//-- Message FIGURE_EIGHT_EXECUTION_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_figure_eight_execution_status_t* _payload = (fmav_figure_eight_execution_status_t*)_msg->payload;

    _payload->time_usec = time_usec;
    _payload->major_radius = major_radius;
    _payload->minor_radius = minor_radius;
    _payload->orientation = orientation;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->frame = frame;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_figure_eight_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_figure_eight_execution_status_pack(
        _msg, sysid, compid,
        _payload->time_usec, _payload->major_radius, _payload->minor_radius, _payload->orientation, _payload->frame, _payload->x, _payload->y, _payload->z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_figure_eight_execution_status_t* _payload = (fmav_figure_eight_execution_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->major_radius = major_radius;
    _payload->minor_radius = minor_radius;
    _payload->orientation = orientation;
    _payload->x = x;
    _payload->y = y;
    _payload->z = z;
    _payload->frame = frame;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_figure_eight_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_figure_eight_execution_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_usec, _payload->major_radius, _payload->minor_radius, _payload->orientation, _payload->frame, _payload->x, _payload->y, _payload->z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z,
    fmav_status_t* _status)
{
    fmav_figure_eight_execution_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.major_radius = major_radius;
    _payload.minor_radius = minor_radius;
    _payload.orientation = orientation;
    _payload.x = x;
    _payload.y = y;
    _payload.z = z;
    _payload.frame = frame;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_figure_eight_execution_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_figure_eight_execution_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FIGURE_EIGHT_EXECUTION_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_figure_eight_execution_status_decode(fmav_figure_eight_execution_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_figure_eight_execution_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_figure_eight_execution_status_get_field_major_radius(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_figure_eight_execution_status_get_field_minor_radius(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_figure_eight_execution_status_get_field_orientation(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_figure_eight_execution_status_get_field_x(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_figure_eight_execution_status_get_field_y(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_figure_eight_execution_status_get_field_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_figure_eight_execution_status_get_field_frame(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS  361

#define mavlink_figure_eight_execution_status_t  fmav_figure_eight_execution_status_t

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_LEN  33
#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_MIN_LEN  33
#define MAVLINK_MSG_ID_361_LEN  33
#define MAVLINK_MSG_ID_361_MIN_LEN  33

#define MAVLINK_MSG_ID_FIGURE_EIGHT_EXECUTION_STATUS_CRC  93
#define MAVLINK_MSG_ID_361_CRC  93




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_figure_eight_execution_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_figure_eight_execution_status_pack(
        _msg, sysid, compid,
        time_usec, major_radius, minor_radius, orientation, frame, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_figure_eight_execution_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_figure_eight_execution_status_t* _payload)
{
    return mavlink_msg_figure_eight_execution_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_usec, _payload->major_radius, _payload->minor_radius, _payload->orientation, _payload->frame, _payload->x, _payload->y, _payload->z);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_figure_eight_execution_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, float major_radius, float minor_radius, float orientation, uint8_t frame, int32_t x, int32_t y, float z)
{
    return fmav_msg_figure_eight_execution_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_usec, major_radius, minor_radius, orientation, frame, x, y, z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_figure_eight_execution_status_decode(const mavlink_message_t* msg, mavlink_figure_eight_execution_status_t* payload)
{
    fmav_msg_figure_eight_execution_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FIGURE_EIGHT_EXECUTION_STATUS_H

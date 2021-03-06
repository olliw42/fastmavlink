//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_CAMERA_STATUS_H
#define FASTMAVLINK_MSG_CAMERA_STATUS_H


//----------------------------------------
//-- Message CAMERA_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_camera_status_t {
    uint64_t time_usec;
    float p1;
    float p2;
    float p3;
    float p4;
    uint16_t img_idx;
    uint8_t target_system;
    uint8_t cam_idx;
    uint8_t event_id;
}) fmav_camera_status_t;


#define FASTMAVLINK_MSG_ID_CAMERA_STATUS  179

#define FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX  29
#define FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA  189

#define FASTMAVLINK_MSG_CAMERA_STATUS_FLAGS  1
#define FASTMAVLINK_MSG_CAMERA_STATUS_TARGET_SYSTEM_OFS  26
#define FASTMAVLINK_MSG_CAMERA_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_CAMERA_STATUS_FRAME_LEN_MAX  54



#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_P1_OFS  8
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_P2_OFS  12
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_P3_OFS  16
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_P4_OFS  20
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_IMG_IDX_OFS  24
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_TARGET_SYSTEM_OFS  26
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_CAM_IDX_OFS  27
#define FASTMAVLINK_MSG_CAMERA_STATUS_FIELD_EVENT_ID_OFS  28


//----------------------------------------
//-- Message CAMERA_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4,
    fmav_status_t* _status)
{
    fmav_camera_status_t* _payload = (fmav_camera_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->p1 = p1;
    _payload->p2 = p2;
    _payload->p3 = p3;
    _payload->p4 = p4;
    _payload->img_idx = img_idx;
    _payload->target_system = target_system;
    _payload->cam_idx = cam_idx;
    _payload->event_id = event_id;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_CAMERA_STATUS;

    msg->target_sysid = target_system;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->event_id, _payload->p1, _payload->p2, _payload->p3, _payload->p4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4,
    fmav_status_t* _status)
{
    fmav_camera_status_t* _payload = (fmav_camera_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->p1 = p1;
    _payload->p2 = p2;
    _payload->p3 = p3;
    _payload->p4 = p4;
    _payload->img_idx = img_idx;
    _payload->target_system = target_system;
    _payload->cam_idx = cam_idx;
    _payload->event_id = event_id;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_CAMERA_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_CAMERA_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_camera_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->target_system, _payload->cam_idx, _payload->img_idx, _payload->event_id, _payload->p1, _payload->p2, _payload->p3, _payload->p4,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4,
    fmav_status_t* _status)
{
    fmav_camera_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.p1 = p1;
    _payload.p2 = p2;
    _payload.p3 = p3;
    _payload.p4 = p4;
    _payload.img_idx = img_idx;
    _payload.target_system = target_system;
    _payload.cam_idx = cam_idx;
    _payload.event_id = event_id;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_CAMERA_STATUS,
        FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_camera_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_CAMERA_STATUS,
        FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_CAMERA_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message CAMERA_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_camera_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_camera_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_camera_status_decode(fmav_camera_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_CAMERA_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_camera_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_status_get_field_p1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_status_get_field_p2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_status_get_field_p3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_camera_status_get_field_p4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_camera_status_get_field_img_idx(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_status_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_status_get_field_cam_idx(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_camera_status_get_field_event_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_CAMERA_STATUS  179

#define mavlink_camera_status_t  fmav_camera_status_t

#define MAVLINK_MSG_ID_CAMERA_STATUS_LEN  29
#define MAVLINK_MSG_ID_CAMERA_STATUS_MIN_LEN  29
#define MAVLINK_MSG_ID_179_LEN  29
#define MAVLINK_MSG_ID_179_MIN_LEN  29

#define MAVLINK_MSG_ID_CAMERA_STATUS_CRC  189
#define MAVLINK_MSG_ID_179_CRC  189




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_camera_status_pack(
        msg, sysid, compid,
        time_usec, target_system, cam_idx, img_idx, event_id, p1, p2, p3, p4,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_camera_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4)
{
    return fmav_msg_camera_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, target_system, cam_idx, img_idx, event_id, p1, p2, p3, p4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_camera_status_decode(const mavlink_message_t* msg, mavlink_camera_status_t* payload)
{
    fmav_msg_camera_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_CAMERA_STATUS_H

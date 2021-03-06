//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_H
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_H


//----------------------------------------
//-- Message OSD_PARAM_CONFIG_REPLY
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_osd_param_config_reply_t {
    uint32_t request_id;
    uint8_t result;
}) fmav_osd_param_config_reply_t;


#define FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY  11034

#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA  79

#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_FLAGS  0
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_FIELD_REQUEST_ID_OFS  0
#define FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_FIELD_RESULT_OFS  4


//----------------------------------------
//-- Message OSD_PARAM_CONFIG_REPLY packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result,
    fmav_status_t* _status)
{
    fmav_osd_param_config_reply_t* _payload = (fmav_osd_param_config_reply_t*)msg->payload;

    _payload->request_id = request_id;
    _payload->result = result;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_osd_param_config_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_osd_param_config_reply_pack(
        msg, sysid, compid,
        _payload->request_id, _payload->result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result,
    fmav_status_t* _status)
{
    fmav_osd_param_config_reply_t* _payload = (fmav_osd_param_config_reply_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->request_id = request_id;
    _payload->result = result;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_osd_param_config_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_osd_param_config_reply_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->request_id, _payload->result,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result,
    fmav_status_t* _status)
{
    fmav_osd_param_config_reply_t _payload;

    _payload.request_id = request_id;
    _payload.result = result;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_osd_param_config_reply_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_osd_param_config_reply_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OSD_PARAM_CONFIG_REPLY unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_osd_param_config_reply_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_osd_param_config_reply_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_osd_param_config_reply_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_osd_param_config_reply_decode(fmav_osd_param_config_reply_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_osd_param_config_reply_get_field_request_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_osd_param_config_reply_get_field_result(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY  11034

#define mavlink_osd_param_config_reply_t  fmav_osd_param_config_reply_t

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY_LEN  5
#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY_MIN_LEN  5
#define MAVLINK_MSG_ID_11034_LEN  5
#define MAVLINK_MSG_ID_11034_MIN_LEN  5

#define MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY_CRC  79
#define MAVLINK_MSG_ID_11034_CRC  79




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_osd_param_config_reply_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t request_id, uint8_t result)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_osd_param_config_reply_pack(
        msg, sysid, compid,
        request_id, result,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_osd_param_config_reply_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t request_id, uint8_t result)
{
    return fmav_msg_osd_param_config_reply_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        request_id, result,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_osd_param_config_reply_decode(const mavlink_message_t* msg, mavlink_osd_param_config_reply_t* payload)
{
    fmav_msg_osd_param_config_reply_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OSD_PARAM_CONFIG_REPLY_H

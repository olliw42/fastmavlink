//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_uavcan_node_status_t {
    uint64_t time_usec;
    uint32_t uptime_sec;
    uint16_t vendor_specific_status_code;
    uint8_t health;
    uint8_t mode;
    uint8_t sub_mode;
}) fmav_uavcan_node_status_t;


#define FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS  310

#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX  17
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA  28

#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FRAME_LEN_MAX  42



#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_UPTIME_SEC_OFS  8
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_VENDOR_SPECIFIC_STATUS_CODE_OFS  12
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_HEALTH_OFS  14
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_MODE_OFS  15
#define FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_FIELD_SUB_MODE_OFS  16


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code,
    fmav_status_t* _status)
{
    fmav_uavcan_node_status_t* _payload = (fmav_uavcan_node_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->vendor_specific_status_code = vendor_specific_status_code;
    _payload->health = health;
    _payload->mode = mode;
    _payload->sub_mode = sub_mode;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->health, _payload->mode, _payload->sub_mode, _payload->vendor_specific_status_code,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code,
    fmav_status_t* _status)
{
    fmav_uavcan_node_status_t* _payload = (fmav_uavcan_node_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->uptime_sec = uptime_sec;
    _payload->vendor_specific_status_code = vendor_specific_status_code;
    _payload->health = health;
    _payload->mode = mode;
    _payload->sub_mode = sub_mode;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_uavcan_node_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->uptime_sec, _payload->health, _payload->mode, _payload->sub_mode, _payload->vendor_specific_status_code,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code,
    fmav_status_t* _status)
{
    fmav_uavcan_node_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.uptime_sec = uptime_sec;
    _payload.vendor_specific_status_code = vendor_specific_status_code;
    _payload.health = health;
    _payload.mode = mode;
    _payload.sub_mode = sub_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_uavcan_node_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_UAVCAN_NODE_STATUS,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message UAVCAN_NODE_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_uavcan_node_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_uavcan_node_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavcan_node_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_uavcan_node_status_decode(fmav_uavcan_node_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_uavcan_node_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_uavcan_node_status_get_field_uptime_sec(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_uavcan_node_status_get_field_vendor_specific_status_code(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_status_get_field_health(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_status_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[15]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_uavcan_node_status_get_field_sub_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS  310

#define mavlink_uavcan_node_status_t  fmav_uavcan_node_status_t

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_LEN  17
#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_MIN_LEN  17
#define MAVLINK_MSG_ID_310_LEN  17
#define MAVLINK_MSG_ID_310_MIN_LEN  17

#define MAVLINK_MSG_ID_UAVCAN_NODE_STATUS_CRC  28
#define MAVLINK_MSG_ID_310_CRC  28




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_uavcan_node_status_pack(
        msg, sysid, compid,
        time_usec, uptime_sec, health, mode, sub_mode, vendor_specific_status_code,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_uavcan_node_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime_sec, uint8_t health, uint8_t mode, uint8_t sub_mode, uint16_t vendor_specific_status_code)
{
    return fmav_msg_uavcan_node_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, uptime_sec, health, mode, sub_mode, vendor_specific_status_code,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_uavcan_node_status_decode(const mavlink_message_t* msg, mavlink_uavcan_node_status_t* payload)
{
    fmav_msg_uavcan_node_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UAVCAN_NODE_STATUS_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MOUNT_STATUS_H
#define FASTMAVLINK_MSG_MOUNT_STATUS_H


//----------------------------------------
//-- Message MOUNT_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mount_status_t {
    int32_t pointing_a;
    int32_t pointing_b;
    int32_t pointing_c;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t mount_mode;
}) fmav_mount_status_t;


#define FASTMAVLINK_MSG_ID_MOUNT_STATUS  158

#define FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX  15
#define FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA  134

#define FASTMAVLINK_MSG_MOUNT_STATUS_FLAGS  3
#define FASTMAVLINK_MSG_MOUNT_STATUS_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_MOUNT_STATUS_TARGET_COMPONENT_OFS  13

#define FASTMAVLINK_MSG_MOUNT_STATUS_FRAME_LEN_MAX  40



#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_POINTING_A_OFS  0
#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_POINTING_B_OFS  4
#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_POINTING_C_OFS  8
#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_TARGET_SYSTEM_OFS  12
#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_TARGET_COMPONENT_OFS  13
#define FASTMAVLINK_MSG_MOUNT_STATUS_FIELD_MOUNT_MODE_OFS  14


//----------------------------------------
//-- Message MOUNT_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode,
    fmav_status_t* _status)
{
    fmav_mount_status_t* _payload = (fmav_mount_status_t*)_msg->payload;

    _payload->pointing_a = pointing_a;
    _payload->pointing_b = pointing_b;
    _payload->pointing_c = pointing_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mount_mode = mount_mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MOUNT_STATUS;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_status_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->pointing_a, _payload->pointing_b, _payload->pointing_c, _payload->mount_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode,
    fmav_status_t* _status)
{
    fmav_mount_status_t* _payload = (fmav_mount_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->pointing_a = pointing_a;
    _payload->pointing_b = pointing_b;
    _payload->pointing_c = pointing_c;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->mount_mode = mount_mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MOUNT_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mount_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->pointing_a, _payload->pointing_b, _payload->pointing_c, _payload->mount_mode,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode,
    fmav_status_t* _status)
{
    fmav_mount_status_t _payload;

    _payload.pointing_a = pointing_a;
    _payload.pointing_b = pointing_b;
    _payload.pointing_c = pointing_c;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.mount_mode = mount_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MOUNT_STATUS,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mount_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mount_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MOUNT_STATUS,
        FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MOUNT_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MOUNT_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mount_status_decode(fmav_mount_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MOUNT_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_mount_status_get_field_pointing_a(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_mount_status_get_field_pointing_b(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_mount_status_get_field_pointing_c(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mount_status_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mount_status_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mount_status_get_field_mount_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MOUNT_STATUS  158

#define mavlink_mount_status_t  fmav_mount_status_t

#define MAVLINK_MSG_ID_MOUNT_STATUS_LEN  15
#define MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN  14
#define MAVLINK_MSG_ID_158_LEN  15
#define MAVLINK_MSG_ID_158_MIN_LEN  14

#define MAVLINK_MSG_ID_MOUNT_STATUS_CRC  134
#define MAVLINK_MSG_ID_158_CRC  134




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mount_status_pack(
        _msg, sysid, compid,
        target_system, target_component, pointing_a, pointing_b, pointing_c, mount_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mount_status_t* _payload)
{
    return mavlink_msg_mount_status_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->pointing_a, _payload->pointing_b, _payload->pointing_c, _payload->mount_mode);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mount_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c, uint8_t mount_mode)
{
    return fmav_msg_mount_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, pointing_a, pointing_b, pointing_c, mount_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mount_status_decode(const mavlink_message_t* msg, mavlink_mount_status_t* payload)
{
    fmav_msg_mount_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MOUNT_STATUS_H

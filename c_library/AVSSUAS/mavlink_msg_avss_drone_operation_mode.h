//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_H
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_H


//----------------------------------------
//-- Message AVSS_DRONE_OPERATION_MODE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_avss_drone_operation_mode_t {
    uint32_t time_boot_ms;
    uint8_t M300_operation_mode;
    uint8_t horsefly_operation_mode;
}) fmav_avss_drone_operation_mode_t;


#define FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE  60053

#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX  6
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_CRCEXTRA  45

#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_FLAGS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_FRAME_LEN_MAX  31



#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_FIELD_M300_OPERATION_MODE_OFS  4
#define FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_FIELD_HORSEFLY_OPERATION_MODE_OFS  5


//----------------------------------------
//-- Message AVSS_DRONE_OPERATION_MODE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode,
    fmav_status_t* _status)
{
    fmav_avss_drone_operation_mode_t* _payload = (fmav_avss_drone_operation_mode_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->M300_operation_mode = M300_operation_mode;
    _payload->horsefly_operation_mode = horsefly_operation_mode;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_operation_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_drone_operation_mode_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->M300_operation_mode, _payload->horsefly_operation_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode,
    fmav_status_t* _status)
{
    fmav_avss_drone_operation_mode_t* _payload = (fmav_avss_drone_operation_mode_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->M300_operation_mode = M300_operation_mode;
    _payload->horsefly_operation_mode = horsefly_operation_mode;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_operation_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_drone_operation_mode_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->M300_operation_mode, _payload->horsefly_operation_mode,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode,
    fmav_status_t* _status)
{
    fmav_avss_drone_operation_mode_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.M300_operation_mode = M300_operation_mode;
    _payload.horsefly_operation_mode = horsefly_operation_mode;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_drone_operation_mode_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_drone_operation_mode_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AVSS_DRONE_OPERATION_MODE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_avss_drone_operation_mode_decode(fmav_avss_drone_operation_mode_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_avss_drone_operation_mode_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_avss_drone_operation_mode_get_field_M300_operation_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_avss_drone_operation_mode_get_field_horsefly_operation_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE  60053

#define mavlink_avss_drone_operation_mode_t  fmav_avss_drone_operation_mode_t

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_LEN  6
#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_MIN_LEN  6
#define MAVLINK_MSG_ID_60053_LEN  6
#define MAVLINK_MSG_ID_60053_MIN_LEN  6

#define MAVLINK_MSG_ID_AVSS_DRONE_OPERATION_MODE_CRC  45
#define MAVLINK_MSG_ID_60053_CRC  45




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_operation_mode_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_avss_drone_operation_mode_pack(
        _msg, sysid, compid,
        time_boot_ms, M300_operation_mode, horsefly_operation_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_operation_mode_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_avss_drone_operation_mode_t* _payload)
{
    return mavlink_msg_avss_drone_operation_mode_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->M300_operation_mode, _payload->horsefly_operation_mode);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_drone_operation_mode_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint8_t M300_operation_mode, uint8_t horsefly_operation_mode)
{
    return fmav_msg_avss_drone_operation_mode_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, M300_operation_mode, horsefly_operation_mode,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_avss_drone_operation_mode_decode(const mavlink_message_t* msg, mavlink_avss_drone_operation_mode_t* payload)
{
    fmav_msg_avss_drone_operation_mode_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AVSS_DRONE_OPERATION_MODE_H

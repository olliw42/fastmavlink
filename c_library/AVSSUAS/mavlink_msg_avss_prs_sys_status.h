//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_H
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_H


//----------------------------------------
//-- Message AVSS_PRS_SYS_STATUS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_avss_prs_sys_status_t {
    uint32_t time_boot_ms;
    uint32_t error_status;
    uint32_t battery_status;
    uint8_t arm_status;
    uint8_t charge_status;
}) fmav_avss_prs_sys_status_t;


#define FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS  60050

#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX  14
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_CRCEXTRA  220

#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FRAME_LEN_MAX  39



#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FIELD_ERROR_STATUS_OFS  4
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FIELD_BATTERY_STATUS_OFS  8
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FIELD_ARM_STATUS_OFS  12
#define FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_FIELD_CHARGE_STATUS_OFS  13


//----------------------------------------
//-- Message AVSS_PRS_SYS_STATUS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t error_status, uint32_t battery_status, uint8_t arm_status, uint8_t charge_status,
    fmav_status_t* _status)
{
    fmav_avss_prs_sys_status_t* _payload = (fmav_avss_prs_sys_status_t*)_msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->error_status = error_status;
    _payload->battery_status = battery_status;
    _payload->arm_status = arm_status;
    _payload->charge_status = charge_status;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_prs_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_prs_sys_status_pack(
        _msg, sysid, compid,
        _payload->time_boot_ms, _payload->error_status, _payload->battery_status, _payload->arm_status, _payload->charge_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t error_status, uint32_t battery_status, uint8_t arm_status, uint8_t charge_status,
    fmav_status_t* _status)
{
    fmav_avss_prs_sys_status_t* _payload = (fmav_avss_prs_sys_status_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->error_status = error_status;
    _payload->battery_status = battery_status;
    _payload->arm_status = arm_status;
    _payload->charge_status = charge_status;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_prs_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_avss_prs_sys_status_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->time_boot_ms, _payload->error_status, _payload->battery_status, _payload->arm_status, _payload->charge_status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t error_status, uint32_t battery_status, uint8_t arm_status, uint8_t charge_status,
    fmav_status_t* _status)
{
    fmav_avss_prs_sys_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.error_status = error_status;
    _payload.battery_status = battery_status;
    _payload.arm_status = arm_status;
    _payload.charge_status = charge_status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_avss_prs_sys_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_avss_prs_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AVSS_PRS_SYS_STATUS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_avss_prs_sys_status_decode(fmav_avss_prs_sys_status_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_avss_prs_sys_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_avss_prs_sys_status_get_field_error_status(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_avss_prs_sys_status_get_field_battery_status(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_avss_prs_sys_status_get_field_arm_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_avss_prs_sys_status_get_field_charge_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS  60050

#define mavlink_avss_prs_sys_status_t  fmav_avss_prs_sys_status_t

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_LEN  14
#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_MIN_LEN  14
#define MAVLINK_MSG_ID_60050_LEN  14
#define MAVLINK_MSG_ID_60050_MIN_LEN  14

#define MAVLINK_MSG_ID_AVSS_PRS_SYS_STATUS_CRC  220
#define MAVLINK_MSG_ID_60050_CRC  220




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_prs_sys_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint32_t time_boot_ms, uint32_t error_status, uint32_t battery_status, uint8_t arm_status, uint8_t charge_status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_avss_prs_sys_status_pack(
        _msg, sysid, compid,
        time_boot_ms, error_status, battery_status, arm_status, charge_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_prs_sys_status_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_avss_prs_sys_status_t* _payload)
{
    return mavlink_msg_avss_prs_sys_status_pack(
        sysid,
        compid,
        _msg,
        _payload->time_boot_ms, _payload->error_status, _payload->battery_status, _payload->arm_status, _payload->charge_status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_avss_prs_sys_status_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t error_status, uint32_t battery_status, uint8_t arm_status, uint8_t charge_status)
{
    return fmav_msg_avss_prs_sys_status_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        time_boot_ms, error_status, battery_status, arm_status, charge_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_avss_prs_sys_status_decode(const mavlink_message_t* msg, mavlink_avss_prs_sys_status_t* payload)
{
    fmav_msg_avss_prs_sys_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AVSS_PRS_SYS_STATUS_H

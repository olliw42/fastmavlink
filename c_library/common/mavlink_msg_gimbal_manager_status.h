//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gimbal_manager_status_t {
    uint32_t time_boot_ms;
    uint32_t flags;
    uint8_t gimbal_device_id;
    uint8_t primary_control_sysid;
    uint8_t primary_control_compid;
    uint8_t secondary_control_sysid;
    uint8_t secondary_control_compid;
}) fmav_gimbal_manager_status_t;


#define FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS  281

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX  13
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA  48

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FRAME_LEN_MAX  38



#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_TIME_BOOT_MS_OFS  0
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_FLAGS_OFS  4
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_GIMBAL_DEVICE_ID_OFS  8
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_PRIMARY_CONTROL_SYSID_OFS  9
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_PRIMARY_CONTROL_COMPID_OFS  10
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_SECONDARY_CONTROL_SYSID_OFS  11
#define FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_FIELD_SECONDARY_CONTROL_COMPID_OFS  12


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t* _payload = (fmav_gimbal_manager_status_t*)msg->payload;

    _payload->time_boot_ms = time_boot_ms;
    _payload->flags = flags;
    _payload->gimbal_device_id = gimbal_device_id;
    _payload->primary_control_sysid = primary_control_sysid;
    _payload->primary_control_compid = primary_control_compid;
    _payload->secondary_control_sysid = secondary_control_sysid;
    _payload->secondary_control_compid = secondary_control_compid;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_status_pack(
        msg, sysid, compid,
        _payload->time_boot_ms, _payload->flags, _payload->gimbal_device_id, _payload->primary_control_sysid, _payload->primary_control_compid, _payload->secondary_control_sysid, _payload->secondary_control_compid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t* _payload = (fmav_gimbal_manager_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_boot_ms = time_boot_ms;
    _payload->flags = flags;
    _payload->gimbal_device_id = gimbal_device_id;
    _payload->primary_control_sysid = primary_control_sysid;
    _payload->primary_control_compid = primary_control_compid;
    _payload->secondary_control_sysid = secondary_control_sysid;
    _payload->secondary_control_compid = secondary_control_compid;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gimbal_manager_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_boot_ms, _payload->flags, _payload->gimbal_device_id, _payload->primary_control_sysid, _payload->primary_control_compid, _payload->secondary_control_sysid, _payload->secondary_control_compid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid,
    fmav_status_t* _status)
{
    fmav_gimbal_manager_status_t _payload;

    _payload.time_boot_ms = time_boot_ms;
    _payload.flags = flags;
    _payload.gimbal_device_id = gimbal_device_id;
    _payload.primary_control_sysid = primary_control_sysid;
    _payload.primary_control_compid = primary_control_compid;
    _payload.secondary_control_sysid = secondary_control_sysid;
    _payload.secondary_control_compid = secondary_control_compid;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gimbal_manager_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gimbal_manager_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GIMBAL_MANAGER_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gimbal_manager_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gimbal_manager_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gimbal_manager_status_decode(fmav_gimbal_manager_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_status_get_field_time_boot_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gimbal_manager_status_get_field_flags(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_gimbal_device_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_primary_control_sysid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_primary_control_compid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_secondary_control_sysid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gimbal_manager_status_get_field_secondary_control_compid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS  281

#define mavlink_gimbal_manager_status_t  fmav_gimbal_manager_status_t

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN  13
#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_MIN_LEN  13
#define MAVLINK_MSG_ID_281_LEN  13
#define MAVLINK_MSG_ID_281_MIN_LEN  13

#define MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_CRC  48
#define MAVLINK_MSG_ID_281_CRC  48




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gimbal_manager_status_pack(
        msg, sysid, compid,
        time_boot_ms, flags, gimbal_device_id, primary_control_sysid, primary_control_compid, secondary_control_sysid, secondary_control_compid,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gimbal_manager_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t time_boot_ms, uint32_t flags, uint8_t gimbal_device_id, uint8_t primary_control_sysid, uint8_t primary_control_compid, uint8_t secondary_control_sysid, uint8_t secondary_control_compid)
{
    return fmav_msg_gimbal_manager_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_boot_ms, flags, gimbal_device_id, primary_control_sysid, primary_control_compid, secondary_control_sysid, secondary_control_compid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gimbal_manager_status_decode(const mavlink_message_t* msg, mavlink_gimbal_manager_status_t* payload)
{
    fmav_msg_gimbal_manager_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GIMBAL_MANAGER_STATUS_H

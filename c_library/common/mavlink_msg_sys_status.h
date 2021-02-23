//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SYS_STATUS_H
#define FASTMAVLINK_MSG_SYS_STATUS_H


//----------------------------------------
//-- Message SYS_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sys_status_t {
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint16_t load;
    uint16_t voltage_battery;
    int16_t current_battery;
    uint16_t drop_rate_comm;
    uint16_t errors_comm;
    uint16_t errors_count1;
    uint16_t errors_count2;
    uint16_t errors_count3;
    uint16_t errors_count4;
    int8_t battery_remaining;
}) fmav_sys_status_t;


#define FASTMAVLINK_MSG_ID_SYS_STATUS  1


#define FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MIN  31
#define FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX  31
#define FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN  31
#define FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA  124

#define FASTMAVLINK_MSG_ID_1_LEN_MIN  31
#define FASTMAVLINK_MSG_ID_1_LEN_MAX  31
#define FASTMAVLINK_MSG_ID_1_LEN  31
#define FASTMAVLINK_MSG_ID_1_CRCEXTRA  124



#define FASTMAVLINK_MSG_SYS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_SYS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SYS_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SYS_STATUS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4,
    fmav_status_t* _status)
{
    fmav_sys_status_t* _payload = (fmav_sys_status_t*)msg->payload;

    _payload->onboard_control_sensors_present = onboard_control_sensors_present;
    _payload->onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    _payload->onboard_control_sensors_health = onboard_control_sensors_health;
    _payload->load = load;
    _payload->voltage_battery = voltage_battery;
    _payload->current_battery = current_battery;
    _payload->drop_rate_comm = drop_rate_comm;
    _payload->errors_comm = errors_comm;
    _payload->errors_count1 = errors_count1;
    _payload->errors_count2 = errors_count2;
    _payload->errors_count3 = errors_count3;
    _payload->errors_count4 = errors_count4;
    _payload->battery_remaining = battery_remaining;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SYS_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sys_status_pack(
        msg, sysid, compid,
        _payload->onboard_control_sensors_present, _payload->onboard_control_sensors_enabled, _payload->onboard_control_sensors_health, _payload->load, _payload->voltage_battery, _payload->current_battery, _payload->battery_remaining, _payload->drop_rate_comm, _payload->errors_comm, _payload->errors_count1, _payload->errors_count2, _payload->errors_count3, _payload->errors_count4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4,
    fmav_status_t* _status)
{
    fmav_sys_status_t* _payload = (fmav_sys_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->onboard_control_sensors_present = onboard_control_sensors_present;
    _payload->onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    _payload->onboard_control_sensors_health = onboard_control_sensors_health;
    _payload->load = load;
    _payload->voltage_battery = voltage_battery;
    _payload->current_battery = current_battery;
    _payload->drop_rate_comm = drop_rate_comm;
    _payload->errors_comm = errors_comm;
    _payload->errors_count1 = errors_count1;
    _payload->errors_count2 = errors_count2;
    _payload->errors_count3 = errors_count3;
    _payload->errors_count4 = errors_count4;
    _payload->battery_remaining = battery_remaining;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SYS_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SYS_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SYS_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sys_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->onboard_control_sensors_present, _payload->onboard_control_sensors_enabled, _payload->onboard_control_sensors_health, _payload->load, _payload->voltage_battery, _payload->current_battery, _payload->battery_remaining, _payload->drop_rate_comm, _payload->errors_comm, _payload->errors_count1, _payload->errors_count2, _payload->errors_count3, _payload->errors_count4,
        _status);
}


//----------------------------------------
//-- Message SYS_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sys_status_decode(fmav_sys_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SYS_STATUS  1

#define mavlink_sys_status_t  fmav_sys_status_t

#define MAVLINK_MSG_ID_SYS_STATUS_LEN  31
#define MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN  31
#define MAVLINK_MSG_ID_1_LEN  31
#define MAVLINK_MSG_ID_1_MIN_LEN  31

#define MAVLINK_MSG_ID_SYS_STATUS_CRC  124
#define MAVLINK_MSG_ID_1_CRC  124




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sys_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sys_status_pack(
        msg, sysid, compid,
        onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sys_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
{
    return fmav_msg_sys_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* payload)
{
    fmav_msg_sys_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SYS_STATUS_H

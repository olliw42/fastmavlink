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

#define FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX  31
#define FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA  124

#define FASTMAVLINK_MSG_SYS_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_SYS_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SYS_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SYS_STATUS_FRAME_LEN_MAX  56



#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ONBOARD_CONTROL_SENSORS_PRESENT_OFS  0
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ONBOARD_CONTROL_SENSORS_ENABLED_OFS  4
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ONBOARD_CONTROL_SENSORS_HEALTH_OFS  8
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_LOAD_OFS  12
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_VOLTAGE_BATTERY_OFS  14
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_CURRENT_BATTERY_OFS  16
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_DROP_RATE_COMM_OFS  18
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ERRORS_COMM_OFS  20
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ERRORS_COUNT1_OFS  22
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ERRORS_COUNT2_OFS  24
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ERRORS_COUNT3_OFS  26
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_ERRORS_COUNT4_OFS  28
#define FASTMAVLINK_MSG_SYS_STATUS_FIELD_BATTERY_REMAINING_OFS  30


//----------------------------------------
//-- Message SYS_STATUS packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4,
    fmav_status_t* _status)
{
    fmav_sys_status_t _payload;

    _payload.onboard_control_sensors_present = onboard_control_sensors_present;
    _payload.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    _payload.onboard_control_sensors_health = onboard_control_sensors_health;
    _payload.load = load;
    _payload.voltage_battery = voltage_battery;
    _payload.current_battery = current_battery;
    _payload.drop_rate_comm = drop_rate_comm;
    _payload.errors_comm = errors_comm;
    _payload.errors_count1 = errors_count1;
    _payload.errors_count2 = errors_count2;
    _payload.errors_count3 = errors_count3;
    _payload.errors_count4 = errors_count4;
    _payload.battery_remaining = battery_remaining;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SYS_STATUS,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sys_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SYS_STATUS,
        FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SYS_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SYS_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_sys_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_sys_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sys_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sys_status_decode(fmav_sys_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SYS_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_sys_status_get_field_onboard_control_sensors_present(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_sys_status_get_field_onboard_control_sensors_enabled(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_sys_status_get_field_onboard_control_sensors_health(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_load(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_voltage_battery(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_sys_status_get_field_current_battery(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_drop_rate_comm(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_errors_comm(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_errors_count1(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_errors_count2(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_errors_count3(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sys_status_get_field_errors_count4(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_sys_status_get_field_battery_remaining(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(int8_t));
    return r;
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

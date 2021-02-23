//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SMART_BATTERY_INFO_H
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_H


//----------------------------------------
//-- Message SMART_BATTERY_INFO
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_smart_battery_info_t {
    int32_t capacity_full_specification;
    int32_t capacity_full;
    uint16_t cycle_count;
    uint16_t weight;
    uint16_t discharge_minimum_voltage;
    uint16_t charging_minimum_voltage;
    uint16_t resting_minimum_voltage;
    uint8_t id;
    uint8_t battery_function;
    uint8_t type;
    char serial_number[16];
    char device_name[50];
}) fmav_smart_battery_info_t;


#define FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO  370


#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MIN  87
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX  87
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN  87
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA  75

#define FASTMAVLINK_MSG_ID_370_LEN_MIN  87
#define FASTMAVLINK_MSG_ID_370_LEN_MAX  87
#define FASTMAVLINK_MSG_ID_370_LEN  87
#define FASTMAVLINK_MSG_ID_370_CRCEXTRA  75

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN  16
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN  50

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FLAGS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message SMART_BATTERY_INFO packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage,
    fmav_status_t* _status)
{
    fmav_smart_battery_info_t* _payload = (fmav_smart_battery_info_t*)msg->payload;

    _payload->capacity_full_specification = capacity_full_specification;
    _payload->capacity_full = capacity_full;
    _payload->cycle_count = cycle_count;
    _payload->weight = weight;
    _payload->discharge_minimum_voltage = discharge_minimum_voltage;
    _payload->charging_minimum_voltage = charging_minimum_voltage;
    _payload->resting_minimum_voltage = resting_minimum_voltage;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload->device_name), device_name, sizeof(char)*50);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_smart_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_smart_battery_info_pack(
        msg, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->capacity_full_specification, _payload->capacity_full, _payload->cycle_count, _payload->serial_number, _payload->device_name, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage,
    fmav_status_t* _status)
{
    fmav_smart_battery_info_t* _payload = (fmav_smart_battery_info_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->capacity_full_specification = capacity_full_specification;
    _payload->capacity_full = capacity_full;
    _payload->cycle_count = cycle_count;
    _payload->weight = weight;
    _payload->discharge_minimum_voltage = discharge_minimum_voltage;
    _payload->charging_minimum_voltage = charging_minimum_voltage;
    _payload->resting_minimum_voltage = resting_minimum_voltage;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload->device_name), device_name, sizeof(char)*50);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_smart_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_smart_battery_info_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->capacity_full_specification, _payload->capacity_full, _payload->cycle_count, _payload->serial_number, _payload->device_name, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage,
        _status);
}


//----------------------------------------
//-- Message SMART_BATTERY_INFO unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_smart_battery_info_decode(fmav_smart_battery_info_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO  370

#define mavlink_smart_battery_info_t  fmav_smart_battery_info_t

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN  87
#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN  87
#define MAVLINK_MSG_ID_370_LEN  87
#define MAVLINK_MSG_ID_370_MIN_LEN  87

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC  75
#define MAVLINK_MSG_ID_370_CRC  75

#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN 16
#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN 50


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_smart_battery_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_smart_battery_info_pack(
        msg, sysid, compid,
        id, battery_function, type, capacity_full_specification, capacity_full, cycle_count, serial_number, device_name, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_smart_battery_info_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage)
{
    return fmav_msg_smart_battery_info_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        id, battery_function, type, capacity_full_specification, capacity_full, cycle_count, serial_number, device_name, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_smart_battery_info_decode(const mavlink_message_t* msg, mavlink_smart_battery_info_t* payload)
{
    fmav_msg_smart_battery_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SMART_BATTERY_INFO_H

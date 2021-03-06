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

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX  87
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA  75

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FLAGS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FRAME_LEN_MAX  112

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_NUM  50 // number of elements in array
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN  50 // length of array = number of bytes

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CAPACITY_FULL_SPECIFICATION_OFS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CAPACITY_FULL_OFS  4
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CYCLE_COUNT_OFS  8
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_WEIGHT_OFS  10
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DISCHARGE_MINIMUM_VOLTAGE_OFS  12
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CHARGING_MINIMUM_VOLTAGE_OFS  14
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_RESTING_MINIMUM_VOLTAGE_OFS  16
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_ID_OFS  18
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_BATTERY_FUNCTION_OFS  19
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_TYPE_OFS  20
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_OFS  21
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_OFS  37


//----------------------------------------
//-- Message SMART_BATTERY_INFO packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage,
    fmav_status_t* _status)
{
    fmav_smart_battery_info_t _payload;

    _payload.capacity_full_specification = capacity_full_specification;
    _payload.capacity_full = capacity_full;
    _payload.cycle_count = cycle_count;
    _payload.weight = weight;
    _payload.discharge_minimum_voltage = discharge_minimum_voltage;
    _payload.charging_minimum_voltage = charging_minimum_voltage;
    _payload.resting_minimum_voltage = resting_minimum_voltage;
    _payload.id = id;
    _payload.battery_function = battery_function;
    _payload.type = type;
    memcpy(&(_payload.serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload.device_name), device_name, sizeof(char)*50);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_smart_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SMART_BATTERY_INFO unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_smart_battery_info_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_smart_battery_info_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_smart_battery_info_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_smart_battery_info_decode(fmav_smart_battery_info_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_smart_battery_info_get_field_capacity_full_specification(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_smart_battery_info_get_field_capacity_full(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_cycle_count(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_weight(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_discharge_minimum_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_charging_minimum_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_resting_minimum_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_smart_battery_info_get_field_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_smart_battery_info_get_field_battery_function(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[19]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_smart_battery_info_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_smart_battery_info_get_field_serial_number_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[21]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_smart_battery_info_get_field_serial_number(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_NUM) return 0;
    return ((char*)&(msg->payload[21]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_smart_battery_info_get_field_device_name_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[37]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_smart_battery_info_get_field_device_name(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_NUM) return 0;
    return ((char*)&(msg->payload[37]))[index];
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

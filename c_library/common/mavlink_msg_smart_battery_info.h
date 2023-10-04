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

// fields are ordered, as they appear on the wire
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
    uint16_t charging_maximum_voltage;
    uint8_t cells_in_series;
    uint32_t discharge_maximum_current;
    uint32_t discharge_maximum_burst_current;
    char manufacture_date[11];
}) fmav_smart_battery_info_t;


#define FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO  370

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX  109
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA  75

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FLAGS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FRAME_LEN_MAX  134

#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_NUM  16 // number of elements in array
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_NUM  50 // number of elements in array
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN  50 // length of array = number of bytes
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_NUM  11 // number of elements in array
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN  11 // length of array = number of bytes

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
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CHARGING_MAXIMUM_VOLTAGE_OFS  87
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_CELLS_IN_SERIES_OFS  89
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DISCHARGE_MAXIMUM_CURRENT_OFS  90
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DISCHARGE_MAXIMUM_BURST_CURRENT_OFS  94
#define FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_OFS  98


//----------------------------------------
//-- Message SMART_BATTERY_INFO pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char* manufacture_date,
    fmav_status_t* _status)
{
    fmav_smart_battery_info_t* _payload = (fmav_smart_battery_info_t*)_msg->payload;

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
    _payload->charging_maximum_voltage = charging_maximum_voltage;
    _payload->cells_in_series = cells_in_series;
    _payload->discharge_maximum_current = discharge_maximum_current;
    _payload->discharge_maximum_burst_current = discharge_maximum_burst_current;
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload->device_name), device_name, sizeof(char)*50);
    memcpy(&(_payload->manufacture_date), manufacture_date, sizeof(char)*11);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_smart_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_smart_battery_info_pack(
        _msg, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->capacity_full_specification, _payload->capacity_full, _payload->cycle_count, _payload->serial_number, _payload->device_name, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->cells_in_series, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->manufacture_date,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char* manufacture_date,
    fmav_status_t* _status)
{
    fmav_smart_battery_info_t* _payload = (fmav_smart_battery_info_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

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
    _payload->charging_maximum_voltage = charging_maximum_voltage;
    _payload->cells_in_series = cells_in_series;
    _payload->discharge_maximum_current = discharge_maximum_current;
    _payload->discharge_maximum_burst_current = discharge_maximum_burst_current;
    memcpy(&(_payload->serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload->device_name), device_name, sizeof(char)*50);
    memcpy(&(_payload->manufacture_date), manufacture_date, sizeof(char)*11);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SMART_BATTERY_INFO >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SMART_BATTERY_INFO_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_smart_battery_info_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_smart_battery_info_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->capacity_full_specification, _payload->capacity_full, _payload->cycle_count, _payload->serial_number, _payload->device_name, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->cells_in_series, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->manufacture_date,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char* manufacture_date,
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
    _payload.charging_maximum_voltage = charging_maximum_voltage;
    _payload.cells_in_series = cells_in_series;
    _payload.discharge_maximum_current = discharge_maximum_current;
    _payload.discharge_maximum_burst_current = discharge_maximum_burst_current;
    memcpy(&(_payload.serial_number), serial_number, sizeof(char)*16);
    memcpy(&(_payload.device_name), device_name, sizeof(char)*50);
    memcpy(&(_payload.manufacture_date), manufacture_date, sizeof(char)*11);

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
//-- Message SMART_BATTERY_INFO decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_smart_battery_info_decode(fmav_smart_battery_info_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SMART_BATTERY_INFO_PAYLOAD_LEN_MAX);
#endif
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


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_smart_battery_info_get_field_charging_maximum_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[87]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_smart_battery_info_get_field_cells_in_series(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[89]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_smart_battery_info_get_field_discharge_maximum_current(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[90]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_smart_battery_info_get_field_discharge_maximum_burst_current(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[94]), sizeof(uint32_t));
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


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_smart_battery_info_get_field_manufacture_date_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[98]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_smart_battery_info_get_field_manufacture_date(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_NUM) return 0;
    return ((char*)&(msg->payload[98]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO  370

#define mavlink_smart_battery_info_t  fmav_smart_battery_info_t

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN  109
#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN  87
#define MAVLINK_MSG_ID_370_LEN  109
#define MAVLINK_MSG_ID_370_MIN_LEN  87

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC  75
#define MAVLINK_MSG_ID_370_CRC  75

#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN 16
#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN 50
#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN 11


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_smart_battery_info_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char* manufacture_date)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_smart_battery_info_pack(
        _msg, sysid, compid,
        id, battery_function, type, capacity_full_specification, capacity_full, cycle_count, serial_number, device_name, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage, charging_maximum_voltage, cells_in_series, discharge_maximum_current, discharge_maximum_burst_current, manufacture_date,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_smart_battery_info_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_smart_battery_info_t* _payload)
{
    return mavlink_msg_smart_battery_info_pack(
        sysid,
        compid,
        _msg,
        _payload->id, _payload->battery_function, _payload->type, _payload->capacity_full_specification, _payload->capacity_full, _payload->cycle_count, _payload->serial_number, _payload->device_name, _payload->weight, _payload->discharge_minimum_voltage, _payload->charging_minimum_voltage, _payload->resting_minimum_voltage, _payload->charging_maximum_voltage, _payload->cells_in_series, _payload->discharge_maximum_current, _payload->discharge_maximum_burst_current, _payload->manufacture_date);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_smart_battery_info_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char* serial_number, const char* device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char* manufacture_date)
{
    return fmav_msg_smart_battery_info_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        id, battery_function, type, capacity_full_specification, capacity_full, cycle_count, serial_number, device_name, weight, discharge_minimum_voltage, charging_minimum_voltage, resting_minimum_voltage, charging_maximum_voltage, cells_in_series, discharge_maximum_current, discharge_maximum_burst_current, manufacture_date,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_smart_battery_info_decode(const mavlink_message_t* msg, mavlink_smart_battery_info_t* payload)
{
    fmav_msg_smart_battery_info_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SMART_BATTERY_INFO_H

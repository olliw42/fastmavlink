//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_BATTERY_STATUS_H
#define FASTMAVLINK_MSG_BATTERY_STATUS_H


//----------------------------------------
//-- Message BATTERY_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_battery_status_t {
    int32_t current_consumed;
    int32_t energy_consumed;
    int16_t temperature;
    uint16_t voltages[10];
    int16_t current_battery;
    uint8_t id;
    uint8_t battery_function;
    uint8_t type;
    int8_t battery_remaining;
    int32_t time_remaining;
    uint8_t charge_state;
    uint16_t voltages_ext[4];
    uint8_t mode;
    uint32_t fault_bitmask;
}) fmav_battery_status_t;


#define FASTMAVLINK_MSG_ID_BATTERY_STATUS  147


#define FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MIN  36
#define FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX  54
#define FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN  54
#define FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA  154

#define FASTMAVLINK_MSG_ID_147_LEN_MIN  36
#define FASTMAVLINK_MSG_ID_147_LEN_MAX  54
#define FASTMAVLINK_MSG_ID_147_LEN  54
#define FASTMAVLINK_MSG_ID_147_CRCEXTRA  154

#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN  10
#define FASTMAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN  4

#define FASTMAVLINK_MSG_BATTERY_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_BATTERY_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_BATTERY_STATUS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_147_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_147_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message BATTERY_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t* _payload = (fmav_battery_status_t*)msg->payload;

    _payload->current_consumed = current_consumed;
    _payload->energy_consumed = energy_consumed;
    _payload->temperature = temperature;
    _payload->current_battery = current_battery;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->battery_remaining = battery_remaining;
    _payload->time_remaining = time_remaining;
    _payload->charge_state = charge_state;
    _payload->mode = mode;
    _payload->fault_bitmask = fault_bitmask;
    memcpy(&(_payload->voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload->voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_BATTERY_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_pack(
        msg, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->temperature, _payload->voltages, _payload->current_battery, _payload->current_consumed, _payload->energy_consumed, _payload->battery_remaining, _payload->time_remaining, _payload->charge_state, _payload->voltages_ext, _payload->mode, _payload->fault_bitmask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t* _payload = (fmav_battery_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->current_consumed = current_consumed;
    _payload->energy_consumed = energy_consumed;
    _payload->temperature = temperature;
    _payload->current_battery = current_battery;
    _payload->id = id;
    _payload->battery_function = battery_function;
    _payload->type = type;
    _payload->battery_remaining = battery_remaining;
    _payload->time_remaining = time_remaining;
    _payload->charge_state = charge_state;
    _payload->mode = mode;
    _payload->fault_bitmask = fault_bitmask;
    memcpy(&(_payload->voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload->voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_BATTERY_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_battery_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->id, _payload->battery_function, _payload->type, _payload->temperature, _payload->voltages, _payload->current_battery, _payload->current_consumed, _payload->energy_consumed, _payload->battery_remaining, _payload->time_remaining, _payload->charge_state, _payload->voltages_ext, _payload->mode, _payload->fault_bitmask,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask,
    fmav_status_t* _status)
{
    fmav_battery_status_t _payload;

    _payload.current_consumed = current_consumed;
    _payload.energy_consumed = energy_consumed;
    _payload.temperature = temperature;
    _payload.current_battery = current_battery;
    _payload.id = id;
    _payload.battery_function = battery_function;
    _payload.type = type;
    _payload.battery_remaining = battery_remaining;
    _payload.time_remaining = time_remaining;
    _payload.charge_state = charge_state;
    _payload.mode = mode;
    _payload.fault_bitmask = fault_bitmask;
    memcpy(&(_payload.voltages), voltages, sizeof(uint16_t)*10);
    memcpy(&(_payload.voltages_ext), voltages_ext, sizeof(uint16_t)*4);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_battery_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_battery_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_BATTERY_STATUS,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_BATTERY_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message BATTERY_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_battery_status_decode(fmav_battery_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_BATTERY_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_BATTERY_STATUS  147

#define mavlink_battery_status_t  fmav_battery_status_t

#define MAVLINK_MSG_ID_BATTERY_STATUS_LEN  54
#define MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN  36
#define MAVLINK_MSG_ID_147_LEN  54
#define MAVLINK_MSG_ID_147_MIN_LEN  36

#define MAVLINK_MSG_ID_BATTERY_STATUS_CRC  154
#define MAVLINK_MSG_ID_147_CRC  154

#define MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN 10
#define MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN 4


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_battery_status_pack(
        msg, sysid, compid,
        id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state, voltages_ext, mode, fault_bitmask,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_battery_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t* voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state, const uint16_t* voltages_ext, uint8_t mode, uint32_t fault_bitmask)
{
    return fmav_msg_battery_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state, voltages_ext, mode, fault_bitmask,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_battery_status_decode(const mavlink_message_t* msg, mavlink_battery_status_t* payload)
{
    fmav_msg_battery_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_BATTERY_STATUS_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GENERATOR_STATUS_H
#define FASTMAVLINK_MSG_GENERATOR_STATUS_H


//----------------------------------------
//-- Message GENERATOR_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_generator_status_t {
    uint64_t status;
    float battery_current;
    float load_current;
    float power_generated;
    float bus_voltage;
    float bat_current_setpoint;
    uint32_t runtime;
    int32_t time_until_maintenance;
    uint16_t generator_speed;
    int16_t rectifier_temperature;
    int16_t generator_temperature;
}) fmav_generator_status_t;


#define FASTMAVLINK_MSG_ID_GENERATOR_STATUS  373


#define FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MIN  42
#define FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX  42
#define FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN  42
#define FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA  117

#define FASTMAVLINK_MSG_ID_373_LEN_MIN  42
#define FASTMAVLINK_MSG_ID_373_LEN_MAX  42
#define FASTMAVLINK_MSG_ID_373_LEN  42
#define FASTMAVLINK_MSG_ID_373_CRCEXTRA  117



#define FASTMAVLINK_MSG_GENERATOR_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_GENERATOR_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GENERATOR_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message GENERATOR_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance,
    fmav_status_t* _status)
{
    fmav_generator_status_t* _payload = (fmav_generator_status_t*)msg->payload;

    _payload->status = status;
    _payload->battery_current = battery_current;
    _payload->load_current = load_current;
    _payload->power_generated = power_generated;
    _payload->bus_voltage = bus_voltage;
    _payload->bat_current_setpoint = bat_current_setpoint;
    _payload->runtime = runtime;
    _payload->time_until_maintenance = time_until_maintenance;
    _payload->generator_speed = generator_speed;
    _payload->rectifier_temperature = rectifier_temperature;
    _payload->generator_temperature = generator_temperature;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GENERATOR_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_generator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_generator_status_pack(
        msg, sysid, compid,
        _payload->status, _payload->generator_speed, _payload->battery_current, _payload->load_current, _payload->power_generated, _payload->bus_voltage, _payload->rectifier_temperature, _payload->bat_current_setpoint, _payload->generator_temperature, _payload->runtime, _payload->time_until_maintenance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance,
    fmav_status_t* _status)
{
    fmav_generator_status_t* _payload = (fmav_generator_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->status = status;
    _payload->battery_current = battery_current;
    _payload->load_current = load_current;
    _payload->power_generated = power_generated;
    _payload->bus_voltage = bus_voltage;
    _payload->bat_current_setpoint = bat_current_setpoint;
    _payload->runtime = runtime;
    _payload->time_until_maintenance = time_until_maintenance;
    _payload->generator_speed = generator_speed;
    _payload->rectifier_temperature = rectifier_temperature;
    _payload->generator_temperature = generator_temperature;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GENERATOR_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GENERATOR_STATUS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_generator_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_generator_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_generator_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->status, _payload->generator_speed, _payload->battery_current, _payload->load_current, _payload->power_generated, _payload->bus_voltage, _payload->rectifier_temperature, _payload->bat_current_setpoint, _payload->generator_temperature, _payload->runtime, _payload->time_until_maintenance,
        _status);
}


//----------------------------------------
//-- Message GENERATOR_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_generator_status_decode(fmav_generator_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_GENERATOR_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GENERATOR_STATUS  373

#define mavlink_generator_status_t  fmav_generator_status_t

#define MAVLINK_MSG_ID_GENERATOR_STATUS_LEN  42
#define MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN  42
#define MAVLINK_MSG_ID_373_LEN  42
#define MAVLINK_MSG_ID_373_MIN_LEN  42

#define MAVLINK_MSG_ID_GENERATOR_STATUS_CRC  117
#define MAVLINK_MSG_ID_373_CRC  117




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_generator_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_generator_status_pack(
        msg, sysid, compid,
        status, generator_speed, battery_current, load_current, power_generated, bus_voltage, rectifier_temperature, bat_current_setpoint, generator_temperature, runtime, time_until_maintenance,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_generator_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
    return fmav_msg_generator_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        status, generator_speed, battery_current, load_current, power_generated, bus_voltage, rectifier_temperature, bat_current_setpoint, generator_temperature, runtime, time_until_maintenance,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_generator_status_decode(const mavlink_message_t* msg, mavlink_generator_status_t* payload)
{
    fmav_msg_generator_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GENERATOR_STATUS_H

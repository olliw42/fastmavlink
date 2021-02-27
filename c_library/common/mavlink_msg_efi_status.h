//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_EFI_STATUS_H
#define FASTMAVLINK_MSG_EFI_STATUS_H


//----------------------------------------
//-- Message EFI_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_efi_status_t {
    float ecu_index;
    float rpm;
    float fuel_consumed;
    float fuel_flow;
    float engine_load;
    float throttle_position;
    float spark_dwell_time;
    float barometric_pressure;
    float intake_manifold_pressure;
    float intake_manifold_temperature;
    float cylinder_head_temperature;
    float ignition_timing;
    float injection_time;
    float exhaust_gas_temperature;
    float throttle_out;
    float pt_compensation;
    uint8_t health;
}) fmav_efi_status_t;


#define FASTMAVLINK_MSG_ID_EFI_STATUS  225


#define FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MIN  65
#define FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX  65
#define FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN  65
#define FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA  208

#define FASTMAVLINK_MSG_ID_225_LEN_MIN  65
#define FASTMAVLINK_MSG_ID_225_LEN_MAX  65
#define FASTMAVLINK_MSG_ID_225_LEN  65
#define FASTMAVLINK_MSG_ID_225_CRCEXTRA  208



#define FASTMAVLINK_MSG_EFI_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_EFI_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_EFI_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_EFI_STATUS_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)
#define FASTMAVLINK_MSG_ID_225_FRAME_LEN_MAX  (FASTMAVLINK_HEADER_V2_LEN+FASTMAVLINK_MSG_ID_225_PAYLOAD_LEN_MAX+FASTMAVLINK_CHECKSUM_LEN+FASTMAVLINK_SIGNATURE_LEN)


//----------------------------------------
//-- Message EFI_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation,
    fmav_status_t* _status)
{
    fmav_efi_status_t* _payload = (fmav_efi_status_t*)msg->payload;

    _payload->ecu_index = ecu_index;
    _payload->rpm = rpm;
    _payload->fuel_consumed = fuel_consumed;
    _payload->fuel_flow = fuel_flow;
    _payload->engine_load = engine_load;
    _payload->throttle_position = throttle_position;
    _payload->spark_dwell_time = spark_dwell_time;
    _payload->barometric_pressure = barometric_pressure;
    _payload->intake_manifold_pressure = intake_manifold_pressure;
    _payload->intake_manifold_temperature = intake_manifold_temperature;
    _payload->cylinder_head_temperature = cylinder_head_temperature;
    _payload->ignition_timing = ignition_timing;
    _payload->injection_time = injection_time;
    _payload->exhaust_gas_temperature = exhaust_gas_temperature;
    _payload->throttle_out = throttle_out;
    _payload->pt_compensation = pt_compensation;
    _payload->health = health;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_EFI_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_efi_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_efi_status_pack(
        msg, sysid, compid,
        _payload->health, _payload->ecu_index, _payload->rpm, _payload->fuel_consumed, _payload->fuel_flow, _payload->engine_load, _payload->throttle_position, _payload->spark_dwell_time, _payload->barometric_pressure, _payload->intake_manifold_pressure, _payload->intake_manifold_temperature, _payload->cylinder_head_temperature, _payload->ignition_timing, _payload->injection_time, _payload->exhaust_gas_temperature, _payload->throttle_out, _payload->pt_compensation,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation,
    fmav_status_t* _status)
{
    fmav_efi_status_t* _payload = (fmav_efi_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->ecu_index = ecu_index;
    _payload->rpm = rpm;
    _payload->fuel_consumed = fuel_consumed;
    _payload->fuel_flow = fuel_flow;
    _payload->engine_load = engine_load;
    _payload->throttle_position = throttle_position;
    _payload->spark_dwell_time = spark_dwell_time;
    _payload->barometric_pressure = barometric_pressure;
    _payload->intake_manifold_pressure = intake_manifold_pressure;
    _payload->intake_manifold_temperature = intake_manifold_temperature;
    _payload->cylinder_head_temperature = cylinder_head_temperature;
    _payload->ignition_timing = ignition_timing;
    _payload->injection_time = injection_time;
    _payload->exhaust_gas_temperature = exhaust_gas_temperature;
    _payload->throttle_out = throttle_out;
    _payload->pt_compensation = pt_compensation;
    _payload->health = health;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_EFI_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_EFI_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_EFI_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_efi_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_efi_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->health, _payload->ecu_index, _payload->rpm, _payload->fuel_consumed, _payload->fuel_flow, _payload->engine_load, _payload->throttle_position, _payload->spark_dwell_time, _payload->barometric_pressure, _payload->intake_manifold_pressure, _payload->intake_manifold_temperature, _payload->cylinder_head_temperature, _payload->ignition_timing, _payload->injection_time, _payload->exhaust_gas_temperature, _payload->throttle_out, _payload->pt_compensation,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation,
    fmav_status_t* _status)
{
    fmav_efi_status_t _payload;

    _payload.ecu_index = ecu_index;
    _payload.rpm = rpm;
    _payload.fuel_consumed = fuel_consumed;
    _payload.fuel_flow = fuel_flow;
    _payload.engine_load = engine_load;
    _payload.throttle_position = throttle_position;
    _payload.spark_dwell_time = spark_dwell_time;
    _payload.barometric_pressure = barometric_pressure;
    _payload.intake_manifold_pressure = intake_manifold_pressure;
    _payload.intake_manifold_temperature = intake_manifold_temperature;
    _payload.cylinder_head_temperature = cylinder_head_temperature;
    _payload.ignition_timing = ignition_timing;
    _payload.injection_time = injection_time;
    _payload.exhaust_gas_temperature = exhaust_gas_temperature;
    _payload.throttle_out = throttle_out;
    _payload.pt_compensation = pt_compensation;
    _payload.health = health;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_EFI_STATUS,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_efi_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_efi_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_EFI_STATUS,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_EFI_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message EFI_STATUS unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_efi_status_decode(fmav_efi_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_EFI_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_EFI_STATUS  225

#define mavlink_efi_status_t  fmav_efi_status_t

#define MAVLINK_MSG_ID_EFI_STATUS_LEN  65
#define MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN  65
#define MAVLINK_MSG_ID_225_LEN  65
#define MAVLINK_MSG_ID_225_MIN_LEN  65

#define MAVLINK_MSG_ID_EFI_STATUS_CRC  208
#define MAVLINK_MSG_ID_225_CRC  208




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_efi_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_efi_status_pack(
        msg, sysid, compid,
        health, ecu_index, rpm, fuel_consumed, fuel_flow, engine_load, throttle_position, spark_dwell_time, barometric_pressure, intake_manifold_pressure, intake_manifold_temperature, cylinder_head_temperature, ignition_timing, injection_time, exhaust_gas_temperature, throttle_out, pt_compensation,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_efi_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation)
{
    return fmav_msg_efi_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        health, ecu_index, rpm, fuel_consumed, fuel_flow, engine_load, throttle_position, spark_dwell_time, barometric_pressure, intake_manifold_pressure, intake_manifold_temperature, cylinder_head_temperature, ignition_timing, injection_time, exhaust_gas_temperature, throttle_out, pt_compensation,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_efi_status_decode(const mavlink_message_t* msg, mavlink_efi_status_t* payload)
{
    fmav_msg_efi_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_EFI_STATUS_H

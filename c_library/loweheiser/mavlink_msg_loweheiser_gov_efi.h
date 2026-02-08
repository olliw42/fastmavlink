//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_H
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_H


//----------------------------------------
//-- Message LOWEHEISER_GOV_EFI
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_loweheiser_gov_efi_t {
    float volt_batt;
    float curr_batt;
    float curr_gen;
    float curr_rot;
    float fuel_level;
    float throttle;
    uint32_t runtime;
    int32_t until_maintenance;
    float rectifier_temp;
    float generator_temp;
    float efi_batt;
    float efi_rpm;
    float efi_pw;
    float efi_fuel_flow;
    float efi_fuel_consumed;
    float efi_baro;
    float efi_mat;
    float efi_clt;
    float efi_tps;
    float efi_exhaust_gas_temperature;
    uint16_t generator_status;
    uint16_t efi_status;
    uint8_t efi_index;
}) fmav_loweheiser_gov_efi_t;


#define FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI  10151

#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX  85
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_CRCEXTRA  195

#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FLAGS  0
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FRAME_LEN_MAX  110



#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_VOLT_BATT_OFS  0
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_CURR_BATT_OFS  4
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_CURR_GEN_OFS  8
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_CURR_ROT_OFS  12
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_FUEL_LEVEL_OFS  16
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_THROTTLE_OFS  20
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_RUNTIME_OFS  24
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_UNTIL_MAINTENANCE_OFS  28
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_RECTIFIER_TEMP_OFS  32
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_GENERATOR_TEMP_OFS  36
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_BATT_OFS  40
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_RPM_OFS  44
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_PW_OFS  48
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_FUEL_FLOW_OFS  52
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_FUEL_CONSUMED_OFS  56
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_BARO_OFS  60
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_MAT_OFS  64
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_CLT_OFS  68
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_TPS_OFS  72
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_EXHAUST_GAS_TEMPERATURE_OFS  76
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_GENERATOR_STATUS_OFS  80
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_STATUS_OFS  82
#define FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_FIELD_EFI_INDEX_OFS  84


//----------------------------------------
//-- Message LOWEHEISER_GOV_EFI pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status,
    fmav_status_t* _status)
{
    fmav_loweheiser_gov_efi_t* _payload = (fmav_loweheiser_gov_efi_t*)_msg->payload;

    _payload->volt_batt = volt_batt;
    _payload->curr_batt = curr_batt;
    _payload->curr_gen = curr_gen;
    _payload->curr_rot = curr_rot;
    _payload->fuel_level = fuel_level;
    _payload->throttle = throttle;
    _payload->runtime = runtime;
    _payload->until_maintenance = until_maintenance;
    _payload->rectifier_temp = rectifier_temp;
    _payload->generator_temp = generator_temp;
    _payload->efi_batt = efi_batt;
    _payload->efi_rpm = efi_rpm;
    _payload->efi_pw = efi_pw;
    _payload->efi_fuel_flow = efi_fuel_flow;
    _payload->efi_fuel_consumed = efi_fuel_consumed;
    _payload->efi_baro = efi_baro;
    _payload->efi_mat = efi_mat;
    _payload->efi_clt = efi_clt;
    _payload->efi_tps = efi_tps;
    _payload->efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    _payload->generator_status = generator_status;
    _payload->efi_status = efi_status;
    _payload->efi_index = efi_index;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_loweheiser_gov_efi_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_loweheiser_gov_efi_pack(
        _msg, sysid, compid,
        _payload->volt_batt, _payload->curr_batt, _payload->curr_gen, _payload->curr_rot, _payload->fuel_level, _payload->throttle, _payload->runtime, _payload->until_maintenance, _payload->rectifier_temp, _payload->generator_temp, _payload->efi_batt, _payload->efi_rpm, _payload->efi_pw, _payload->efi_fuel_flow, _payload->efi_fuel_consumed, _payload->efi_baro, _payload->efi_mat, _payload->efi_clt, _payload->efi_tps, _payload->efi_exhaust_gas_temperature, _payload->efi_index, _payload->generator_status, _payload->efi_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status,
    fmav_status_t* _status)
{
    fmav_loweheiser_gov_efi_t* _payload = (fmav_loweheiser_gov_efi_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->volt_batt = volt_batt;
    _payload->curr_batt = curr_batt;
    _payload->curr_gen = curr_gen;
    _payload->curr_rot = curr_rot;
    _payload->fuel_level = fuel_level;
    _payload->throttle = throttle;
    _payload->runtime = runtime;
    _payload->until_maintenance = until_maintenance;
    _payload->rectifier_temp = rectifier_temp;
    _payload->generator_temp = generator_temp;
    _payload->efi_batt = efi_batt;
    _payload->efi_rpm = efi_rpm;
    _payload->efi_pw = efi_pw;
    _payload->efi_fuel_flow = efi_fuel_flow;
    _payload->efi_fuel_consumed = efi_fuel_consumed;
    _payload->efi_baro = efi_baro;
    _payload->efi_mat = efi_mat;
    _payload->efi_clt = efi_clt;
    _payload->efi_tps = efi_tps;
    _payload->efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    _payload->generator_status = generator_status;
    _payload->efi_status = efi_status;
    _payload->efi_index = efi_index;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_loweheiser_gov_efi_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_loweheiser_gov_efi_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->volt_batt, _payload->curr_batt, _payload->curr_gen, _payload->curr_rot, _payload->fuel_level, _payload->throttle, _payload->runtime, _payload->until_maintenance, _payload->rectifier_temp, _payload->generator_temp, _payload->efi_batt, _payload->efi_rpm, _payload->efi_pw, _payload->efi_fuel_flow, _payload->efi_fuel_consumed, _payload->efi_baro, _payload->efi_mat, _payload->efi_clt, _payload->efi_tps, _payload->efi_exhaust_gas_temperature, _payload->efi_index, _payload->generator_status, _payload->efi_status,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status,
    fmav_status_t* _status)
{
    fmav_loweheiser_gov_efi_t _payload;

    _payload.volt_batt = volt_batt;
    _payload.curr_batt = curr_batt;
    _payload.curr_gen = curr_gen;
    _payload.curr_rot = curr_rot;
    _payload.fuel_level = fuel_level;
    _payload.throttle = throttle;
    _payload.runtime = runtime;
    _payload.until_maintenance = until_maintenance;
    _payload.rectifier_temp = rectifier_temp;
    _payload.generator_temp = generator_temp;
    _payload.efi_batt = efi_batt;
    _payload.efi_rpm = efi_rpm;
    _payload.efi_pw = efi_pw;
    _payload.efi_fuel_flow = efi_fuel_flow;
    _payload.efi_fuel_consumed = efi_fuel_consumed;
    _payload.efi_baro = efi_baro;
    _payload.efi_mat = efi_mat;
    _payload.efi_clt = efi_clt;
    _payload.efi_tps = efi_tps;
    _payload.efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    _payload.generator_status = generator_status;
    _payload.efi_status = efi_status;
    _payload.efi_index = efi_index;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_loweheiser_gov_efi_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_LOWEHEISER_GOV_EFI,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message LOWEHEISER_GOV_EFI decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_loweheiser_gov_efi_decode(fmav_loweheiser_gov_efi_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_volt_batt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_curr_batt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_curr_gen(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_curr_rot(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_fuel_level(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_throttle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_loweheiser_gov_efi_get_field_runtime(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_loweheiser_gov_efi_get_field_until_maintenance(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_rectifier_temp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_generator_temp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_batt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_rpm(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_pw(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_fuel_flow(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_fuel_consumed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_baro(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_mat(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_clt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_tps(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_loweheiser_gov_efi_get_field_efi_exhaust_gas_temperature(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[76]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_get_field_generator_status(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[80]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_loweheiser_gov_efi_get_field_efi_status(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[82]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_loweheiser_gov_efi_get_field_efi_index(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[84]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI  10151

#define mavlink_loweheiser_gov_efi_t  fmav_loweheiser_gov_efi_t

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN  85
#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN  85
#define MAVLINK_MSG_ID_10151_LEN  85
#define MAVLINK_MSG_ID_10151_MIN_LEN  85

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC  195
#define MAVLINK_MSG_ID_10151_CRC  195




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_loweheiser_gov_efi_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_loweheiser_gov_efi_pack(
        _msg, sysid, compid,
        volt_batt, curr_batt, curr_gen, curr_rot, fuel_level, throttle, runtime, until_maintenance, rectifier_temp, generator_temp, efi_batt, efi_rpm, efi_pw, efi_fuel_flow, efi_fuel_consumed, efi_baro, efi_mat, efi_clt, efi_tps, efi_exhaust_gas_temperature, efi_index, generator_status, efi_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_loweheiser_gov_efi_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_loweheiser_gov_efi_t* _payload)
{
    return mavlink_msg_loweheiser_gov_efi_pack(
        sysid,
        compid,
        _msg,
        _payload->volt_batt, _payload->curr_batt, _payload->curr_gen, _payload->curr_rot, _payload->fuel_level, _payload->throttle, _payload->runtime, _payload->until_maintenance, _payload->rectifier_temp, _payload->generator_temp, _payload->efi_batt, _payload->efi_rpm, _payload->efi_pw, _payload->efi_fuel_flow, _payload->efi_fuel_consumed, _payload->efi_baro, _payload->efi_mat, _payload->efi_clt, _payload->efi_tps, _payload->efi_exhaust_gas_temperature, _payload->efi_index, _payload->generator_status, _payload->efi_status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_loweheiser_gov_efi_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
    return fmav_msg_loweheiser_gov_efi_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        volt_batt, curr_batt, curr_gen, curr_rot, fuel_level, throttle, runtime, until_maintenance, rectifier_temp, generator_temp, efi_batt, efi_rpm, efi_pw, efi_fuel_flow, efi_fuel_consumed, efi_baro, efi_mat, efi_clt, efi_tps, efi_exhaust_gas_temperature, efi_index, generator_status, efi_status,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_loweheiser_gov_efi_decode(const mavlink_message_t* msg, mavlink_loweheiser_gov_efi_t* payload)
{
    fmav_msg_loweheiser_gov_efi_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_LOWEHEISER_GOV_EFI_H

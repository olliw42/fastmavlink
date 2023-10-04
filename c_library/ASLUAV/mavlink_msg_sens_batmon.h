//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENS_BATMON_H
#define FASTMAVLINK_MSG_SENS_BATMON_H


//----------------------------------------
//-- Message SENS_BATMON
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sens_batmon_t {
    uint64_t batmon_timestamp;
    float temperature;
    uint32_t safetystatus;
    uint32_t operationstatus;
    uint16_t voltage;
    int16_t current;
    uint16_t batterystatus;
    uint16_t serialnumber;
    uint16_t cellvoltage1;
    uint16_t cellvoltage2;
    uint16_t cellvoltage3;
    uint16_t cellvoltage4;
    uint16_t cellvoltage5;
    uint16_t cellvoltage6;
    uint8_t SoC;
}) fmav_sens_batmon_t;


#define FASTMAVLINK_MSG_ID_SENS_BATMON  8010

#define FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX  41
#define FASTMAVLINK_MSG_SENS_BATMON_CRCEXTRA  155

#define FASTMAVLINK_MSG_SENS_BATMON_FLAGS  0
#define FASTMAVLINK_MSG_SENS_BATMON_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENS_BATMON_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENS_BATMON_FRAME_LEN_MAX  66



#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_BATMON_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_TEMPERATURE_OFS  8
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_SAFETYSTATUS_OFS  12
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_OPERATIONSTATUS_OFS  16
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_VOLTAGE_OFS  20
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CURRENT_OFS  22
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_BATTERYSTATUS_OFS  24
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_SERIALNUMBER_OFS  26
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE1_OFS  28
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE2_OFS  30
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE3_OFS  32
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE4_OFS  34
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE5_OFS  36
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_CELLVOLTAGE6_OFS  38
#define FASTMAVLINK_MSG_SENS_BATMON_FIELD_SOC_OFS  40


//----------------------------------------
//-- Message SENS_BATMON pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6,
    fmav_status_t* _status)
{
    fmav_sens_batmon_t* _payload = (fmav_sens_batmon_t*)_msg->payload;

    _payload->batmon_timestamp = batmon_timestamp;
    _payload->temperature = temperature;
    _payload->safetystatus = safetystatus;
    _payload->operationstatus = operationstatus;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->batterystatus = batterystatus;
    _payload->serialnumber = serialnumber;
    _payload->cellvoltage1 = cellvoltage1;
    _payload->cellvoltage2 = cellvoltage2;
    _payload->cellvoltage3 = cellvoltage3;
    _payload->cellvoltage4 = cellvoltage4;
    _payload->cellvoltage5 = cellvoltage5;
    _payload->cellvoltage6 = cellvoltage6;
    _payload->SoC = SoC;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENS_BATMON;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENS_BATMON_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_batmon_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_batmon_pack(
        _msg, sysid, compid,
        _payload->batmon_timestamp, _payload->temperature, _payload->voltage, _payload->current, _payload->SoC, _payload->batterystatus, _payload->serialnumber, _payload->safetystatus, _payload->operationstatus, _payload->cellvoltage1, _payload->cellvoltage2, _payload->cellvoltage3, _payload->cellvoltage4, _payload->cellvoltage5, _payload->cellvoltage6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6,
    fmav_status_t* _status)
{
    fmav_sens_batmon_t* _payload = (fmav_sens_batmon_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->batmon_timestamp = batmon_timestamp;
    _payload->temperature = temperature;
    _payload->safetystatus = safetystatus;
    _payload->operationstatus = operationstatus;
    _payload->voltage = voltage;
    _payload->current = current;
    _payload->batterystatus = batterystatus;
    _payload->serialnumber = serialnumber;
    _payload->cellvoltage1 = cellvoltage1;
    _payload->cellvoltage2 = cellvoltage2;
    _payload->cellvoltage3 = cellvoltage3;
    _payload->cellvoltage4 = cellvoltage4;
    _payload->cellvoltage5 = cellvoltage5;
    _payload->cellvoltage6 = cellvoltage6;
    _payload->SoC = SoC;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENS_BATMON;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_BATMON >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_BATMON >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_BATMON_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_batmon_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_batmon_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->batmon_timestamp, _payload->temperature, _payload->voltage, _payload->current, _payload->SoC, _payload->batterystatus, _payload->serialnumber, _payload->safetystatus, _payload->operationstatus, _payload->cellvoltage1, _payload->cellvoltage2, _payload->cellvoltage3, _payload->cellvoltage4, _payload->cellvoltage5, _payload->cellvoltage6,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6,
    fmav_status_t* _status)
{
    fmav_sens_batmon_t _payload;

    _payload.batmon_timestamp = batmon_timestamp;
    _payload.temperature = temperature;
    _payload.safetystatus = safetystatus;
    _payload.operationstatus = operationstatus;
    _payload.voltage = voltage;
    _payload.current = current;
    _payload.batterystatus = batterystatus;
    _payload.serialnumber = serialnumber;
    _payload.cellvoltage1 = cellvoltage1;
    _payload.cellvoltage2 = cellvoltage2;
    _payload.cellvoltage3 = cellvoltage3;
    _payload.cellvoltage4 = cellvoltage4;
    _payload.cellvoltage5 = cellvoltage5;
    _payload.cellvoltage6 = cellvoltage6;
    _payload.SoC = SoC;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENS_BATMON,
        FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_BATMON_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_batmon_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENS_BATMON,
        FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_BATMON_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENS_BATMON decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sens_batmon_decode(fmav_sens_batmon_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_BATMON_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_sens_batmon_get_field_batmon_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_batmon_get_field_temperature(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_sens_batmon_get_field_safetystatus(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_sens_batmon_get_field_operationstatus(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_voltage(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[20]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_sens_batmon_get_field_current(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[22]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_batterystatus(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[24]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_serialnumber(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage1(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage2(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[30]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage3(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[32]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage4(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[34]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage5(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[36]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_batmon_get_field_cellvoltage6(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[38]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_batmon_get_field_SoC(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENS_BATMON  8010

#define mavlink_sens_batmon_t  fmav_sens_batmon_t

#define MAVLINK_MSG_ID_SENS_BATMON_LEN  41
#define MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN  41
#define MAVLINK_MSG_ID_8010_LEN  41
#define MAVLINK_MSG_ID_8010_MIN_LEN  41

#define MAVLINK_MSG_ID_SENS_BATMON_CRC  155
#define MAVLINK_MSG_ID_8010_CRC  155




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_batmon_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sens_batmon_pack(
        _msg, sysid, compid,
        batmon_timestamp, temperature, voltage, current, SoC, batterystatus, serialnumber, safetystatus, operationstatus, cellvoltage1, cellvoltage2, cellvoltage3, cellvoltage4, cellvoltage5, cellvoltage6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_batmon_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sens_batmon_t* _payload)
{
    return mavlink_msg_sens_batmon_pack(
        sysid,
        compid,
        _msg,
        _payload->batmon_timestamp, _payload->temperature, _payload->voltage, _payload->current, _payload->SoC, _payload->batterystatus, _payload->serialnumber, _payload->safetystatus, _payload->operationstatus, _payload->cellvoltage1, _payload->cellvoltage2, _payload->cellvoltage3, _payload->cellvoltage4, _payload->cellvoltage5, _payload->cellvoltage6);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_batmon_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
    return fmav_msg_sens_batmon_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        batmon_timestamp, temperature, voltage, current, SoC, batterystatus, serialnumber, safetystatus, operationstatus, cellvoltage1, cellvoltage2, cellvoltage3, cellvoltage4, cellvoltage5, cellvoltage6,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sens_batmon_decode(const mavlink_message_t* msg, mavlink_sens_batmon_t* payload)
{
    fmav_msg_sens_batmon_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENS_BATMON_H

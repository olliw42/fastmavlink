//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_H
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_H


//----------------------------------------
//-- Message ONBOARD_COMPUTER_STATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_onboard_computer_status_t {
    uint64_t time_usec;
    uint32_t uptime;
    uint32_t ram_usage;
    uint32_t ram_total;
    uint32_t storage_type[4];
    uint32_t storage_usage[4];
    uint32_t storage_total[4];
    uint32_t link_type[6];
    uint32_t link_tx_rate[6];
    uint32_t link_rx_rate[6];
    uint32_t link_tx_max[6];
    uint32_t link_rx_max[6];
    int16_t fan_speed[4];
    uint8_t type;
    uint8_t cpu_cores[8];
    uint8_t cpu_combined[10];
    uint8_t gpu_cores[4];
    uint8_t gpu_combined[10];
    int8_t temperature_board;
    int8_t temperature_core[8];
}) fmav_onboard_computer_status_t;


#define FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS  390

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX  238
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA  156

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FRAME_LEN_MAX  263

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_LEN  16 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_LEN  24 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_LEN  8 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_LEN  10 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_NUM  4 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_LEN  4 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_NUM  10 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_LEN  10 // length of array = number of bytes
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_NUM  8 // number of elements in array
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_LEN  8 // length of array = number of bytes

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_UPTIME_OFS  8
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_RAM_USAGE_OFS  12
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_RAM_TOTAL_OFS  16
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_OFS  20
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_OFS  36
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_OFS  52
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_OFS  68
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_OFS  92
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_OFS  116
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_OFS  140
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_OFS  164
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_OFS  188
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TYPE_OFS  196
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_OFS  197
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_OFS  205
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_OFS  215
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_OFS  219
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_BOARD_OFS  229
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_OFS  230


//----------------------------------------
//-- Message ONBOARD_COMPUTER_STATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime, uint8_t type, const uint8_t* cpu_cores, const uint8_t* cpu_combined, const uint8_t* gpu_cores, const uint8_t* gpu_combined, int8_t temperature_board, const int8_t* temperature_core, const int16_t* fan_speed, uint32_t ram_usage, uint32_t ram_total, const uint32_t* storage_type, const uint32_t* storage_usage, const uint32_t* storage_total, const uint32_t* link_type, const uint32_t* link_tx_rate, const uint32_t* link_rx_rate, const uint32_t* link_tx_max, const uint32_t* link_rx_max,
    fmav_status_t* _status)
{
    fmav_onboard_computer_status_t* _payload = (fmav_onboard_computer_status_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->uptime = uptime;
    _payload->ram_usage = ram_usage;
    _payload->ram_total = ram_total;
    _payload->type = type;
    _payload->temperature_board = temperature_board;
    memcpy(&(_payload->storage_type), storage_type, sizeof(uint32_t)*4);
    memcpy(&(_payload->storage_usage), storage_usage, sizeof(uint32_t)*4);
    memcpy(&(_payload->storage_total), storage_total, sizeof(uint32_t)*4);
    memcpy(&(_payload->link_type), link_type, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_tx_rate), link_tx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_rx_rate), link_rx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_tx_max), link_tx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_rx_max), link_rx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload->fan_speed), fan_speed, sizeof(int16_t)*4);
    memcpy(&(_payload->cpu_cores), cpu_cores, sizeof(uint8_t)*8);
    memcpy(&(_payload->cpu_combined), cpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload->gpu_cores), gpu_cores, sizeof(uint8_t)*4);
    memcpy(&(_payload->gpu_combined), gpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload->temperature_core), temperature_core, sizeof(int8_t)*8);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_onboard_computer_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_onboard_computer_status_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->uptime, _payload->type, _payload->cpu_cores, _payload->cpu_combined, _payload->gpu_cores, _payload->gpu_combined, _payload->temperature_board, _payload->temperature_core, _payload->fan_speed, _payload->ram_usage, _payload->ram_total, _payload->storage_type, _payload->storage_usage, _payload->storage_total, _payload->link_type, _payload->link_tx_rate, _payload->link_rx_rate, _payload->link_tx_max, _payload->link_rx_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime, uint8_t type, const uint8_t* cpu_cores, const uint8_t* cpu_combined, const uint8_t* gpu_cores, const uint8_t* gpu_combined, int8_t temperature_board, const int8_t* temperature_core, const int16_t* fan_speed, uint32_t ram_usage, uint32_t ram_total, const uint32_t* storage_type, const uint32_t* storage_usage, const uint32_t* storage_total, const uint32_t* link_type, const uint32_t* link_tx_rate, const uint32_t* link_rx_rate, const uint32_t* link_tx_max, const uint32_t* link_rx_max,
    fmav_status_t* _status)
{
    fmav_onboard_computer_status_t* _payload = (fmav_onboard_computer_status_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->uptime = uptime;
    _payload->ram_usage = ram_usage;
    _payload->ram_total = ram_total;
    _payload->type = type;
    _payload->temperature_board = temperature_board;
    memcpy(&(_payload->storage_type), storage_type, sizeof(uint32_t)*4);
    memcpy(&(_payload->storage_usage), storage_usage, sizeof(uint32_t)*4);
    memcpy(&(_payload->storage_total), storage_total, sizeof(uint32_t)*4);
    memcpy(&(_payload->link_type), link_type, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_tx_rate), link_tx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_rx_rate), link_rx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_tx_max), link_tx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload->link_rx_max), link_rx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload->fan_speed), fan_speed, sizeof(int16_t)*4);
    memcpy(&(_payload->cpu_cores), cpu_cores, sizeof(uint8_t)*8);
    memcpy(&(_payload->cpu_combined), cpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload->gpu_cores), gpu_cores, sizeof(uint8_t)*4);
    memcpy(&(_payload->gpu_combined), gpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload->temperature_core), temperature_core, sizeof(int8_t)*8);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_onboard_computer_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_onboard_computer_status_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->uptime, _payload->type, _payload->cpu_cores, _payload->cpu_combined, _payload->gpu_cores, _payload->gpu_combined, _payload->temperature_board, _payload->temperature_core, _payload->fan_speed, _payload->ram_usage, _payload->ram_total, _payload->storage_type, _payload->storage_usage, _payload->storage_total, _payload->link_type, _payload->link_tx_rate, _payload->link_rx_rate, _payload->link_tx_max, _payload->link_rx_max,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime, uint8_t type, const uint8_t* cpu_cores, const uint8_t* cpu_combined, const uint8_t* gpu_cores, const uint8_t* gpu_combined, int8_t temperature_board, const int8_t* temperature_core, const int16_t* fan_speed, uint32_t ram_usage, uint32_t ram_total, const uint32_t* storage_type, const uint32_t* storage_usage, const uint32_t* storage_total, const uint32_t* link_type, const uint32_t* link_tx_rate, const uint32_t* link_rx_rate, const uint32_t* link_tx_max, const uint32_t* link_rx_max,
    fmav_status_t* _status)
{
    fmav_onboard_computer_status_t _payload;

    _payload.time_usec = time_usec;
    _payload.uptime = uptime;
    _payload.ram_usage = ram_usage;
    _payload.ram_total = ram_total;
    _payload.type = type;
    _payload.temperature_board = temperature_board;
    memcpy(&(_payload.storage_type), storage_type, sizeof(uint32_t)*4);
    memcpy(&(_payload.storage_usage), storage_usage, sizeof(uint32_t)*4);
    memcpy(&(_payload.storage_total), storage_total, sizeof(uint32_t)*4);
    memcpy(&(_payload.link_type), link_type, sizeof(uint32_t)*6);
    memcpy(&(_payload.link_tx_rate), link_tx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload.link_rx_rate), link_rx_rate, sizeof(uint32_t)*6);
    memcpy(&(_payload.link_tx_max), link_tx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload.link_rx_max), link_rx_max, sizeof(uint32_t)*6);
    memcpy(&(_payload.fan_speed), fan_speed, sizeof(int16_t)*4);
    memcpy(&(_payload.cpu_cores), cpu_cores, sizeof(uint8_t)*8);
    memcpy(&(_payload.cpu_combined), cpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload.gpu_cores), gpu_cores, sizeof(uint8_t)*4);
    memcpy(&(_payload.gpu_combined), gpu_combined, sizeof(uint8_t)*10);
    memcpy(&(_payload.temperature_core), temperature_core, sizeof(int8_t)*8);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_onboard_computer_status_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_onboard_computer_status_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ONBOARD_COMPUTER_STATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_onboard_computer_status_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_onboard_computer_status_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_onboard_computer_status_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_onboard_computer_status_decode(fmav_onboard_computer_status_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_onboard_computer_status_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_uptime(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_ram_usage(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_ram_total(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_onboard_computer_status_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[196]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_onboard_computer_status_get_field_temperature_board(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[229]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_storage_type_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_storage_type(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_NUM) return 0;
    return ((uint32_t*)&(msg->payload[20]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_storage_usage_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[36]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_storage_usage(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_NUM) return 0;
    return ((uint32_t*)&(msg->payload[36]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_storage_total_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[52]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_storage_total(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_NUM) return 0;
    return ((uint32_t*)&(msg->payload[52]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_link_type_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[68]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_link_type(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_NUM) return 0;
    return ((uint32_t*)&(msg->payload[68]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_link_tx_rate_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[92]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_link_tx_rate(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_NUM) return 0;
    return ((uint32_t*)&(msg->payload[92]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_link_rx_rate_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[116]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_link_rx_rate(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_NUM) return 0;
    return ((uint32_t*)&(msg->payload[116]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_link_tx_max_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[140]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_link_tx_max(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_NUM) return 0;
    return ((uint32_t*)&(msg->payload[140]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t* fmav_msg_onboard_computer_status_get_field_link_rx_max_ptr(const fmav_message_t* msg)
{
    return (uint32_t*)&(msg->payload[164]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_onboard_computer_status_get_field_link_rx_max(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_NUM) return 0;
    return ((uint32_t*)&(msg->payload[164]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t* fmav_msg_onboard_computer_status_get_field_fan_speed_ptr(const fmav_message_t* msg)
{
    return (int16_t*)&(msg->payload[188]);
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_onboard_computer_status_get_field_fan_speed(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_NUM) return 0;
    return ((int16_t*)&(msg->payload[188]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_onboard_computer_status_get_field_cpu_cores_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[197]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_onboard_computer_status_get_field_cpu_cores(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_NUM) return 0;
    return ((uint8_t*)&(msg->payload[197]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_onboard_computer_status_get_field_cpu_combined_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[205]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_onboard_computer_status_get_field_cpu_combined(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_NUM) return 0;
    return ((uint8_t*)&(msg->payload[205]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_onboard_computer_status_get_field_gpu_cores_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[215]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_onboard_computer_status_get_field_gpu_cores(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_NUM) return 0;
    return ((uint8_t*)&(msg->payload[215]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_onboard_computer_status_get_field_gpu_combined_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[219]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_onboard_computer_status_get_field_gpu_combined(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_NUM) return 0;
    return ((uint8_t*)&(msg->payload[219]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t* fmav_msg_onboard_computer_status_get_field_temperature_core_ptr(const fmav_message_t* msg)
{
    return (int8_t*)&(msg->payload[230]);
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_onboard_computer_status_get_field_temperature_core(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_NUM) return 0;
    return ((int8_t*)&(msg->payload[230]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS  390

#define mavlink_onboard_computer_status_t  fmav_onboard_computer_status_t

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS_LEN  238
#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS_MIN_LEN  238
#define MAVLINK_MSG_ID_390_LEN  238
#define MAVLINK_MSG_ID_390_MIN_LEN  238

#define MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS_CRC  156
#define MAVLINK_MSG_ID_390_CRC  156

#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_LEN 4
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_LEN 4
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_LEN 4
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_LEN 6
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_LEN 6
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_LEN 6
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_LEN 6
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_LEN 6
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_LEN 4
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_LEN 8
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_LEN 10
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_LEN 4
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_LEN 10
#define MAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_LEN 8


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_onboard_computer_status_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint32_t uptime, uint8_t type, const uint8_t* cpu_cores, const uint8_t* cpu_combined, const uint8_t* gpu_cores, const uint8_t* gpu_combined, int8_t temperature_board, const int8_t* temperature_core, const int16_t* fan_speed, uint32_t ram_usage, uint32_t ram_total, const uint32_t* storage_type, const uint32_t* storage_usage, const uint32_t* storage_total, const uint32_t* link_type, const uint32_t* link_tx_rate, const uint32_t* link_rx_rate, const uint32_t* link_tx_max, const uint32_t* link_rx_max)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_onboard_computer_status_pack(
        msg, sysid, compid,
        time_usec, uptime, type, cpu_cores, cpu_combined, gpu_cores, gpu_combined, temperature_board, temperature_core, fan_speed, ram_usage, ram_total, storage_type, storage_usage, storage_total, link_type, link_tx_rate, link_rx_rate, link_tx_max, link_rx_max,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_onboard_computer_status_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint32_t uptime, uint8_t type, const uint8_t* cpu_cores, const uint8_t* cpu_combined, const uint8_t* gpu_cores, const uint8_t* gpu_combined, int8_t temperature_board, const int8_t* temperature_core, const int16_t* fan_speed, uint32_t ram_usage, uint32_t ram_total, const uint32_t* storage_type, const uint32_t* storage_usage, const uint32_t* storage_total, const uint32_t* link_type, const uint32_t* link_tx_rate, const uint32_t* link_rx_rate, const uint32_t* link_tx_max, const uint32_t* link_rx_max)
{
    return fmav_msg_onboard_computer_status_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, uptime, type, cpu_cores, cpu_combined, gpu_cores, gpu_combined, temperature_board, temperature_core, fan_speed, ram_usage, ram_total, storage_type, storage_usage, storage_total, link_type, link_tx_rate, link_rx_rate, link_tx_max, link_rx_max,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_onboard_computer_status_decode(const mavlink_message_t* msg, mavlink_onboard_computer_status_t* payload)
{
    fmav_msg_onboard_computer_status_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_H

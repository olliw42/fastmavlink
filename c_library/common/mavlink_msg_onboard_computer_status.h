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


#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MIN  238
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX  238
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN  238
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_CRCEXTRA  156

#define FASTMAVLINK_MSG_ID_390_LEN_MIN  238
#define FASTMAVLINK_MSG_ID_390_LEN_MAX  238
#define FASTMAVLINK_MSG_ID_390_LEN  238
#define FASTMAVLINK_MSG_ID_390_CRCEXTRA  156

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TYPE_LEN  4
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_USAGE_LEN  4
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_STORAGE_TOTAL_LEN  4
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TYPE_LEN  6
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_RATE_LEN  6
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_RATE_LEN  6
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_TX_MAX_LEN  6
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_LINK_RX_MAX_LEN  6
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_FAN_SPEED_LEN  4
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_CORES_LEN  8
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_CPU_COMBINED_LEN  10
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_CORES_LEN  4
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_GPU_COMBINED_LEN  10
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FIELD_TEMPERATURE_CORE_LEN  8

#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_FLAGS  0
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ONBOARD_COMPUTER_STATUS packing routines
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
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message ONBOARD_COMPUTER_STATUS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_onboard_computer_status_decode(fmav_onboard_computer_status_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ONBOARD_COMPUTER_STATUS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

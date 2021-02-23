//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEVICE_OP_READ_H
#define FASTMAVLINK_MSG_DEVICE_OP_READ_H


//----------------------------------------
//-- Message DEVICE_OP_READ
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_device_op_read_t {
    uint32_t request_id;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t bustype;
    uint8_t bus;
    uint8_t address;
    char busname[40];
    uint8_t regstart;
    uint8_t count;
    uint8_t bank;
}) fmav_device_op_read_t;


#define FASTMAVLINK_MSG_ID_DEVICE_OP_READ  11000


#define FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MIN  51
#define FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN  52
#define FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA  134

#define FASTMAVLINK_MSG_ID_11000_LEN_MIN  51
#define FASTMAVLINK_MSG_ID_11000_LEN_MAX  52
#define FASTMAVLINK_MSG_ID_11000_LEN  52
#define FASTMAVLINK_MSG_ID_11000_CRCEXTRA  134

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_LEN  40

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FLAGS  3
#define FASTMAVLINK_MSG_DEVICE_OP_READ_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DEVICE_OP_READ_TARGET_COMPONENT_OFS  5


//----------------------------------------
//-- Message DEVICE_OP_READ packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_t* _payload = (fmav_device_op_read_t*)msg->payload;

    _payload->request_id = request_id;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bustype = bustype;
    _payload->bus = bus;
    _payload->address = address;
    _payload->regstart = regstart;
    _payload->count = count;
    _payload->bank = bank;
    memcpy(&(_payload->busname), busname, sizeof(char)*40);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_DEVICE_OP_READ;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_read_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->bustype, _payload->bus, _payload->address, _payload->busname, _payload->regstart, _payload->count, _payload->bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_t* _payload = (fmav_device_op_read_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->request_id = request_id;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->bustype = bustype;
    _payload->bus = bus;
    _payload->address = address;
    _payload->regstart = regstart;
    _payload->count = count;
    _payload->bank = bank;
    memcpy(&(_payload->busname), busname, sizeof(char)*40);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_READ >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_read_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->bustype, _payload->bus, _payload->address, _payload->busname, _payload->regstart, _payload->count, _payload->bank,
        _status);
}


//----------------------------------------
//-- Message DEVICE_OP_READ unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_device_op_read_decode(fmav_device_op_read_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEVICE_OP_READ  11000

#define mavlink_device_op_read_t  fmav_device_op_read_t

#define MAVLINK_MSG_ID_DEVICE_OP_READ_LEN  52
#define MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN  51
#define MAVLINK_MSG_ID_11000_LEN  52
#define MAVLINK_MSG_ID_11000_MIN_LEN  51

#define MAVLINK_MSG_ID_DEVICE_OP_READ_CRC  134
#define MAVLINK_MSG_ID_11000_CRC  134

#define MAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_LEN 40


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_read_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, uint8_t bank)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_device_op_read_pack(
        msg, sysid, compid,
        target_system, target_component, request_id, bustype, bus, address, busname, regstart, count, bank,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_read_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, uint8_t bank)
{
    return fmav_msg_device_op_read_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, request_id, bustype, bus, address, busname, regstart, count, bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_device_op_read_decode(const mavlink_message_t* msg, mavlink_device_op_read_t* payload)
{
    fmav_msg_device_op_read_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEVICE_OP_READ_H

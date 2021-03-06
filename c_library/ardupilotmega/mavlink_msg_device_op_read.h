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

#define FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX  52
#define FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA  134

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FLAGS  3
#define FASTMAVLINK_MSG_DEVICE_OP_READ_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DEVICE_OP_READ_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FRAME_LEN_MAX  77

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_NUM  40 // number of elements in array
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_LEN  40 // length of array = number of bytes

#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_REQUEST_ID_OFS  0
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSTYPE_OFS  6
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUS_OFS  7
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_ADDRESS_OFS  8
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_OFS  9
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_REGSTART_OFS  49
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_COUNT_OFS  50
#define FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BANK_OFS  51


//----------------------------------------
//-- Message DEVICE_OP_READ packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_read_t _payload;

    _payload.request_id = request_id;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.bustype = bustype;
    _payload.bus = bus;
    _payload.address = address;
    _payload.regstart = regstart;
    _payload.count = count;
    _payload.bank = bank;
    memcpy(&(_payload.busname), busname, sizeof(char)*40);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_READ,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_read_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_read_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_READ,
        FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_READ_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEVICE_OP_READ unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_device_op_read_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_device_op_read_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_device_op_read_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_device_op_read_decode(fmav_device_op_read_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEVICE_OP_READ_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_device_op_read_get_field_request_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_bustype(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_bus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_address(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_regstart(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_read_get_field_bank(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[51]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_device_op_read_get_field_busname_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[9]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_device_op_read_get_field_busname(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_NUM) return 0;
    return ((char*)&(msg->payload[9]))[index];
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_DEVICE_OP_WRITE_H
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_H


//----------------------------------------
//-- Message DEVICE_OP_WRITE
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_device_op_write_t {
    uint32_t request_id;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t bustype;
    uint8_t bus;
    uint8_t address;
    char busname[40];
    uint8_t regstart;
    uint8_t count;
    uint8_t data[128];
    uint8_t bank;
}) fmav_device_op_write_t;


#define FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE  11002

#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX  180
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA  234

#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FLAGS  3
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FRAME_LEN_MAX  205

#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSNAME_NUM  40 // number of elements in array
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSNAME_LEN  40 // length of array = number of bytes
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_DATA_NUM  128 // number of elements in array
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_DATA_LEN  128 // length of array = number of bytes

#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_REQUEST_ID_OFS  0
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSTYPE_OFS  6
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUS_OFS  7
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_ADDRESS_OFS  8
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSNAME_OFS  9
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_REGSTART_OFS  49
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_COUNT_OFS  50
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_DATA_OFS  51
#define FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BANK_OFS  179


//----------------------------------------
//-- Message DEVICE_OP_WRITE pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_write_t* _payload = (fmav_device_op_write_t*)_msg->payload;

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
    memcpy(&(_payload->data), data, sizeof(uint8_t)*128);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_write_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_write_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->bustype, _payload->bus, _payload->address, _payload->busname, _payload->regstart, _payload->count, _payload->data, _payload->bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_write_t* _payload = (fmav_device_op_write_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

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
    memcpy(&(_payload->data), data, sizeof(uint8_t)*128);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_write_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_device_op_write_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->bustype, _payload->bus, _payload->address, _payload->busname, _payload->regstart, _payload->count, _payload->data, _payload->bank,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank,
    fmav_status_t* _status)
{
    fmav_device_op_write_t _payload;

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
    memcpy(&(_payload.data), data, sizeof(uint8_t)*128);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_device_op_write_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_device_op_write_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_DEVICE_OP_WRITE,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_DEVICE_OP_WRITE_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message DEVICE_OP_WRITE decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_device_op_write_decode(fmav_device_op_write_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_DEVICE_OP_WRITE_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_device_op_write_get_field_request_id(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_bustype(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_bus(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_address(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_regstart(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[49]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[50]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_bank(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[179]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_device_op_write_get_field_busname_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[9]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_device_op_write_get_field_busname(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSNAME_NUM) return 0;
    return ((char*)&(msg->payload[9]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_device_op_write_get_field_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[51]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_device_op_write_get_field_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_DEVICE_OP_WRITE_FIELD_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[51]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_DEVICE_OP_WRITE  11002

#define mavlink_device_op_write_t  fmav_device_op_write_t

#define MAVLINK_MSG_ID_DEVICE_OP_WRITE_LEN  180
#define MAVLINK_MSG_ID_DEVICE_OP_WRITE_MIN_LEN  179
#define MAVLINK_MSG_ID_11002_LEN  180
#define MAVLINK_MSG_ID_11002_MIN_LEN  179

#define MAVLINK_MSG_ID_DEVICE_OP_WRITE_CRC  234
#define MAVLINK_MSG_ID_11002_CRC  234

#define MAVLINK_MSG_DEVICE_OP_WRITE_FIELD_BUSNAME_LEN 40
#define MAVLINK_MSG_DEVICE_OP_WRITE_FIELD_DATA_LEN 128


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_write_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_device_op_write_pack(
        _msg, sysid, compid,
        target_system, target_component, request_id, bustype, bus, address, busname, regstart, count, data, bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_write_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_device_op_write_t* _payload)
{
    return mavlink_msg_device_op_write_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->request_id, _payload->bustype, _payload->bus, _payload->address, _payload->busname, _payload->regstart, _payload->count, _payload->data, _payload->bank);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_device_op_write_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char* busname, uint8_t regstart, uint8_t count, const uint8_t* data, uint8_t bank)
{
    return fmav_msg_device_op_write_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, request_id, bustype, bus, address, busname, regstart, count, data, bank,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_device_op_write_decode(const mavlink_message_t* msg, mavlink_device_op_write_t* payload)
{
    fmav_msg_device_op_write_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_DEVICE_OP_WRITE_H

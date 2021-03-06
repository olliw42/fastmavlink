//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_H
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_H


//----------------------------------------
//-- Message OPEN_DRONE_ID_AUTHENTICATION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_open_drone_id_authentication_t {
    uint32_t timestamp;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t id_or_mac[20];
    uint8_t authentication_type;
    uint8_t data_page;
    uint8_t page_count;
    uint8_t length;
    uint8_t authentication_data[23];
}) fmav_open_drone_id_authentication_t;


#define FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION  12902

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX  53
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA  49

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FLAGS  3
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_TARGET_COMPONENT_OFS  5

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FRAME_LEN_MAX  78

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_NUM  20 // number of elements in array
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_LEN  20 // length of array = number of bytes
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_NUM  23 // number of elements in array
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN  23 // length of array = number of bytes

#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_TARGET_SYSTEM_OFS  4
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_TARGET_COMPONENT_OFS  5
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_OFS  6
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_TYPE_OFS  26
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_DATA_PAGE_OFS  27
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_PAGE_COUNT_OFS  28
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_LENGTH_OFS  29
#define FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_OFS  30


//----------------------------------------
//-- Message OPEN_DRONE_ID_AUTHENTICATION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t authentication_type, uint8_t data_page, uint8_t page_count, uint8_t length, uint32_t timestamp, const uint8_t* authentication_data,
    fmav_status_t* _status)
{
    fmav_open_drone_id_authentication_t* _payload = (fmav_open_drone_id_authentication_t*)msg->payload;

    _payload->timestamp = timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->authentication_type = authentication_type;
    _payload->data_page = data_page;
    _payload->page_count = page_count;
    _payload->length = length;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);
    memcpy(&(_payload->authentication_data), authentication_data, sizeof(uint8_t)*23);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_authentication_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_authentication_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->authentication_type, _payload->data_page, _payload->page_count, _payload->length, _payload->timestamp, _payload->authentication_data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t authentication_type, uint8_t data_page, uint8_t page_count, uint8_t length, uint32_t timestamp, const uint8_t* authentication_data,
    fmav_status_t* _status)
{
    fmav_open_drone_id_authentication_t* _payload = (fmav_open_drone_id_authentication_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->authentication_type = authentication_type;
    _payload->data_page = data_page;
    _payload->page_count = page_count;
    _payload->length = length;
    memcpy(&(_payload->id_or_mac), id_or_mac, sizeof(uint8_t)*20);
    memcpy(&(_payload->authentication_data), authentication_data, sizeof(uint8_t)*23);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_authentication_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_open_drone_id_authentication_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->id_or_mac, _payload->authentication_type, _payload->data_page, _payload->page_count, _payload->length, _payload->timestamp, _payload->authentication_data,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t authentication_type, uint8_t data_page, uint8_t page_count, uint8_t length, uint32_t timestamp, const uint8_t* authentication_data,
    fmav_status_t* _status)
{
    fmav_open_drone_id_authentication_t _payload;

    _payload.timestamp = timestamp;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.authentication_type = authentication_type;
    _payload.data_page = data_page;
    _payload.page_count = page_count;
    _payload.length = length;
    memcpy(&(_payload.id_or_mac), id_or_mac, sizeof(uint8_t)*20);
    memcpy(&(_payload.authentication_data), authentication_data, sizeof(uint8_t)*23);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_open_drone_id_authentication_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_open_drone_id_authentication_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message OPEN_DRONE_ID_AUTHENTICATION unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_open_drone_id_authentication_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_open_drone_id_authentication_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_open_drone_id_authentication_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_open_drone_id_authentication_decode(fmav_open_drone_id_authentication_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_open_drone_id_authentication_get_field_timestamp(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[5]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_authentication_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_data_page(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_page_count(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[28]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_length(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[29]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_open_drone_id_authentication_get_field_id_or_mac_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[6]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_id_or_mac(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_NUM) return 0;
    return ((uint8_t*)&(msg->payload[6]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t* fmav_msg_open_drone_id_authentication_get_field_authentication_data_ptr(const fmav_message_t* msg)
{
    return (uint8_t*)&(msg->payload[30]);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_open_drone_id_authentication_get_field_authentication_data(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_NUM) return 0;
    return ((uint8_t*)&(msg->payload[30]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION  12902

#define mavlink_open_drone_id_authentication_t  fmav_open_drone_id_authentication_t

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_LEN  53
#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_MIN_LEN  53
#define MAVLINK_MSG_ID_12902_LEN  53
#define MAVLINK_MSG_ID_12902_MIN_LEN  53

#define MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION_CRC  49
#define MAVLINK_MSG_ID_12902_CRC  49

#define MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_ID_OR_MAC_LEN 20
#define MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN 23


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_authentication_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t authentication_type, uint8_t data_page, uint8_t page_count, uint8_t length, uint32_t timestamp, const uint8_t* authentication_data)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_open_drone_id_authentication_pack(
        msg, sysid, compid,
        target_system, target_component, id_or_mac, authentication_type, data_page, page_count, length, timestamp, authentication_data,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_open_drone_id_authentication_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, const uint8_t* id_or_mac, uint8_t authentication_type, uint8_t data_page, uint8_t page_count, uint8_t length, uint32_t timestamp, const uint8_t* authentication_data)
{
    return fmav_msg_open_drone_id_authentication_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, id_or_mac, authentication_type, data_page, page_count, length, timestamp, authentication_data,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_open_drone_id_authentication_decode(const mavlink_message_t* msg, mavlink_open_drone_id_authentication_t* payload)
{
    fmav_msg_open_drone_id_authentication_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_H

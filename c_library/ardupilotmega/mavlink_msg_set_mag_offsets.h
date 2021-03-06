//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SET_MAG_OFFSETS_H
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_H


//----------------------------------------
//-- Message SET_MAG_OFFSETS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_set_mag_offsets_t {
    int16_t mag_ofs_x;
    int16_t mag_ofs_y;
    int16_t mag_ofs_z;
    uint8_t target_system;
    uint8_t target_component;
}) fmav_set_mag_offsets_t;


#define FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS  151

#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA  219

#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FLAGS  3
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_TARGET_COMPONENT_OFS  7

#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FIELD_MAG_OFS_X_OFS  0
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FIELD_MAG_OFS_Y_OFS  2
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FIELD_MAG_OFS_Z_OFS  4
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FIELD_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FIELD_TARGET_COMPONENT_OFS  7


//----------------------------------------
//-- Message SET_MAG_OFFSETS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z,
    fmav_status_t* _status)
{
    fmav_set_mag_offsets_t* _payload = (fmav_set_mag_offsets_t*)msg->payload;

    _payload->mag_ofs_x = mag_ofs_x;
    _payload->mag_ofs_y = mag_ofs_y;
    _payload->mag_ofs_z = mag_ofs_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS;

    msg->target_sysid = target_system;
    msg->target_compid = target_component;
    msg->crc_extra = FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mag_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mag_offsets_pack(
        msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z,
    fmav_status_t* _status)
{
    fmav_set_mag_offsets_t* _payload = (fmav_set_mag_offsets_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->mag_ofs_x = mag_ofs_x;
    _payload->mag_ofs_y = mag_ofs_y;
    _payload->mag_ofs_z = mag_ofs_z;
    _payload->target_system = target_system;
    _payload->target_component = target_component;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mag_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_set_mag_offsets_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->mag_ofs_x, _payload->mag_ofs_y, _payload->mag_ofs_z,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z,
    fmav_status_t* _status)
{
    fmav_set_mag_offsets_t _payload;

    _payload.mag_ofs_x = mag_ofs_x;
    _payload.mag_ofs_y = mag_ofs_y;
    _payload.mag_ofs_z = mag_ofs_z;
    _payload.target_system = target_system;
    _payload.target_component = target_component;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_set_mag_offsets_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_set_mag_offsets_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SET_MAG_OFFSETS,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SET_MAG_OFFSETS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_set_mag_offsets_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_set_mag_offsets_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_mag_offsets_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_mag_offsets_decode(fmav_set_mag_offsets_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_set_mag_offsets_get_field_mag_ofs_x(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_set_mag_offsets_get_field_mag_ofs_y(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_set_mag_offsets_get_field_mag_ofs_z(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_mag_offsets_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_set_mag_offsets_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[7]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS  151

#define mavlink_set_mag_offsets_t  fmav_set_mag_offsets_t

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN  8
#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_MIN_LEN  8
#define MAVLINK_MSG_ID_151_LEN  8
#define MAVLINK_MSG_ID_151_MIN_LEN  8

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC  219
#define MAVLINK_MSG_ID_151_CRC  219




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mag_offsets_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_set_mag_offsets_pack(
        msg, sysid, compid,
        target_system, target_component, mag_ofs_x, mag_ofs_y, mag_ofs_z,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_set_mag_offsets_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
    return fmav_msg_set_mag_offsets_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        target_system, target_component, mag_ofs_x, mag_ofs_y, mag_ofs_z,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_set_mag_offsets_decode(const mavlink_message_t* msg, mavlink_set_mag_offsets_t* payload)
{
    fmav_msg_set_mag_offsets_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SET_MAG_OFFSETS_H

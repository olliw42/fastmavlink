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


#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MIN  8
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN  8
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_CRCEXTRA  219

#define FASTMAVLINK_MSG_ID_151_LEN_MIN  8
#define FASTMAVLINK_MSG_ID_151_LEN_MAX  8
#define FASTMAVLINK_MSG_ID_151_LEN  8
#define FASTMAVLINK_MSG_ID_151_CRCEXTRA  219



#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_FLAGS  3
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_TARGET_SYSTEM_OFS  6
#define FASTMAVLINK_MSG_SET_MAG_OFFSETS_TARGET_COMPONENT_OFS  7


//----------------------------------------
//-- Message SET_MAG_OFFSETS packing routines
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
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MIN,
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
        FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MIN,
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


//----------------------------------------
//-- Message SET_MAG_OFFSETS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_set_mag_offsets_decode(fmav_set_mag_offsets_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_SET_MAG_OFFSETS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_CHECKSUM_H
#define FASTMAVLINK_MSG_MISSION_CHECKSUM_H


//----------------------------------------
//-- Message MISSION_CHECKSUM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_checksum_t {
    uint32_t checksum;
    uint8_t mission_type;
}) fmav_mission_checksum_t;


#define FASTMAVLINK_MSG_ID_MISSION_CHECKSUM  53

#define FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX  5
#define FASTMAVLINK_MSG_MISSION_CHECKSUM_CRCEXTRA  3

#define FASTMAVLINK_MSG_MISSION_CHECKSUM_FLAGS  0
#define FASTMAVLINK_MSG_MISSION_CHECKSUM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MISSION_CHECKSUM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MISSION_CHECKSUM_FRAME_LEN_MAX  30



#define FASTMAVLINK_MSG_MISSION_CHECKSUM_FIELD_CHECKSUM_OFS  0
#define FASTMAVLINK_MSG_MISSION_CHECKSUM_FIELD_MISSION_TYPE_OFS  4


//----------------------------------------
//-- Message MISSION_CHECKSUM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mission_type, uint32_t checksum,
    fmav_status_t* _status)
{
    fmav_mission_checksum_t* _payload = (fmav_mission_checksum_t*)_msg->payload;

    _payload->checksum = checksum;
    _payload->mission_type = mission_type;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MISSION_CHECKSUM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_MISSION_CHECKSUM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_checksum_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_checksum_pack(
        _msg, sysid, compid,
        _payload->mission_type, _payload->checksum,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mission_type, uint32_t checksum,
    fmav_status_t* _status)
{
    fmav_mission_checksum_t* _payload = (fmav_mission_checksum_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->checksum = checksum;
    _payload->mission_type = mission_type;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_CHECKSUM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHECKSUM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_CHECKSUM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_checksum_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_checksum_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->mission_type, _payload->checksum,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t mission_type, uint32_t checksum,
    fmav_status_t* _status)
{
    fmav_mission_checksum_t _payload;

    _payload.checksum = checksum;
    _payload.mission_type = mission_type;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_CHECKSUM,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_checksum_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_checksum_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_CHECKSUM,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_CHECKSUM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_CHECKSUM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zerofill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_checksum_decode(fmav_mission_checksum_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    memcpy(payload, msg->payload, msg->len);
    // ensure that returned payload is zero filled
    if (msg->len < FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX - msg->len);
    }
#else
    // this requires that msg payload had been zero filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_CHECKSUM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_mission_checksum_get_field_checksum(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mission_checksum_get_field_mission_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_CHECKSUM  53

#define mavlink_mission_checksum_t  fmav_mission_checksum_t

#define MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN  5
#define MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN  5
#define MAVLINK_MSG_ID_53_LEN  5
#define MAVLINK_MSG_ID_53_MIN_LEN  5

#define MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC  3
#define MAVLINK_MSG_ID_53_CRC  3




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_checksum_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t mission_type, uint32_t checksum)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_checksum_pack(
        _msg, sysid, compid,
        mission_type, checksum,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_checksum_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t mission_type, uint32_t checksum)
{
    return fmav_msg_mission_checksum_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        mission_type, checksum,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_checksum_decode(const mavlink_message_t* msg, mavlink_mission_checksum_t* payload)
{
    fmav_msg_mission_checksum_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_CHECKSUM_H

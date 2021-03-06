//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MISSION_ITEM_REACHED_H
#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_H


//----------------------------------------
//-- Message MISSION_ITEM_REACHED
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mission_item_reached_t {
    uint16_t seq;
}) fmav_mission_item_reached_t;


#define FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED  46

#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX  2
#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA  11

#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_FLAGS  0
#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_FRAME_LEN_MAX  27



#define FASTMAVLINK_MSG_MISSION_ITEM_REACHED_FIELD_SEQ_OFS  0


//----------------------------------------
//-- Message MISSION_ITEM_REACHED packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq,
    fmav_status_t* _status)
{
    fmav_mission_item_reached_t* _payload = (fmav_mission_item_reached_t*)msg->payload;

    _payload->seq = seq;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_reached_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_reached_pack(
        msg, sysid, compid,
        _payload->seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq,
    fmav_status_t* _status)
{
    fmav_mission_item_reached_t* _payload = (fmav_mission_item_reached_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->seq = seq;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_reached_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mission_item_reached_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->seq,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq,
    fmav_status_t* _status)
{
    fmav_mission_item_reached_t _payload;

    _payload.seq = seq;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mission_item_reached_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MISSION_ITEM_REACHED,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MISSION_ITEM_REACHED_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MISSION_ITEM_REACHED unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_mission_item_reached_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_mission_item_reached_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_item_reached_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mission_item_reached_decode(fmav_mission_item_reached_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MISSION_ITEM_REACHED_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mission_item_reached_get_field_seq(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED  46

#define mavlink_mission_item_reached_t  fmav_mission_item_reached_t

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED_LEN  2
#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED_MIN_LEN  2
#define MAVLINK_MSG_ID_46_LEN  2
#define MAVLINK_MSG_ID_46_MIN_LEN  2

#define MAVLINK_MSG_ID_MISSION_ITEM_REACHED_CRC  11
#define MAVLINK_MSG_ID_46_CRC  11




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_reached_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t seq)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mission_item_reached_pack(
        msg, sysid, compid,
        seq,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mission_item_reached_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t seq)
{
    return fmav_msg_mission_item_reached_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        seq,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mission_item_reached_decode(const mavlink_message_t* msg, mavlink_mission_item_reached_t* payload)
{
    fmav_msg_mission_item_reached_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MISSION_ITEM_REACHED_H

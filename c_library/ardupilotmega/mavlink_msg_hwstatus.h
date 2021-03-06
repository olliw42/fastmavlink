//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HWSTATUS_H
#define FASTMAVLINK_MSG_HWSTATUS_H


//----------------------------------------
//-- Message HWSTATUS
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_hwstatus_t {
    uint16_t Vcc;
    uint8_t I2Cerr;
}) fmav_hwstatus_t;


#define FASTMAVLINK_MSG_ID_HWSTATUS  165

#define FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX  3
#define FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA  21

#define FASTMAVLINK_MSG_HWSTATUS_FLAGS  0
#define FASTMAVLINK_MSG_HWSTATUS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HWSTATUS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HWSTATUS_FRAME_LEN_MAX  28



#define FASTMAVLINK_MSG_HWSTATUS_FIELD_VCC_OFS  0
#define FASTMAVLINK_MSG_HWSTATUS_FIELD_I2CERR_OFS  2


//----------------------------------------
//-- Message HWSTATUS packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint16_t Vcc, uint8_t I2Cerr,
    fmav_status_t* _status)
{
    fmav_hwstatus_t* _payload = (fmav_hwstatus_t*)msg->payload;

    _payload->Vcc = Vcc;
    _payload->I2Cerr = I2Cerr;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_HWSTATUS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hwstatus_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hwstatus_pack(
        msg, sysid, compid,
        _payload->Vcc, _payload->I2Cerr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint16_t Vcc, uint8_t I2Cerr,
    fmav_status_t* _status)
{
    fmav_hwstatus_t* _payload = (fmav_hwstatus_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->Vcc = Vcc;
    _payload->I2Cerr = I2Cerr;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HWSTATUS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HWSTATUS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HWSTATUS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_hwstatus_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_hwstatus_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->Vcc, _payload->I2Cerr,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint16_t Vcc, uint8_t I2Cerr,
    fmav_status_t* _status)
{
    fmav_hwstatus_t _payload;

    _payload.Vcc = Vcc;
    _payload.I2Cerr = I2Cerr;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HWSTATUS,
        FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_hwstatus_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HWSTATUS,
        FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HWSTATUS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HWSTATUS unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_hwstatus_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_hwstatus_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hwstatus_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_hwstatus_decode(fmav_hwstatus_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HWSTATUS_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_hwstatus_get_field_Vcc(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_hwstatus_get_field_I2Cerr(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HWSTATUS  165

#define mavlink_hwstatus_t  fmav_hwstatus_t

#define MAVLINK_MSG_ID_HWSTATUS_LEN  3
#define MAVLINK_MSG_ID_HWSTATUS_MIN_LEN  3
#define MAVLINK_MSG_ID_165_LEN  3
#define MAVLINK_MSG_ID_165_MIN_LEN  3

#define MAVLINK_MSG_ID_HWSTATUS_CRC  21
#define MAVLINK_MSG_ID_165_CRC  21




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hwstatus_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint16_t Vcc, uint8_t I2Cerr)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_hwstatus_pack(
        msg, sysid, compid,
        Vcc, I2Cerr,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_hwstatus_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint16_t Vcc, uint8_t I2Cerr)
{
    return fmav_msg_hwstatus_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        Vcc, I2Cerr,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_hwstatus_decode(const mavlink_message_t* msg, mavlink_hwstatus_t* payload)
{
    fmav_msg_hwstatus_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HWSTATUS_H

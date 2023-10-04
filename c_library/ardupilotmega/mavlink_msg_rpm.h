//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_RPM_H
#define FASTMAVLINK_MSG_RPM_H


//----------------------------------------
//-- Message RPM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_rpm_t {
    float rpm1;
    float rpm2;
}) fmav_rpm_t;


#define FASTMAVLINK_MSG_ID_RPM  226

#define FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX  8
#define FASTMAVLINK_MSG_RPM_CRCEXTRA  207

#define FASTMAVLINK_MSG_RPM_FLAGS  0
#define FASTMAVLINK_MSG_RPM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_RPM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_RPM_FRAME_LEN_MAX  33



#define FASTMAVLINK_MSG_RPM_FIELD_RPM1_OFS  0
#define FASTMAVLINK_MSG_RPM_FIELD_RPM2_OFS  4


//----------------------------------------
//-- Message RPM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2,
    fmav_status_t* _status)
{
    fmav_rpm_t* _payload = (fmav_rpm_t*)_msg->payload;

    _payload->rpm1 = rpm1;
    _payload->rpm2 = rpm2;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_RPM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_RPM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rpm_pack(
        _msg, sysid, compid,
        _payload->rpm1, _payload->rpm2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2,
    fmav_status_t* _status)
{
    fmav_rpm_t* _payload = (fmav_rpm_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rpm1 = rpm1;
    _payload->rpm2 = rpm2;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_RPM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_RPM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_RPM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_rpm_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->rpm1, _payload->rpm2,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2,
    fmav_status_t* _status)
{
    fmav_rpm_t _payload;

    _payload.rpm1 = rpm1;
    _payload.rpm2 = rpm2;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_RPM,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RPM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_rpm_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_rpm_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_RPM,
        FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_RPM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message RPM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_rpm_decode(fmav_rpm_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_RPM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rpm_get_field_rpm1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_rpm_get_field_rpm2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_RPM  226

#define mavlink_rpm_t  fmav_rpm_t

#define MAVLINK_MSG_ID_RPM_LEN  8
#define MAVLINK_MSG_ID_RPM_MIN_LEN  8
#define MAVLINK_MSG_ID_226_LEN  8
#define MAVLINK_MSG_ID_226_MIN_LEN  8

#define MAVLINK_MSG_ID_RPM_CRC  207
#define MAVLINK_MSG_ID_226_CRC  207




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rpm_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    float rpm1, float rpm2)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_rpm_pack(
        _msg, sysid, compid,
        rpm1, rpm2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rpm_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_rpm_t* _payload)
{
    return mavlink_msg_rpm_pack(
        sysid,
        compid,
        _msg,
        _payload->rpm1, _payload->rpm2);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_rpm_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float rpm1, float rpm2)
{
    return fmav_msg_rpm_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        rpm1, rpm2,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_rpm_decode(const mavlink_message_t* msg, mavlink_rpm_t* payload)
{
    fmav_msg_rpm_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_RPM_H

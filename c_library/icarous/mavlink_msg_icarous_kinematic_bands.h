//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_H
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_H


//----------------------------------------
//-- Message ICAROUS_KINEMATIC_BANDS
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_icarous_kinematic_bands_t {
    float min1;
    float max1;
    float min2;
    float max2;
    float min3;
    float max3;
    float min4;
    float max4;
    float min5;
    float max5;
    int8_t numBands;
    uint8_t type1;
    uint8_t type2;
    uint8_t type3;
    uint8_t type4;
    uint8_t type5;
}) fmav_icarous_kinematic_bands_t;


#define FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS  42001

#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX  46
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA  239

#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FLAGS  0
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FRAME_LEN_MAX  71



#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MIN1_OFS  0
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MAX1_OFS  4
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MIN2_OFS  8
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MAX2_OFS  12
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MIN3_OFS  16
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MAX3_OFS  20
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MIN4_OFS  24
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MAX4_OFS  28
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MIN5_OFS  32
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_MAX5_OFS  36
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_NUMBANDS_OFS  40
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_TYPE1_OFS  41
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_TYPE2_OFS  42
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_TYPE3_OFS  43
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_TYPE4_OFS  44
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FIELD_TYPE5_OFS  45


//----------------------------------------
//-- Message ICAROUS_KINEMATIC_BANDS pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5,
    fmav_status_t* _status)
{
    fmav_icarous_kinematic_bands_t* _payload = (fmav_icarous_kinematic_bands_t*)_msg->payload;

    _payload->min1 = min1;
    _payload->max1 = max1;
    _payload->min2 = min2;
    _payload->max2 = max2;
    _payload->min3 = min3;
    _payload->max3 = max3;
    _payload->min4 = min4;
    _payload->max4 = max4;
    _payload->min5 = min5;
    _payload->max5 = max5;
    _payload->numBands = numBands;
    _payload->type1 = type1;
    _payload->type2 = type2;
    _payload->type3 = type3;
    _payload->type4 = type4;
    _payload->type5 = type5;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_kinematic_bands_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_kinematic_bands_pack(
        _msg, sysid, compid,
        _payload->numBands, _payload->type1, _payload->min1, _payload->max1, _payload->type2, _payload->min2, _payload->max2, _payload->type3, _payload->min3, _payload->max3, _payload->type4, _payload->min4, _payload->max4, _payload->type5, _payload->min5, _payload->max5,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5,
    fmav_status_t* _status)
{
    fmav_icarous_kinematic_bands_t* _payload = (fmav_icarous_kinematic_bands_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->min1 = min1;
    _payload->max1 = max1;
    _payload->min2 = min2;
    _payload->max2 = max2;
    _payload->min3 = min3;
    _payload->max3 = max3;
    _payload->min4 = min4;
    _payload->max4 = max4;
    _payload->min5 = min5;
    _payload->max5 = max5;
    _payload->numBands = numBands;
    _payload->type1 = type1;
    _payload->type2 = type2;
    _payload->type3 = type3;
    _payload->type4 = type4;
    _payload->type5 = type5;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_kinematic_bands_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->numBands, _payload->type1, _payload->min1, _payload->max1, _payload->type2, _payload->min2, _payload->max2, _payload->type3, _payload->min3, _payload->max3, _payload->type4, _payload->min4, _payload->max4, _payload->type5, _payload->min5, _payload->max5,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5,
    fmav_status_t* _status)
{
    fmav_icarous_kinematic_bands_t _payload;

    _payload.min1 = min1;
    _payload.max1 = max1;
    _payload.min2 = min2;
    _payload.max2 = max2;
    _payload.min3 = min3;
    _payload.max3 = max3;
    _payload.min4 = min4;
    _payload.max4 = max4;
    _payload.min5 = min5;
    _payload.max5 = max5;
    _payload.numBands = numBands;
    _payload.type1 = type1;
    _payload.type2 = type2;
    _payload.type3 = type3;
    _payload.type4 = type4;
    _payload.type5 = type5;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_kinematic_bands_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ICAROUS_KINEMATIC_BANDS decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_icarous_kinematic_bands_decode(fmav_icarous_kinematic_bands_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_min1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_max1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_min2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_max2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_min3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_max3(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_min4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_max4(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_min5(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_icarous_kinematic_bands_get_field_max5(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_icarous_kinematic_bands_get_field_numBands(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[40]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_kinematic_bands_get_field_type1(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[41]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_kinematic_bands_get_field_type2(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[42]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_kinematic_bands_get_field_type3(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[43]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_kinematic_bands_get_field_type4(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_icarous_kinematic_bands_get_field_type5(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS  42001

#define mavlink_icarous_kinematic_bands_t  fmav_icarous_kinematic_bands_t

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN  46
#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN  46
#define MAVLINK_MSG_ID_42001_LEN  46
#define MAVLINK_MSG_ID_42001_MIN_LEN  46

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC  239
#define MAVLINK_MSG_ID_42001_CRC  239




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_kinematic_bands_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_icarous_kinematic_bands_pack(
        _msg, sysid, compid,
        numBands, type1, min1, max1, type2, min2, max2, type3, min3, max3, type4, min4, max4, type5, min5, max5,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_kinematic_bands_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_icarous_kinematic_bands_t* _payload)
{
    return mavlink_msg_icarous_kinematic_bands_pack(
        sysid,
        compid,
        _msg,
        _payload->numBands, _payload->type1, _payload->min1, _payload->max1, _payload->type2, _payload->min2, _payload->max2, _payload->type3, _payload->min3, _payload->max3, _payload->type4, _payload->min4, _payload->max4, _payload->type5, _payload->min5, _payload->max5);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_kinematic_bands_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
    return fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        numBands, type1, min1, max1, type2, min2, max2, type3, min3, max3, type4, min4, max4, type5, min5, max5,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_icarous_kinematic_bands_decode(const mavlink_message_t* msg, mavlink_icarous_kinematic_bands_t* payload)
{
    fmav_msg_icarous_kinematic_bands_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_H

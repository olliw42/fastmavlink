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

// fields are ordered, as they are on the wire
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


#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MIN  46
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX  46
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN  46
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA  239

#define FASTMAVLINK_MSG_ID_42001_LEN_MIN  46
#define FASTMAVLINK_MSG_ID_42001_LEN_MAX  46
#define FASTMAVLINK_MSG_ID_42001_LEN  46
#define FASTMAVLINK_MSG_ID_42001_CRCEXTRA  239



#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_FLAGS  0
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message ICAROUS_KINEMATIC_BANDS packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5,
    fmav_status_t* _status)
{
    fmav_icarous_kinematic_bands_t* _payload = (fmav_icarous_kinematic_bands_t*)msg->payload;

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


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_kinematic_bands_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_kinematic_bands_pack(
        msg, sysid, compid,
        _payload->numBands, _payload->type1, _payload->min1, _payload->max1, _payload->type2, _payload->min2, _payload->max2, _payload->type3, _payload->min3, _payload->max3, _payload->type4, _payload->min4, _payload->max4, _payload->type5, _payload->min5, _payload->max5,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5,
    fmav_status_t* _status)
{
    fmav_icarous_kinematic_bands_t* _payload = (fmav_icarous_kinematic_bands_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

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


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_icarous_kinematic_bands_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_icarous_kinematic_bands_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->numBands, _payload->type1, _payload->min1, _payload->max1, _payload->type2, _payload->min2, _payload->max2, _payload->type3, _payload->min3, _payload->max3, _payload->type4, _payload->min4, _payload->max4, _payload->type5, _payload->min5, _payload->max5,
        _status);
}


//----------------------------------------
//-- Message ICAROUS_KINEMATIC_BANDS unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_icarous_kinematic_bands_decode(fmav_icarous_kinematic_bands_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_ICAROUS_KINEMATIC_BANDS_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
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
    mavlink_message_t* msg,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_icarous_kinematic_bands_pack(
        msg, sysid, compid,
        numBands, type1, min1, max1, type2, min2, max2, type3, min3, max3, type4, min4, max4, type5, min5, max5,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_icarous_kinematic_bands_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
    return fmav_msg_icarous_kinematic_bands_pack_to_frame_buf(
        (uint8_t*)buf,
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

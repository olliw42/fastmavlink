//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ASLCTRL_DATA_H
#define FASTMAVLINK_MSG_ASLCTRL_DATA_H


//----------------------------------------
//-- Message ASLCTRL_DATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_aslctrl_data_t {
    uint64_t timestamp;
    float h;
    float hRef;
    float hRef_t;
    float PitchAngle;
    float PitchAngleRef;
    float q;
    float qRef;
    float uElev;
    float uThrot;
    float uThrot2;
    float nZ;
    float AirspeedRef;
    float YawAngle;
    float YawAngleRef;
    float RollAngle;
    float RollAngleRef;
    float p;
    float pRef;
    float r;
    float rRef;
    float uAil;
    float uRud;
    uint8_t aslctrl_mode;
    uint8_t SpoilersEngaged;
}) fmav_aslctrl_data_t;


#define FASTMAVLINK_MSG_ID_ASLCTRL_DATA  8004

#define FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX  98
#define FASTMAVLINK_MSG_ASLCTRL_DATA_CRCEXTRA  172

#define FASTMAVLINK_MSG_ASLCTRL_DATA_FLAGS  0
#define FASTMAVLINK_MSG_ASLCTRL_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_ASLCTRL_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_ASLCTRL_DATA_FRAME_LEN_MAX  123



#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_H_OFS  8
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_HREF_OFS  12
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_HREF_T_OFS  16
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_PITCHANGLE_OFS  20
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_PITCHANGLEREF_OFS  24
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_Q_OFS  28
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_QREF_OFS  32
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_UELEV_OFS  36
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_UTHROT_OFS  40
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_UTHROT2_OFS  44
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_NZ_OFS  48
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_AIRSPEEDREF_OFS  52
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_YAWANGLE_OFS  56
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_YAWANGLEREF_OFS  60
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_ROLLANGLE_OFS  64
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_ROLLANGLEREF_OFS  68
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_P_OFS  72
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_PREF_OFS  76
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_R_OFS  80
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_RREF_OFS  84
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_UAIL_OFS  88
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_URUD_OFS  92
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_ASLCTRL_MODE_OFS  96
#define FASTMAVLINK_MSG_ASLCTRL_DATA_FIELD_SPOILERSENGAGED_OFS  97


//----------------------------------------
//-- Message ASLCTRL_DATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud,
    fmav_status_t* _status)
{
    fmav_aslctrl_data_t* _payload = (fmav_aslctrl_data_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->h = h;
    _payload->hRef = hRef;
    _payload->hRef_t = hRef_t;
    _payload->PitchAngle = PitchAngle;
    _payload->PitchAngleRef = PitchAngleRef;
    _payload->q = q;
    _payload->qRef = qRef;
    _payload->uElev = uElev;
    _payload->uThrot = uThrot;
    _payload->uThrot2 = uThrot2;
    _payload->nZ = nZ;
    _payload->AirspeedRef = AirspeedRef;
    _payload->YawAngle = YawAngle;
    _payload->YawAngleRef = YawAngleRef;
    _payload->RollAngle = RollAngle;
    _payload->RollAngleRef = RollAngleRef;
    _payload->p = p;
    _payload->pRef = pRef;
    _payload->r = r;
    _payload->rRef = rRef;
    _payload->uAil = uAil;
    _payload->uRud = uRud;
    _payload->aslctrl_mode = aslctrl_mode;
    _payload->SpoilersEngaged = SpoilersEngaged;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_ASLCTRL_DATA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_ASLCTRL_DATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aslctrl_data_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->aslctrl_mode, _payload->h, _payload->hRef, _payload->hRef_t, _payload->PitchAngle, _payload->PitchAngleRef, _payload->q, _payload->qRef, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->nZ, _payload->AirspeedRef, _payload->SpoilersEngaged, _payload->YawAngle, _payload->YawAngleRef, _payload->RollAngle, _payload->RollAngleRef, _payload->p, _payload->pRef, _payload->r, _payload->rRef, _payload->uAil, _payload->uRud,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud,
    fmav_status_t* _status)
{
    fmav_aslctrl_data_t* _payload = (fmav_aslctrl_data_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->h = h;
    _payload->hRef = hRef;
    _payload->hRef_t = hRef_t;
    _payload->PitchAngle = PitchAngle;
    _payload->PitchAngleRef = PitchAngleRef;
    _payload->q = q;
    _payload->qRef = qRef;
    _payload->uElev = uElev;
    _payload->uThrot = uThrot;
    _payload->uThrot2 = uThrot2;
    _payload->nZ = nZ;
    _payload->AirspeedRef = AirspeedRef;
    _payload->YawAngle = YawAngle;
    _payload->YawAngleRef = YawAngleRef;
    _payload->RollAngle = RollAngle;
    _payload->RollAngleRef = RollAngleRef;
    _payload->p = p;
    _payload->pRef = pRef;
    _payload->r = r;
    _payload->rRef = rRef;
    _payload->uAil = uAil;
    _payload->uRud = uRud;
    _payload->aslctrl_mode = aslctrl_mode;
    _payload->SpoilersEngaged = SpoilersEngaged;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_ASLCTRL_DATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLCTRL_DATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_ASLCTRL_DATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_aslctrl_data_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->aslctrl_mode, _payload->h, _payload->hRef, _payload->hRef_t, _payload->PitchAngle, _payload->PitchAngleRef, _payload->q, _payload->qRef, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->nZ, _payload->AirspeedRef, _payload->SpoilersEngaged, _payload->YawAngle, _payload->YawAngleRef, _payload->RollAngle, _payload->RollAngleRef, _payload->p, _payload->pRef, _payload->r, _payload->rRef, _payload->uAil, _payload->uRud,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud,
    fmav_status_t* _status)
{
    fmav_aslctrl_data_t _payload;

    _payload.timestamp = timestamp;
    _payload.h = h;
    _payload.hRef = hRef;
    _payload.hRef_t = hRef_t;
    _payload.PitchAngle = PitchAngle;
    _payload.PitchAngleRef = PitchAngleRef;
    _payload.q = q;
    _payload.qRef = qRef;
    _payload.uElev = uElev;
    _payload.uThrot = uThrot;
    _payload.uThrot2 = uThrot2;
    _payload.nZ = nZ;
    _payload.AirspeedRef = AirspeedRef;
    _payload.YawAngle = YawAngle;
    _payload.YawAngleRef = YawAngleRef;
    _payload.RollAngle = RollAngle;
    _payload.RollAngleRef = RollAngleRef;
    _payload.p = p;
    _payload.pRef = pRef;
    _payload.r = r;
    _payload.rRef = rRef;
    _payload.uAil = uAil;
    _payload.uRud = uRud;
    _payload.aslctrl_mode = aslctrl_mode;
    _payload.SpoilersEngaged = SpoilersEngaged;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_ASLCTRL_DATA,
        FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_aslctrl_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_aslctrl_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_ASLCTRL_DATA,
        FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_ASLCTRL_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ASLCTRL_DATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_aslctrl_data_decode(fmav_aslctrl_data_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_ASLCTRL_DATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_aslctrl_data_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_h(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_hRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_hRef_t(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_PitchAngle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_PitchAngleRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_q(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_qRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_uElev(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_uThrot(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_uThrot2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_nZ(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_AirspeedRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_YawAngle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_YawAngleRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_RollAngle(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_RollAngleRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_p(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_pRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[76]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_r(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[80]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_rRef(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[84]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_uAil(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[88]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_aslctrl_data_get_field_uRud(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[92]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_aslctrl_data_get_field_aslctrl_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[96]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_aslctrl_data_get_field_SpoilersEngaged(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[97]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_ASLCTRL_DATA  8004

#define mavlink_aslctrl_data_t  fmav_aslctrl_data_t

#define MAVLINK_MSG_ID_ASLCTRL_DATA_LEN  98
#define MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN  98
#define MAVLINK_MSG_ID_8004_LEN  98
#define MAVLINK_MSG_ID_8004_MIN_LEN  98

#define MAVLINK_MSG_ID_ASLCTRL_DATA_CRC  172
#define MAVLINK_MSG_ID_8004_CRC  172




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_aslctrl_data_pack(
        _msg, sysid, compid,
        timestamp, aslctrl_mode, h, hRef, hRef_t, PitchAngle, PitchAngleRef, q, qRef, uElev, uThrot, uThrot2, nZ, AirspeedRef, SpoilersEngaged, YawAngle, YawAngleRef, RollAngle, RollAngleRef, p, pRef, r, rRef, uAil, uRud,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_data_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_aslctrl_data_t* _payload)
{
    return mavlink_msg_aslctrl_data_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->aslctrl_mode, _payload->h, _payload->hRef, _payload->hRef_t, _payload->PitchAngle, _payload->PitchAngleRef, _payload->q, _payload->qRef, _payload->uElev, _payload->uThrot, _payload->uThrot2, _payload->nZ, _payload->AirspeedRef, _payload->SpoilersEngaged, _payload->YawAngle, _payload->YawAngleRef, _payload->RollAngle, _payload->RollAngleRef, _payload->p, _payload->pRef, _payload->r, _payload->rRef, _payload->uAil, _payload->uRud);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_aslctrl_data_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
    return fmav_msg_aslctrl_data_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, aslctrl_mode, h, hRef, hRef_t, PitchAngle, PitchAngleRef, q, qRef, uElev, uThrot, uThrot2, nZ, AirspeedRef, SpoilersEngaged, YawAngle, YawAngleRef, RollAngle, RollAngleRef, p, pRef, r, rRef, uAil, uRud,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_aslctrl_data_decode(const mavlink_message_t* msg, mavlink_aslctrl_data_t* payload)
{
    fmav_msg_aslctrl_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_ASLCTRL_DATA_H

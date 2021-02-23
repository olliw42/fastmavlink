//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_H
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_H


//----------------------------------------
//-- Message AIRSPEED_AUTOCAL
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_airspeed_autocal_t {
    float vx;
    float vy;
    float vz;
    float diff_pressure;
    float EAS2TAS;
    float ratio;
    float state_x;
    float state_y;
    float state_z;
    float Pax;
    float Pby;
    float Pcz;
}) fmav_airspeed_autocal_t;


#define FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL  174


#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MIN  48
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX  48
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN  48
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA  167

#define FASTMAVLINK_MSG_ID_174_LEN_MIN  48
#define FASTMAVLINK_MSG_ID_174_LEN_MAX  48
#define FASTMAVLINK_MSG_ID_174_LEN  48
#define FASTMAVLINK_MSG_ID_174_CRCEXTRA  167



#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FLAGS  0
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message AIRSPEED_AUTOCAL packing routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz,
    fmav_status_t* _status)
{
    fmav_airspeed_autocal_t* _payload = (fmav_airspeed_autocal_t*)msg->payload;

    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->diff_pressure = diff_pressure;
    _payload->EAS2TAS = EAS2TAS;
    _payload->ratio = ratio;
    _payload->state_x = state_x;
    _payload->state_y = state_y;
    _payload->state_z = state_z;
    _payload->Pax = Pax;
    _payload->Pby = Pby;
    _payload->Pcz = Pcz;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_autocal_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airspeed_autocal_pack(
        msg, sysid, compid,
        _payload->vx, _payload->vy, _payload->vz, _payload->diff_pressure, _payload->EAS2TAS, _payload->ratio, _payload->state_x, _payload->state_y, _payload->state_z, _payload->Pax, _payload->Pby, _payload->Pcz,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz,
    fmav_status_t* _status)
{
    fmav_airspeed_autocal_t* _payload = (fmav_airspeed_autocal_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->diff_pressure = diff_pressure;
    _payload->EAS2TAS = EAS2TAS;
    _payload->ratio = ratio;
    _payload->state_x = state_x;
    _payload->state_y = state_y;
    _payload->state_z = state_z;
    _payload->Pax = Pax;
    _payload->Pby = Pby;
    _payload->Pcz = Pcz;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_autocal_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_airspeed_autocal_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->vx, _payload->vy, _payload->vz, _payload->diff_pressure, _payload->EAS2TAS, _payload->ratio, _payload->state_x, _payload->state_y, _payload->state_z, _payload->Pax, _payload->Pby, _payload->Pcz,
        _status);
}


//----------------------------------------
//-- Message AIRSPEED_AUTOCAL unpacking routines
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airspeed_autocal_decode(fmav_airspeed_autocal_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL  174

#define mavlink_airspeed_autocal_t  fmav_airspeed_autocal_t

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN  48
#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN  48
#define MAVLINK_MSG_ID_174_LEN  48
#define MAVLINK_MSG_ID_174_MIN_LEN  48

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC  167
#define MAVLINK_MSG_ID_174_CRC  167




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_autocal_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_airspeed_autocal_pack(
        msg, sysid, compid,
        vx, vy, vz, diff_pressure, EAS2TAS, ratio, state_x, state_y, state_z, Pax, Pby, Pcz,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_airspeed_autocal_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
    return fmav_msg_airspeed_autocal_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        vx, vy, vz, diff_pressure, EAS2TAS, ratio, state_x, state_y, state_z, Pax, Pby, Pcz,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_airspeed_autocal_decode(const mavlink_message_t* msg, mavlink_airspeed_autocal_t* payload)
{
    fmav_msg_airspeed_autocal_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_H

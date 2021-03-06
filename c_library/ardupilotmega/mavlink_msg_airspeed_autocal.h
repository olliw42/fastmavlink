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

#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX  48
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA  167

#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FLAGS  0
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FRAME_LEN_MAX  73



#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_VX_OFS  0
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_VY_OFS  4
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_VZ_OFS  8
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_DIFF_PRESSURE_OFS  12
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_EAS2TAS_OFS  16
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_RATIO_OFS  20
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_STATE_X_OFS  24
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_STATE_Y_OFS  28
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_STATE_Z_OFS  32
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_PAX_OFS  36
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_PBY_OFS  40
#define FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_FIELD_PCZ_OFS  44


//----------------------------------------
//-- Message AIRSPEED_AUTOCAL packing routines, for sending
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


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz,
    fmav_status_t* _status)
{
    fmav_airspeed_autocal_t _payload;

    _payload.vx = vx;
    _payload.vy = vy;
    _payload.vz = vz;
    _payload.diff_pressure = diff_pressure;
    _payload.EAS2TAS = EAS2TAS;
    _payload.ratio = ratio;
    _payload.state_x = state_x;
    _payload.state_y = state_y;
    _payload.state_z = state_z;
    _payload.Pax = Pax;
    _payload.Pby = Pby;
    _payload.Pcz = Pcz;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_airspeed_autocal_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_airspeed_autocal_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_AIRSPEED_AUTOCAL,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message AIRSPEED_AUTOCAL unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_airspeed_autocal_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_airspeed_autocal_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airspeed_autocal_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_airspeed_autocal_decode(fmav_airspeed_autocal_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_AIRSPEED_AUTOCAL_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_vx(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[0]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_vy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[4]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_vz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_diff_pressure(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_EAS2TAS(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_ratio(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_state_x(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_state_y(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_state_z(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_Pax(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_Pby(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_airspeed_autocal_get_field_Pcz(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
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

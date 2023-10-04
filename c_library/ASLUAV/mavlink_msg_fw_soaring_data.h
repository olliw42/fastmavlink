//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_FW_SOARING_DATA_H
#define FASTMAVLINK_MSG_FW_SOARING_DATA_H


//----------------------------------------
//-- Message FW_SOARING_DATA
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_fw_soaring_data_t {
    uint64_t timestamp;
    uint64_t timestampModeChanged;
    float xW;
    float xR;
    float xLat;
    float xLon;
    float VarW;
    float VarR;
    float VarLat;
    float VarLon;
    float LoiterRadius;
    float LoiterDirection;
    float DistToSoarPoint;
    float vSinkExp;
    float z1_LocalUpdraftSpeed;
    float z2_DeltaRoll;
    float z1_exp;
    float z2_exp;
    float ThermalGSNorth;
    float ThermalGSEast;
    float TSE_dot;
    float DebugVar1;
    float DebugVar2;
    uint8_t ControlMode;
    uint8_t valid;
}) fmav_fw_soaring_data_t;


#define FASTMAVLINK_MSG_ID_FW_SOARING_DATA  8011

#define FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX  102
#define FASTMAVLINK_MSG_FW_SOARING_DATA_CRCEXTRA  20

#define FASTMAVLINK_MSG_FW_SOARING_DATA_FLAGS  0
#define FASTMAVLINK_MSG_FW_SOARING_DATA_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_FW_SOARING_DATA_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_FW_SOARING_DATA_FRAME_LEN_MAX  127



#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_TIMESTAMPMODECHANGED_OFS  8
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_XW_OFS  16
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_XR_OFS  20
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_XLAT_OFS  24
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_XLON_OFS  28
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VARW_OFS  32
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VARR_OFS  36
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VARLAT_OFS  40
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VARLON_OFS  44
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_LOITERRADIUS_OFS  48
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_LOITERDIRECTION_OFS  52
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_DISTTOSOARPOINT_OFS  56
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VSINKEXP_OFS  60
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_Z1_LOCALUPDRAFTSPEED_OFS  64
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_Z2_DELTAROLL_OFS  68
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_Z1_EXP_OFS  72
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_Z2_EXP_OFS  76
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_THERMALGSNORTH_OFS  80
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_THERMALGSEAST_OFS  84
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_TSE_DOT_OFS  88
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_DEBUGVAR1_OFS  92
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_DEBUGVAR2_OFS  96
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_CONTROLMODE_OFS  100
#define FASTMAVLINK_MSG_FW_SOARING_DATA_FIELD_VALID_OFS  101


//----------------------------------------
//-- Message FW_SOARING_DATA pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid,
    fmav_status_t* _status)
{
    fmav_fw_soaring_data_t* _payload = (fmav_fw_soaring_data_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->timestampModeChanged = timestampModeChanged;
    _payload->xW = xW;
    _payload->xR = xR;
    _payload->xLat = xLat;
    _payload->xLon = xLon;
    _payload->VarW = VarW;
    _payload->VarR = VarR;
    _payload->VarLat = VarLat;
    _payload->VarLon = VarLon;
    _payload->LoiterRadius = LoiterRadius;
    _payload->LoiterDirection = LoiterDirection;
    _payload->DistToSoarPoint = DistToSoarPoint;
    _payload->vSinkExp = vSinkExp;
    _payload->z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    _payload->z2_DeltaRoll = z2_DeltaRoll;
    _payload->z1_exp = z1_exp;
    _payload->z2_exp = z2_exp;
    _payload->ThermalGSNorth = ThermalGSNorth;
    _payload->ThermalGSEast = ThermalGSEast;
    _payload->TSE_dot = TSE_dot;
    _payload->DebugVar1 = DebugVar1;
    _payload->DebugVar2 = DebugVar2;
    _payload->ControlMode = ControlMode;
    _payload->valid = valid;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_FW_SOARING_DATA;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_FW_SOARING_DATA_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fw_soaring_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fw_soaring_data_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->timestampModeChanged, _payload->xW, _payload->xR, _payload->xLat, _payload->xLon, _payload->VarW, _payload->VarR, _payload->VarLat, _payload->VarLon, _payload->LoiterRadius, _payload->LoiterDirection, _payload->DistToSoarPoint, _payload->vSinkExp, _payload->z1_LocalUpdraftSpeed, _payload->z2_DeltaRoll, _payload->z1_exp, _payload->z2_exp, _payload->ThermalGSNorth, _payload->ThermalGSEast, _payload->TSE_dot, _payload->DebugVar1, _payload->DebugVar2, _payload->ControlMode, _payload->valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid,
    fmav_status_t* _status)
{
    fmav_fw_soaring_data_t* _payload = (fmav_fw_soaring_data_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->timestampModeChanged = timestampModeChanged;
    _payload->xW = xW;
    _payload->xR = xR;
    _payload->xLat = xLat;
    _payload->xLon = xLon;
    _payload->VarW = VarW;
    _payload->VarR = VarR;
    _payload->VarLat = VarLat;
    _payload->VarLon = VarLon;
    _payload->LoiterRadius = LoiterRadius;
    _payload->LoiterDirection = LoiterDirection;
    _payload->DistToSoarPoint = DistToSoarPoint;
    _payload->vSinkExp = vSinkExp;
    _payload->z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    _payload->z2_DeltaRoll = z2_DeltaRoll;
    _payload->z1_exp = z1_exp;
    _payload->z2_exp = z2_exp;
    _payload->ThermalGSNorth = ThermalGSNorth;
    _payload->ThermalGSEast = ThermalGSEast;
    _payload->TSE_dot = TSE_dot;
    _payload->DebugVar1 = DebugVar1;
    _payload->DebugVar2 = DebugVar2;
    _payload->ControlMode = ControlMode;
    _payload->valid = valid;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_FW_SOARING_DATA;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_FW_SOARING_DATA >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_FW_SOARING_DATA >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FW_SOARING_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_fw_soaring_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_fw_soaring_data_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->timestampModeChanged, _payload->xW, _payload->xR, _payload->xLat, _payload->xLon, _payload->VarW, _payload->VarR, _payload->VarLat, _payload->VarLon, _payload->LoiterRadius, _payload->LoiterDirection, _payload->DistToSoarPoint, _payload->vSinkExp, _payload->z1_LocalUpdraftSpeed, _payload->z2_DeltaRoll, _payload->z1_exp, _payload->z2_exp, _payload->ThermalGSNorth, _payload->ThermalGSEast, _payload->TSE_dot, _payload->DebugVar1, _payload->DebugVar2, _payload->ControlMode, _payload->valid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid,
    fmav_status_t* _status)
{
    fmav_fw_soaring_data_t _payload;

    _payload.timestamp = timestamp;
    _payload.timestampModeChanged = timestampModeChanged;
    _payload.xW = xW;
    _payload.xR = xR;
    _payload.xLat = xLat;
    _payload.xLon = xLon;
    _payload.VarW = VarW;
    _payload.VarR = VarR;
    _payload.VarLat = VarLat;
    _payload.VarLon = VarLon;
    _payload.LoiterRadius = LoiterRadius;
    _payload.LoiterDirection = LoiterDirection;
    _payload.DistToSoarPoint = DistToSoarPoint;
    _payload.vSinkExp = vSinkExp;
    _payload.z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    _payload.z2_DeltaRoll = z2_DeltaRoll;
    _payload.z1_exp = z1_exp;
    _payload.z2_exp = z2_exp;
    _payload.ThermalGSNorth = ThermalGSNorth;
    _payload.ThermalGSEast = ThermalGSEast;
    _payload.TSE_dot = TSE_dot;
    _payload.DebugVar1 = DebugVar1;
    _payload.DebugVar2 = DebugVar2;
    _payload.ControlMode = ControlMode;
    _payload.valid = valid;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_FW_SOARING_DATA,
        FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FW_SOARING_DATA_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_fw_soaring_data_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_fw_soaring_data_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_FW_SOARING_DATA,
        FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_FW_SOARING_DATA_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message FW_SOARING_DATA decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_fw_soaring_data_decode(fmav_fw_soaring_data_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_FW_SOARING_DATA_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_fw_soaring_data_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_fw_soaring_data_get_field_timestampModeChanged(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_xW(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_xR(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_xLat(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_xLon(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_VarW(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_VarR(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_VarLat(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_VarLon(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_LoiterRadius(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_LoiterDirection(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_DistToSoarPoint(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[56]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_vSinkExp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[60]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_z1_LocalUpdraftSpeed(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[64]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_z2_DeltaRoll(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[68]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_z1_exp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[72]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_z2_exp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[76]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_ThermalGSNorth(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[80]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_ThermalGSEast(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[84]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_TSE_dot(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[88]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_DebugVar1(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[92]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_fw_soaring_data_get_field_DebugVar2(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[96]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_fw_soaring_data_get_field_ControlMode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[100]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_fw_soaring_data_get_field_valid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[101]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_FW_SOARING_DATA  8011

#define mavlink_fw_soaring_data_t  fmav_fw_soaring_data_t

#define MAVLINK_MSG_ID_FW_SOARING_DATA_LEN  102
#define MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN  102
#define MAVLINK_MSG_ID_8011_LEN  102
#define MAVLINK_MSG_ID_8011_MIN_LEN  102

#define MAVLINK_MSG_ID_FW_SOARING_DATA_CRC  20
#define MAVLINK_MSG_ID_8011_CRC  20




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fw_soaring_data_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_fw_soaring_data_pack(
        _msg, sysid, compid,
        timestamp, timestampModeChanged, xW, xR, xLat, xLon, VarW, VarR, VarLat, VarLon, LoiterRadius, LoiterDirection, DistToSoarPoint, vSinkExp, z1_LocalUpdraftSpeed, z2_DeltaRoll, z1_exp, z2_exp, ThermalGSNorth, ThermalGSEast, TSE_dot, DebugVar1, DebugVar2, ControlMode, valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fw_soaring_data_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_fw_soaring_data_t* _payload)
{
    return mavlink_msg_fw_soaring_data_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->timestampModeChanged, _payload->xW, _payload->xR, _payload->xLat, _payload->xLon, _payload->VarW, _payload->VarR, _payload->VarLat, _payload->VarLon, _payload->LoiterRadius, _payload->LoiterDirection, _payload->DistToSoarPoint, _payload->vSinkExp, _payload->z1_LocalUpdraftSpeed, _payload->z2_DeltaRoll, _payload->z1_exp, _payload->z2_exp, _payload->ThermalGSNorth, _payload->ThermalGSEast, _payload->TSE_dot, _payload->DebugVar1, _payload->DebugVar2, _payload->ControlMode, _payload->valid);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_fw_soaring_data_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
    return fmav_msg_fw_soaring_data_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, timestampModeChanged, xW, xR, xLat, xLon, VarW, VarR, VarLat, VarLon, LoiterRadius, LoiterDirection, DistToSoarPoint, vSinkExp, z1_LocalUpdraftSpeed, z2_DeltaRoll, z1_exp, z2_exp, ThermalGSNorth, ThermalGSEast, TSE_dot, DebugVar1, DebugVar2, ControlMode, valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_fw_soaring_data_decode(const mavlink_message_t* msg, mavlink_fw_soaring_data_t* payload)
{
    fmav_msg_fw_soaring_data_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_FW_SOARING_DATA_H

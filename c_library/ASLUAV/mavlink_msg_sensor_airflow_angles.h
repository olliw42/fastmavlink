//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_H
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_H


//----------------------------------------
//-- Message SENSOR_AIRFLOW_ANGLES
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sensor_airflow_angles_t {
    uint64_t timestamp;
    float angleofattack;
    float sideslip;
    uint8_t angleofattack_valid;
    uint8_t sideslip_valid;
}) fmav_sensor_airflow_angles_t;


#define FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES  8016

#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX  18
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_CRCEXTRA  149

#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FLAGS  0
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FRAME_LEN_MAX  43



#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FIELD_ANGLEOFATTACK_OFS  8
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FIELD_SIDESLIP_OFS  12
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FIELD_ANGLEOFATTACK_VALID_OFS  16
#define FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_FIELD_SIDESLIP_VALID_OFS  17


//----------------------------------------
//-- Message SENSOR_AIRFLOW_ANGLES pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid,
    fmav_status_t* _status)
{
    fmav_sensor_airflow_angles_t* _payload = (fmav_sensor_airflow_angles_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->angleofattack = angleofattack;
    _payload->sideslip = sideslip;
    _payload->angleofattack_valid = angleofattack_valid;
    _payload->sideslip_valid = sideslip_valid;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_airflow_angles_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_airflow_angles_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->angleofattack, _payload->angleofattack_valid, _payload->sideslip, _payload->sideslip_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid,
    fmav_status_t* _status)
{
    fmav_sensor_airflow_angles_t* _payload = (fmav_sensor_airflow_angles_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->angleofattack = angleofattack;
    _payload->sideslip = sideslip;
    _payload->angleofattack_valid = angleofattack_valid;
    _payload->sideslip_valid = sideslip_valid;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_airflow_angles_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sensor_airflow_angles_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->angleofattack, _payload->angleofattack_valid, _payload->sideslip, _payload->sideslip_valid,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid,
    fmav_status_t* _status)
{
    fmav_sensor_airflow_angles_t _payload;

    _payload.timestamp = timestamp;
    _payload.angleofattack = angleofattack;
    _payload.sideslip = sideslip;
    _payload.angleofattack_valid = angleofattack_valid;
    _payload.sideslip_valid = sideslip_valid;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sensor_airflow_angles_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sensor_airflow_angles_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENSOR_AIRFLOW_ANGLES decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sensor_airflow_angles_decode(fmav_sensor_airflow_angles_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_sensor_airflow_angles_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_airflow_angles_get_field_angleofattack(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sensor_airflow_angles_get_field_sideslip(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensor_airflow_angles_get_field_angleofattack_valid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sensor_airflow_angles_get_field_sideslip_valid(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[17]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES  8016

#define mavlink_sensor_airflow_angles_t  fmav_sensor_airflow_angles_t

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN  18
#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN  18
#define MAVLINK_MSG_ID_8016_LEN  18
#define MAVLINK_MSG_ID_8016_MIN_LEN  18

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC  149
#define MAVLINK_MSG_ID_8016_CRC  149




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_airflow_angles_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sensor_airflow_angles_pack(
        _msg, sysid, compid,
        timestamp, angleofattack, angleofattack_valid, sideslip, sideslip_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_airflow_angles_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sensor_airflow_angles_t* _payload)
{
    return mavlink_msg_sensor_airflow_angles_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->angleofattack, _payload->angleofattack_valid, _payload->sideslip, _payload->sideslip_valid);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sensor_airflow_angles_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
    return fmav_msg_sensor_airflow_angles_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, angleofattack, angleofattack_valid, sideslip, sideslip_valid,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sensor_airflow_angles_decode(const mavlink_message_t* msg, mavlink_sensor_airflow_angles_t* payload)
{
    fmav_msg_sensor_airflow_angles_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENSOR_AIRFLOW_ANGLES_H

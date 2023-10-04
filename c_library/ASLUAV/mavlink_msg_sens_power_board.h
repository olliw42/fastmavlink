//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_SENS_POWER_BOARD_H
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_H


//----------------------------------------
//-- Message SENS_POWER_BOARD
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_sens_power_board_t {
    uint64_t timestamp;
    float pwr_brd_system_volt;
    float pwr_brd_servo_volt;
    float pwr_brd_digital_volt;
    float pwr_brd_mot_l_amp;
    float pwr_brd_mot_r_amp;
    float pwr_brd_analog_amp;
    float pwr_brd_digital_amp;
    float pwr_brd_ext_amp;
    float pwr_brd_aux_amp;
    uint8_t pwr_brd_status;
    uint8_t pwr_brd_led_status;
}) fmav_sens_power_board_t;


#define FASTMAVLINK_MSG_ID_SENS_POWER_BOARD  8013

#define FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX  46
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_CRCEXTRA  222

#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FLAGS  0
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FRAME_LEN_MAX  71



#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_TIMESTAMP_OFS  0
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_SYSTEM_VOLT_OFS  8
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_SERVO_VOLT_OFS  12
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_DIGITAL_VOLT_OFS  16
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_MOT_L_AMP_OFS  20
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_MOT_R_AMP_OFS  24
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_ANALOG_AMP_OFS  28
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_DIGITAL_AMP_OFS  32
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_EXT_AMP_OFS  36
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_AUX_AMP_OFS  40
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_STATUS_OFS  44
#define FASTMAVLINK_MSG_SENS_POWER_BOARD_FIELD_PWR_BRD_LED_STATUS_OFS  45


//----------------------------------------
//-- Message SENS_POWER_BOARD pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_board_t* _payload = (fmav_sens_power_board_t*)_msg->payload;

    _payload->timestamp = timestamp;
    _payload->pwr_brd_system_volt = pwr_brd_system_volt;
    _payload->pwr_brd_servo_volt = pwr_brd_servo_volt;
    _payload->pwr_brd_digital_volt = pwr_brd_digital_volt;
    _payload->pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    _payload->pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    _payload->pwr_brd_analog_amp = pwr_brd_analog_amp;
    _payload->pwr_brd_digital_amp = pwr_brd_digital_amp;
    _payload->pwr_brd_ext_amp = pwr_brd_ext_amp;
    _payload->pwr_brd_aux_amp = pwr_brd_aux_amp;
    _payload->pwr_brd_status = pwr_brd_status;
    _payload->pwr_brd_led_status = pwr_brd_led_status;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_SENS_POWER_BOARD;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_SENS_POWER_BOARD_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_board_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_power_board_pack(
        _msg, sysid, compid,
        _payload->timestamp, _payload->pwr_brd_status, _payload->pwr_brd_led_status, _payload->pwr_brd_system_volt, _payload->pwr_brd_servo_volt, _payload->pwr_brd_digital_volt, _payload->pwr_brd_mot_l_amp, _payload->pwr_brd_mot_r_amp, _payload->pwr_brd_analog_amp, _payload->pwr_brd_digital_amp, _payload->pwr_brd_ext_amp, _payload->pwr_brd_aux_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_board_t* _payload = (fmav_sens_power_board_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->timestamp = timestamp;
    _payload->pwr_brd_system_volt = pwr_brd_system_volt;
    _payload->pwr_brd_servo_volt = pwr_brd_servo_volt;
    _payload->pwr_brd_digital_volt = pwr_brd_digital_volt;
    _payload->pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    _payload->pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    _payload->pwr_brd_analog_amp = pwr_brd_analog_amp;
    _payload->pwr_brd_digital_amp = pwr_brd_digital_amp;
    _payload->pwr_brd_ext_amp = pwr_brd_ext_amp;
    _payload->pwr_brd_aux_amp = pwr_brd_aux_amp;
    _payload->pwr_brd_status = pwr_brd_status;
    _payload->pwr_brd_led_status = pwr_brd_led_status;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_SENS_POWER_BOARD;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_POWER_BOARD >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_SENS_POWER_BOARD >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_board_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_sens_power_board_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->timestamp, _payload->pwr_brd_status, _payload->pwr_brd_led_status, _payload->pwr_brd_system_volt, _payload->pwr_brd_servo_volt, _payload->pwr_brd_digital_volt, _payload->pwr_brd_mot_l_amp, _payload->pwr_brd_mot_r_amp, _payload->pwr_brd_analog_amp, _payload->pwr_brd_digital_amp, _payload->pwr_brd_ext_amp, _payload->pwr_brd_aux_amp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp,
    fmav_status_t* _status)
{
    fmav_sens_power_board_t _payload;

    _payload.timestamp = timestamp;
    _payload.pwr_brd_system_volt = pwr_brd_system_volt;
    _payload.pwr_brd_servo_volt = pwr_brd_servo_volt;
    _payload.pwr_brd_digital_volt = pwr_brd_digital_volt;
    _payload.pwr_brd_mot_l_amp = pwr_brd_mot_l_amp;
    _payload.pwr_brd_mot_r_amp = pwr_brd_mot_r_amp;
    _payload.pwr_brd_analog_amp = pwr_brd_analog_amp;
    _payload.pwr_brd_digital_amp = pwr_brd_digital_amp;
    _payload.pwr_brd_ext_amp = pwr_brd_ext_amp;
    _payload.pwr_brd_aux_amp = pwr_brd_aux_amp;
    _payload.pwr_brd_status = pwr_brd_status;
    _payload.pwr_brd_led_status = pwr_brd_led_status;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_SENS_POWER_BOARD,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_sens_power_board_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_sens_power_board_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_SENS_POWER_BOARD,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_SENS_POWER_BOARD_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message SENS_POWER_BOARD decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_sens_power_board_decode(fmav_sens_power_board_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_SENS_POWER_BOARD_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_sens_power_board_get_field_timestamp(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_system_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[8]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_servo_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[12]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_digital_volt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[16]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_mot_l_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_mot_r_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_analog_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_digital_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_ext_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_sens_power_board_get_field_pwr_brd_aux_amp(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_power_board_get_field_pwr_brd_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[44]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_sens_power_board_get_field_pwr_brd_led_status(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[45]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_SENS_POWER_BOARD  8013

#define mavlink_sens_power_board_t  fmav_sens_power_board_t

#define MAVLINK_MSG_ID_SENS_POWER_BOARD_LEN  46
#define MAVLINK_MSG_ID_SENS_POWER_BOARD_MIN_LEN  46
#define MAVLINK_MSG_ID_8013_LEN  46
#define MAVLINK_MSG_ID_8013_MIN_LEN  46

#define MAVLINK_MSG_ID_SENS_POWER_BOARD_CRC  222
#define MAVLINK_MSG_ID_8013_CRC  222




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_board_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_sens_power_board_pack(
        _msg, sysid, compid,
        timestamp, pwr_brd_status, pwr_brd_led_status, pwr_brd_system_volt, pwr_brd_servo_volt, pwr_brd_digital_volt, pwr_brd_mot_l_amp, pwr_brd_mot_r_amp, pwr_brd_analog_amp, pwr_brd_digital_amp, pwr_brd_ext_amp, pwr_brd_aux_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_board_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_sens_power_board_t* _payload)
{
    return mavlink_msg_sens_power_board_pack(
        sysid,
        compid,
        _msg,
        _payload->timestamp, _payload->pwr_brd_status, _payload->pwr_brd_led_status, _payload->pwr_brd_system_volt, _payload->pwr_brd_servo_volt, _payload->pwr_brd_digital_volt, _payload->pwr_brd_mot_l_amp, _payload->pwr_brd_mot_r_amp, _payload->pwr_brd_analog_amp, _payload->pwr_brd_digital_amp, _payload->pwr_brd_ext_amp, _payload->pwr_brd_aux_amp);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_sens_power_board_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t timestamp, uint8_t pwr_brd_status, uint8_t pwr_brd_led_status, float pwr_brd_system_volt, float pwr_brd_servo_volt, float pwr_brd_digital_volt, float pwr_brd_mot_l_amp, float pwr_brd_mot_r_amp, float pwr_brd_analog_amp, float pwr_brd_digital_amp, float pwr_brd_ext_amp, float pwr_brd_aux_amp)
{
    return fmav_msg_sens_power_board_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        timestamp, pwr_brd_status, pwr_brd_led_status, pwr_brd_system_volt, pwr_brd_servo_volt, pwr_brd_digital_volt, pwr_brd_mot_l_amp, pwr_brd_mot_r_amp, pwr_brd_analog_amp, pwr_brd_digital_amp, pwr_brd_ext_amp, pwr_brd_aux_amp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_sens_power_board_decode(const mavlink_message_t* msg, mavlink_sens_power_board_t* payload)
{
    fmav_msg_sens_power_board_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_SENS_POWER_BOARD_H

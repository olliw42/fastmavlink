//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_H
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_H


//----------------------------------------
//-- Message MLRS_RADIO_LINK_INFORMATION
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_mlrs_radio_link_information_t {
    uint16_t tx_frame_rate;
    uint16_t rx_frame_rate;
    uint16_t tx_ser_data_rate;
    uint16_t rx_ser_data_rate;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t type;
    uint8_t mode;
    int8_t tx_power;
    int8_t rx_power;
    char mode_str[6];
    char band_str[6];
    uint8_t tx_receive_sensitivity;
    uint8_t rx_receive_sensitivity;
}) fmav_mlrs_radio_link_information_t;


#define FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION  60046

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX  28
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_CRCEXTRA  171

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FLAGS  3
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_TARGET_COMPONENT_OFS  9

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FRAME_LEN_MAX  53

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_STR_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_STR_LEN  6 // length of array = number of bytes
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_BAND_STR_NUM  6 // number of elements in array
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_BAND_STR_LEN  6 // length of array = number of bytes

#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TX_FRAME_RATE_OFS  0
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_RX_FRAME_RATE_OFS  2
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TX_SER_DATA_RATE_OFS  4
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_RX_SER_DATA_RATE_OFS  6
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TARGET_SYSTEM_OFS  8
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TARGET_COMPONENT_OFS  9
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TYPE_OFS  10
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_OFS  11
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TX_POWER_OFS  12
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_RX_POWER_OFS  13
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_STR_OFS  14
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_BAND_STR_OFS  20
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_TX_RECEIVE_SENSITIVITY_OFS  26
#define FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_RX_RECEIVE_SENSITIVITY_OFS  27


//----------------------------------------
//-- Message MLRS_RADIO_LINK_INFORMATION pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, int8_t tx_power, int8_t rx_power, uint16_t tx_frame_rate, uint16_t rx_frame_rate, const char* mode_str, const char* band_str, uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_information_t* _payload = (fmav_mlrs_radio_link_information_t*)_msg->payload;

    _payload->tx_frame_rate = tx_frame_rate;
    _payload->rx_frame_rate = rx_frame_rate;
    _payload->tx_ser_data_rate = tx_ser_data_rate;
    _payload->rx_ser_data_rate = rx_ser_data_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type = type;
    _payload->mode = mode;
    _payload->tx_power = tx_power;
    _payload->rx_power = rx_power;
    _payload->tx_receive_sensitivity = tx_receive_sensitivity;
    _payload->rx_receive_sensitivity = rx_receive_sensitivity;
    memcpy(&(_payload->mode_str), mode_str, sizeof(char)*6);
    memcpy(&(_payload->band_str), band_str, sizeof(char)*6);

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION;
    _msg->target_sysid = target_system;
    _msg->target_compid = target_component;
    _msg->crc_extra = FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mlrs_radio_link_information_pack(
        _msg, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_power, _payload->rx_power, _payload->tx_frame_rate, _payload->rx_frame_rate, _payload->mode_str, _payload->band_str, _payload->tx_ser_data_rate, _payload->rx_ser_data_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, int8_t tx_power, int8_t rx_power, uint16_t tx_frame_rate, uint16_t rx_frame_rate, const char* mode_str, const char* band_str, uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_information_t* _payload = (fmav_mlrs_radio_link_information_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->tx_frame_rate = tx_frame_rate;
    _payload->rx_frame_rate = rx_frame_rate;
    _payload->tx_ser_data_rate = tx_ser_data_rate;
    _payload->rx_ser_data_rate = rx_ser_data_rate;
    _payload->target_system = target_system;
    _payload->target_component = target_component;
    _payload->type = type;
    _payload->mode = mode;
    _payload->tx_power = tx_power;
    _payload->rx_power = rx_power;
    _payload->tx_receive_sensitivity = tx_receive_sensitivity;
    _payload->rx_receive_sensitivity = rx_receive_sensitivity;
    memcpy(&(_payload->mode_str), mode_str, sizeof(char)*6);
    memcpy(&(_payload->band_str), band_str, sizeof(char)*6);

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_mlrs_radio_link_information_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_power, _payload->rx_power, _payload->tx_frame_rate, _payload->rx_frame_rate, _payload->mode_str, _payload->band_str, _payload->tx_ser_data_rate, _payload->rx_ser_data_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, int8_t tx_power, int8_t rx_power, uint16_t tx_frame_rate, uint16_t rx_frame_rate, const char* mode_str, const char* band_str, uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity,
    fmav_status_t* _status)
{
    fmav_mlrs_radio_link_information_t _payload;

    _payload.tx_frame_rate = tx_frame_rate;
    _payload.rx_frame_rate = rx_frame_rate;
    _payload.tx_ser_data_rate = tx_ser_data_rate;
    _payload.rx_ser_data_rate = rx_ser_data_rate;
    _payload.target_system = target_system;
    _payload.target_component = target_component;
    _payload.type = type;
    _payload.mode = mode;
    _payload.tx_power = tx_power;
    _payload.rx_power = rx_power;
    _payload.tx_receive_sensitivity = tx_receive_sensitivity;
    _payload.rx_receive_sensitivity = rx_receive_sensitivity;
    memcpy(&(_payload.mode_str), mode_str, sizeof(char)*6);
    memcpy(&(_payload.band_str), band_str, sizeof(char)*6);

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_mlrs_radio_link_information_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message MLRS_RADIO_LINK_INFORMATION decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_mlrs_radio_link_information_decode(fmav_mlrs_radio_link_information_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_get_field_tx_frame_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_get_field_rx_frame_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[2]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_get_field_tx_ser_data_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_mlrs_radio_link_information_get_field_rx_ser_data_rate(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[6]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_target_system(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_target_component(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[9]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[10]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_mode(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[11]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_mlrs_radio_link_information_get_field_tx_power(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int8_t fmav_msg_mlrs_radio_link_information_get_field_rx_power(const fmav_message_t* msg)
{
    int8_t r;
    memcpy(&r, &(msg->payload[13]), sizeof(int8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_tx_receive_sensitivity(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[26]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_mlrs_radio_link_information_get_field_rx_receive_sensitivity(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[27]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_mlrs_radio_link_information_get_field_mode_str_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[14]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_mlrs_radio_link_information_get_field_mode_str(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_STR_NUM) return 0;
    return ((char*)&(msg->payload[14]))[index];
}


FASTMAVLINK_FUNCTION_DECORATOR char* fmav_msg_mlrs_radio_link_information_get_field_band_str_ptr(const fmav_message_t* msg)
{
    return (char*)&(msg->payload[20]);
}


FASTMAVLINK_FUNCTION_DECORATOR char fmav_msg_mlrs_radio_link_information_get_field_band_str(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_BAND_STR_NUM) return 0;
    return ((char*)&(msg->payload[20]))[index];
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION  60046

#define mavlink_mlrs_radio_link_information_t  fmav_mlrs_radio_link_information_t

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION_LEN  28
#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION_MIN_LEN  28
#define MAVLINK_MSG_ID_60046_LEN  28
#define MAVLINK_MSG_ID_60046_MIN_LEN  28

#define MAVLINK_MSG_ID_MLRS_RADIO_LINK_INFORMATION_CRC  171
#define MAVLINK_MSG_ID_60046_CRC  171

#define MAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_MODE_STR_LEN 6
#define MAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_FIELD_BAND_STR_LEN 6


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_information_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, int8_t tx_power, int8_t rx_power, uint16_t tx_frame_rate, uint16_t rx_frame_rate, const char* mode_str, const char* band_str, uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_mlrs_radio_link_information_pack(
        _msg, sysid, compid,
        target_system, target_component, type, mode, tx_power, rx_power, tx_frame_rate, rx_frame_rate, mode_str, band_str, tx_ser_data_rate, rx_ser_data_rate, tx_receive_sensitivity, rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_information_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_mlrs_radio_link_information_t* _payload)
{
    return mavlink_msg_mlrs_radio_link_information_pack(
        sysid,
        compid,
        _msg,
        _payload->target_system, _payload->target_component, _payload->type, _payload->mode, _payload->tx_power, _payload->rx_power, _payload->tx_frame_rate, _payload->rx_frame_rate, _payload->mode_str, _payload->band_str, _payload->tx_ser_data_rate, _payload->rx_ser_data_rate, _payload->tx_receive_sensitivity, _payload->rx_receive_sensitivity);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_mlrs_radio_link_information_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mode, int8_t tx_power, int8_t rx_power, uint16_t tx_frame_rate, uint16_t rx_frame_rate, const char* mode_str, const char* band_str, uint16_t tx_ser_data_rate, uint16_t rx_ser_data_rate, uint8_t tx_receive_sensitivity, uint8_t rx_receive_sensitivity)
{
    return fmav_msg_mlrs_radio_link_information_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        target_system, target_component, type, mode, tx_power, rx_power, tx_frame_rate, rx_frame_rate, mode_str, band_str, tx_ser_data_rate, rx_ser_data_rate, tx_receive_sensitivity, rx_receive_sensitivity,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_mlrs_radio_link_information_decode(const mavlink_message_t* msg, mavlink_mlrs_radio_link_information_t* payload)
{
    fmav_msg_mlrs_radio_link_information_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_MLRS_RADIO_LINK_INFORMATION_H

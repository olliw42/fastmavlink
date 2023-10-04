//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_HERELINK_TELEM_H
#define FASTMAVLINK_MSG_HERELINK_TELEM_H


//----------------------------------------
//-- Message HERELINK_TELEM
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_herelink_telem_t {
    uint32_t rf_freq;
    uint32_t link_bw;
    uint32_t link_rate;
    int16_t snr;
    int16_t cpu_temp;
    int16_t board_temp;
    uint8_t rssi;
}) fmav_herelink_telem_t;


#define FASTMAVLINK_MSG_ID_HERELINK_TELEM  50003

#define FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX  19
#define FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA  62

#define FASTMAVLINK_MSG_HERELINK_TELEM_FLAGS  0
#define FASTMAVLINK_MSG_HERELINK_TELEM_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_HERELINK_TELEM_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_HERELINK_TELEM_FRAME_LEN_MAX  44



#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_RF_FREQ_OFS  0
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_LINK_BW_OFS  4
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_LINK_RATE_OFS  8
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_SNR_OFS  12
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_CPU_TEMP_OFS  14
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_BOARD_TEMP_OFS  16
#define FASTMAVLINK_MSG_HERELINK_TELEM_FIELD_RSSI_OFS  18


//----------------------------------------
//-- Message HERELINK_TELEM pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp,
    fmav_status_t* _status)
{
    fmav_herelink_telem_t* _payload = (fmav_herelink_telem_t*)_msg->payload;

    _payload->rf_freq = rf_freq;
    _payload->link_bw = link_bw;
    _payload->link_rate = link_rate;
    _payload->snr = snr;
    _payload->cpu_temp = cpu_temp;
    _payload->board_temp = board_temp;
    _payload->rssi = rssi;


    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_HERELINK_TELEM;
    _msg->target_sysid = 0;
    _msg->target_compid = 0;
    _msg->crc_extra = FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_herelink_telem_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_herelink_telem_pack(
        _msg, sysid, compid,
        _payload->rssi, _payload->snr, _payload->rf_freq, _payload->link_bw, _payload->link_rate, _payload->cpu_temp, _payload->board_temp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp,
    fmav_status_t* _status)
{
    fmav_herelink_telem_t* _payload = (fmav_herelink_telem_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->rf_freq = rf_freq;
    _payload->link_bw = link_bw;
    _payload->link_rate = link_rate;
    _payload->snr = snr;
    _payload->cpu_temp = cpu_temp;
    _payload->board_temp = board_temp;
    _payload->rssi = rssi;


    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_HERELINK_TELEM;
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_HERELINK_TELEM >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_HERELINK_TELEM >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_herelink_telem_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_herelink_telem_pack_to_frame_buf(
        _buf, sysid, compid,
        _payload->rssi, _payload->snr, _payload->rf_freq, _payload->link_bw, _payload->link_rate, _payload->cpu_temp, _payload->board_temp,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp,
    fmav_status_t* _status)
{
    fmav_herelink_telem_t _payload;

    _payload.rf_freq = rf_freq;
    _payload.link_bw = link_bw;
    _payload.link_rate = link_rate;
    _payload.snr = snr;
    _payload.cpu_temp = cpu_temp;
    _payload.board_temp = board_temp;
    _payload.rssi = rssi;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_HERELINK_TELEM,
        FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_herelink_telem_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_herelink_telem_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_HERELINK_TELEM,
        FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_HERELINK_TELEM_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message HERELINK_TELEM decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_herelink_telem_decode(fmav_herelink_telem_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_HERELINK_TELEM_PAYLOAD_LEN_MAX);
#endif
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_herelink_telem_get_field_rf_freq(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_herelink_telem_get_field_link_bw(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[4]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_herelink_telem_get_field_link_rate(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_herelink_telem_get_field_snr(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_herelink_telem_get_field_cpu_temp(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[14]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_msg_herelink_telem_get_field_board_temp(const fmav_message_t* msg)
{
    int16_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_herelink_telem_get_field_rssi(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[18]), sizeof(uint8_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_HERELINK_TELEM  50003

#define mavlink_herelink_telem_t  fmav_herelink_telem_t

#define MAVLINK_MSG_ID_HERELINK_TELEM_LEN  19
#define MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN  19
#define MAVLINK_MSG_ID_50003_LEN  19
#define MAVLINK_MSG_ID_50003_MIN_LEN  19

#define MAVLINK_MSG_ID_HERELINK_TELEM_CRC  62
#define MAVLINK_MSG_ID_50003_CRC  62




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_herelink_telem_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_herelink_telem_pack(
        _msg, sysid, compid,
        rssi, snr, rf_freq, link_bw, link_rate, cpu_temp, board_temp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_herelink_telem_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_herelink_telem_t* _payload)
{
    return mavlink_msg_herelink_telem_pack(
        sysid,
        compid,
        _msg,
        _payload->rssi, _payload->snr, _payload->rf_freq, _payload->link_bw, _payload->link_rate, _payload->cpu_temp, _payload->board_temp);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_herelink_telem_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
    return fmav_msg_herelink_telem_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        rssi, snr, rf_freq, link_bw, link_rate, cpu_temp, board_temp,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_herelink_telem_decode(const mavlink_message_t* msg, mavlink_herelink_telem_t* payload)
{
    fmav_msg_herelink_telem_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_HERELINK_TELEM_H

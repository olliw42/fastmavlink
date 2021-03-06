//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_GPS_INPUT_H
#define FASTMAVLINK_MSG_GPS_INPUT_H


//----------------------------------------
//-- Message GPS_INPUT
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_gps_input_t {
    uint64_t time_usec;
    uint32_t time_week_ms;
    int32_t lat;
    int32_t lon;
    float alt;
    float hdop;
    float vdop;
    float vn;
    float ve;
    float vd;
    float speed_accuracy;
    float horiz_accuracy;
    float vert_accuracy;
    uint16_t ignore_flags;
    uint16_t time_week;
    uint8_t gps_id;
    uint8_t fix_type;
    uint8_t satellites_visible;
    uint16_t yaw;
}) fmav_gps_input_t;


#define FASTMAVLINK_MSG_ID_GPS_INPUT  232

#define FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX  65
#define FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA  151

#define FASTMAVLINK_MSG_GPS_INPUT_FLAGS  0
#define FASTMAVLINK_MSG_GPS_INPUT_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_GPS_INPUT_TARGET_COMPONENT_OFS  0

#define FASTMAVLINK_MSG_GPS_INPUT_FRAME_LEN_MAX  90



#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_TIME_USEC_OFS  0
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_TIME_WEEK_MS_OFS  8
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_LAT_OFS  12
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_LON_OFS  16
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_ALT_OFS  20
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_HDOP_OFS  24
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_VDOP_OFS  28
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_VN_OFS  32
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_VE_OFS  36
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_VD_OFS  40
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_SPEED_ACCURACY_OFS  44
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_HORIZ_ACCURACY_OFS  48
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_VERT_ACCURACY_OFS  52
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_IGNORE_FLAGS_OFS  56
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_TIME_WEEK_OFS  58
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_GPS_ID_OFS  60
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_FIX_TYPE_OFS  61
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_SATELLITES_VISIBLE_OFS  62
#define FASTMAVLINK_MSG_GPS_INPUT_FIELD_YAW_OFS  63


//----------------------------------------
//-- Message GPS_INPUT packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_input_t* _payload = (fmav_gps_input_t*)msg->payload;

    _payload->time_usec = time_usec;
    _payload->time_week_ms = time_week_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->hdop = hdop;
    _payload->vdop = vdop;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->speed_accuracy = speed_accuracy;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;
    _payload->ignore_flags = ignore_flags;
    _payload->time_week = time_week;
    _payload->gps_id = gps_id;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->yaw = yaw;


    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_GPS_INPUT;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_input_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_input_pack(
        msg, sysid, compid,
        _payload->time_usec, _payload->gps_id, _payload->ignore_flags, _payload->time_week_ms, _payload->time_week, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->hdop, _payload->vdop, _payload->vn, _payload->ve, _payload->vd, _payload->speed_accuracy, _payload->horiz_accuracy, _payload->vert_accuracy, _payload->satellites_visible, _payload->yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_input_t* _payload = (fmav_gps_input_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time_usec = time_usec;
    _payload->time_week_ms = time_week_ms;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->hdop = hdop;
    _payload->vdop = vdop;
    _payload->vn = vn;
    _payload->ve = ve;
    _payload->vd = vd;
    _payload->speed_accuracy = speed_accuracy;
    _payload->horiz_accuracy = horiz_accuracy;
    _payload->vert_accuracy = vert_accuracy;
    _payload->ignore_flags = ignore_flags;
    _payload->time_week = time_week;
    _payload->gps_id = gps_id;
    _payload->fix_type = fix_type;
    _payload->satellites_visible = satellites_visible;
    _payload->yaw = yaw;


    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_GPS_INPUT;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INPUT >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_GPS_INPUT >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_input_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_gps_input_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time_usec, _payload->gps_id, _payload->ignore_flags, _payload->time_week_ms, _payload->time_week, _payload->fix_type, _payload->lat, _payload->lon, _payload->alt, _payload->hdop, _payload->vdop, _payload->vn, _payload->ve, _payload->vd, _payload->speed_accuracy, _payload->horiz_accuracy, _payload->vert_accuracy, _payload->satellites_visible, _payload->yaw,
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible, uint16_t yaw,
    fmav_status_t* _status)
{
    fmav_gps_input_t _payload;

    _payload.time_usec = time_usec;
    _payload.time_week_ms = time_week_ms;
    _payload.lat = lat;
    _payload.lon = lon;
    _payload.alt = alt;
    _payload.hdop = hdop;
    _payload.vdop = vdop;
    _payload.vn = vn;
    _payload.ve = ve;
    _payload.vd = vd;
    _payload.speed_accuracy = speed_accuracy;
    _payload.horiz_accuracy = horiz_accuracy;
    _payload.vert_accuracy = vert_accuracy;
    _payload.ignore_flags = ignore_flags;
    _payload.time_week = time_week;
    _payload.gps_id = gps_id;
    _payload.fix_type = fix_type;
    _payload.satellites_visible = satellites_visible;
    _payload.yaw = yaw;


    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_GPS_INPUT,
        FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_gps_input_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_GPS_INPUT,
        FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_GPS_INPUT_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message GPS_INPUT unpacking routines, for receiving
//----------------------------------------
// for these functions to work correctly, msg payload must have been zero filled before
// while for the fmav_msg_gps_input_decode() function, this could be accounted for,
// there is no easy&reasonable way to do it for the fmav_msg_gps_input_get_field_yyy() functions.
// So, we generally require it.

// this should not be needed, but we provide it just in case
FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_input_zero_fill(fmav_message_t* msg)
{
    if (msg->len < FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX) {
        memset(&(((uint8_t*)msg->payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_gps_input_decode(fmav_gps_input_t* payload, const fmav_message_t* msg)
{
    // this assumes msg payload has been zero filled
    //memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX);

    // let's assume it is not zero filled, this should not be needed, but let's just play it safe
    if (msg->len < FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX - msg->len); // zero-fill
    } else {
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_GPS_INPUT_PAYLOAD_LEN_MAX);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint64_t fmav_msg_gps_input_get_field_time_usec(const fmav_message_t* msg)
{
    uint64_t r;
    memcpy(&r, &(msg->payload[0]), sizeof(uint64_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint32_t fmav_msg_gps_input_get_field_time_week_ms(const fmav_message_t* msg)
{
    uint32_t r;
    memcpy(&r, &(msg->payload[8]), sizeof(uint32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_input_get_field_lat(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[12]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR int32_t fmav_msg_gps_input_get_field_lon(const fmav_message_t* msg)
{
    int32_t r;
    memcpy(&r, &(msg->payload[16]), sizeof(int32_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_alt(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[20]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_hdop(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[24]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_vdop(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[28]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_vn(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[32]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_ve(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[36]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_vd(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[40]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_speed_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[44]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_horiz_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[48]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR float fmav_msg_gps_input_get_field_vert_accuracy(const fmav_message_t* msg)
{
    float r;
    memcpy(&r, &(msg->payload[52]), sizeof(float));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_get_field_ignore_flags(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[56]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_get_field_time_week(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[58]), sizeof(uint16_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_input_get_field_gps_id(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[60]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_input_get_field_fix_type(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[61]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_msg_gps_input_get_field_satellites_visible(const fmav_message_t* msg)
{
    uint8_t r;
    memcpy(&r, &(msg->payload[62]), sizeof(uint8_t));
    return r;
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_gps_input_get_field_yaw(const fmav_message_t* msg)
{
    uint16_t r;
    memcpy(&r, &(msg->payload[63]), sizeof(uint16_t));
    return r;
}





//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_GPS_INPUT  232

#define mavlink_gps_input_t  fmav_gps_input_t

#define MAVLINK_MSG_ID_GPS_INPUT_LEN  65
#define MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN  63
#define MAVLINK_MSG_ID_232_LEN  65
#define MAVLINK_MSG_ID_232_MIN_LEN  63

#define MAVLINK_MSG_ID_GPS_INPUT_CRC  151
#define MAVLINK_MSG_ID_232_CRC  151




#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_input_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible, uint16_t yaw)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_gps_input_pack(
        msg, sysid, compid,
        time_usec, gps_id, ignore_flags, time_week_ms, time_week, fix_type, lat, lon, alt, hdop, vdop, vn, ve, vd, speed_accuracy, horiz_accuracy, vert_accuracy, satellites_visible, yaw,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_gps_input_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible, uint16_t yaw)
{
    return fmav_msg_gps_input_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time_usec, gps_id, ignore_flags, time_week_ms, time_week, fix_type, lat, lon, alt, hdop, vdop, vn, ve, vd, speed_accuracy, horiz_accuracy, vert_accuracy, satellites_visible, yaw,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_gps_input_decode(const mavlink_message_t* msg, mavlink_gps_input_t* payload)
{
    fmav_msg_gps_input_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_GPS_INPUT_H

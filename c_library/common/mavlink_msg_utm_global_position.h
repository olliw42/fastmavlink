//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION
//----------------------------------------

// fields are ordered, as they are on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_utm_global_position_t {
    uint64_t time;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    int32_t next_lat;
    int32_t next_lon;
    int32_t next_alt;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    uint16_t h_acc;
    uint16_t v_acc;
    uint16_t vel_acc;
    uint16_t update_rate;
    uint8_t uas_id[18];
    uint8_t flight_state;
    uint8_t flags;
}) fmav_utm_global_position_t;


#define FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340


#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MIN  70
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX  70
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN  70
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA  99

#define FASTMAVLINK_MSG_ID_340_LEN_MIN  70
#define FASTMAVLINK_MSG_ID_340_LEN_MAX  70
#define FASTMAVLINK_MSG_ID_340_LEN  70
#define FASTMAVLINK_MSG_ID_340_CRCEXTRA  99

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN  18

#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_FLAGS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_SYSTEM_OFS  0
#define FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_TARGET_COMPONENT_OFS  0


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION packing routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)msg->payload;

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;

    msg->target_sysid = 0;
    msg->target_compid = 0;
    msg->crc_extra = FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA;

    return fmav_finalize_msg(
        msg,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode(
    fmav_message_t* msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack(
        msg, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_pack_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags,
    fmav_status_t* _status)
{
    fmav_utm_global_position_t* _payload = (fmav_utm_global_position_t*)(&buf[FASTMAVLINK_HEADER_V2_LEN]);

    _payload->time = time;
    _payload->lat = lat;
    _payload->lon = lon;
    _payload->alt = alt;
    _payload->relative_alt = relative_alt;
    _payload->next_lat = next_lat;
    _payload->next_lon = next_lon;
    _payload->next_alt = next_alt;
    _payload->vx = vx;
    _payload->vy = vy;
    _payload->vz = vz;
    _payload->h_acc = h_acc;
    _payload->v_acc = v_acc;
    _payload->vel_acc = vel_acc;
    _payload->update_rate = update_rate;
    _payload->flight_state = flight_state;
    _payload->flags = flags;
    memcpy(&(_payload->uas_id), uas_id, sizeof(uint8_t)*18);

    buf[5] = sysid;
    buf[6] = compid;
    buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
    buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 8);
    buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_UTM_GLOBAL_POSITION >> 16);

    return fmav_finalize_frame_buf(
        buf,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MIN,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_CRCEXTRA,
        _status);
}

    
FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_utm_global_position_encode_to_frame_buf(
    uint8_t* buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_utm_global_position_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        buf, sysid, compid,
        _payload->time, _payload->uas_id, _payload->lat, _payload->lon, _payload->alt, _payload->relative_alt, _payload->vx, _payload->vy, _payload->vz, _payload->h_acc, _payload->v_acc, _payload->vel_acc, _payload->next_lat, _payload->next_lon, _payload->next_alt, _payload->update_rate, _payload->flight_state, _payload->flags,
        _status);
}


//----------------------------------------
//-- Message UTM_GLOBAL_POSITION unpacking routines, for receiving
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_utm_global_position_decode(fmav_utm_global_position_t* payload, const fmav_message_t* msg)
{
    uint8_t len = (msg->len < FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX) ? msg->len : FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX;

    memset(payload, 0, FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_PAYLOAD_LEN_MAX);
    memcpy(payload, msg->payload, len);
}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION  340

#define mavlink_utm_global_position_t  fmav_utm_global_position_t

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN  70
#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_MIN_LEN  70
#define MAVLINK_MSG_ID_340_LEN  70
#define MAVLINK_MSG_ID_340_MIN_LEN  70

#define MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_CRC  99
#define MAVLINK_MSG_ID_340_CRC  99

#define MAVLINK_MSG_UTM_GLOBAL_POSITION_FIELD_UAS_ID_LEN 18


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* msg,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_utm_global_position_pack(
        msg, sysid, compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_utm_global_position_pack_txbuf(
    char* buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    uint64_t time, const uint8_t* uas_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t h_acc, uint16_t v_acc, uint16_t vel_acc, int32_t next_lat, int32_t next_lon, int32_t next_alt, uint16_t update_rate, uint8_t flight_state, uint8_t flags)
{
    return fmav_msg_utm_global_position_pack_to_frame_buf(
        (uint8_t*)buf,
        sysid,
        compid,
        time, uas_id, lat, lon, alt, relative_alt, vx, vy, vz, h_acc, v_acc, vel_acc, next_lat, next_lon, next_alt, update_rate, flight_state, flags,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_utm_global_position_decode(const mavlink_message_t* msg, mavlink_utm_global_position_t* payload)
{
    fmav_msg_utm_global_position_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_UTM_GLOBAL_POSITION_H

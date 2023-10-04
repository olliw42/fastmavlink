//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_${name}_H
#define FASTMAVLINK_MSG_${name}_H


//----------------------------------------
//-- Message ${name}
//----------------------------------------

// fields are ordered, as they appear on the wire
FASTMAVLINK_PACK(
typedef struct _fmav_${name_lower}_t {
${{ordered_fields:    ${type} ${name}${array_suffix};
}}
}) fmav_${name_lower}_t;


#define FASTMAVLINK_MSG_ID_${name}  ${id}

#define FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX  ${payload_max_length}
#define FASTMAVLINK_MSG_${name}_CRCEXTRA  ${crc_extra}

#define FASTMAVLINK_MSG_${name}_FLAGS  ${message_flags}
#define FASTMAVLINK_MSG_${name}_TARGET_SYSTEM_OFS  ${target_system_ofs}
#define FASTMAVLINK_MSG_${name}_TARGET_COMPONENT_OFS  ${target_component_ofs}

#define FASTMAVLINK_MSG_${name}_FRAME_LEN_MAX  ${frame_max_length}

${{array_fields:#define FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM  ${array_length} // number of elements in array
#define FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN  ${field_length} // length of array = number of bytes
}}

${{ordered_fields:#define FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_OFS  ${payload_offset}
}}


//----------------------------------------
//-- Message ${name} pack,encode routines, for sending
//----------------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_pack(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    ${{arg_fields:${array_const}${type}${array_prefix} ${name}, }},
    fmav_status_t* _status)
{
    fmav_${name_lower}_t* _payload = (fmav_${name_lower}_t*)_msg->payload;

${{scalar_fields:    _payload->${name} = ${name_for_setting_payload};
}}
${{array_fields:    memcpy(&(_payload->${name}), ${name}, sizeof(${type})*${array_length});
}}

    _msg->sysid = sysid;
    _msg->compid = compid;
    _msg->msgid = FASTMAVLINK_MSG_ID_${name};
    _msg->target_sysid = ${target_system_field_name};
    _msg->target_compid = ${target_component_field_name};
    _msg->crc_extra = FASTMAVLINK_MSG_${name}_CRCEXTRA;
    _msg->payload_max_len = FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX;

    return fmav_finalize_msg(_msg, _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_encode(
    fmav_message_t* _msg,
    uint8_t sysid,
    uint8_t compid,
    const fmav_${name_lower}_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_${name_lower}_pack(
        _msg, sysid, compid,
        ${{arg_fields:_payload->${name}, }},
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_pack_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    ${{arg_fields:${array_const}${type}${array_prefix} ${name}, }},
    fmav_status_t* _status)
{
    fmav_${name_lower}_t* _payload = (fmav_${name_lower}_t*)(&_buf[FASTMAVLINK_HEADER_V2_LEN]);

${{scalar_fields:    _payload->${name} = ${name_for_setting_payload};
}}
${{array_fields:    memcpy(&(_payload->${name}), ${name}, sizeof(${type})*${array_length});
}}

    _buf[5] = sysid;
    _buf[6] = compid;
    _buf[7] = (uint8_t)FASTMAVLINK_MSG_ID_${name};
    _buf[8] = ((uint32_t)FASTMAVLINK_MSG_ID_${name} >> 8);
    _buf[9] = ((uint32_t)FASTMAVLINK_MSG_ID_${name} >> 16);

    return fmav_finalize_frame_buf(
        _buf,
        FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_${name}_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_encode_to_frame_buf(
    uint8_t* _buf,
    uint8_t sysid,
    uint8_t compid,
    const fmav_${name_lower}_t* _payload,
    fmav_status_t* _status)
{
    return fmav_msg_${name_lower}_pack_to_frame_buf(
        _buf, sysid, compid,
        ${{arg_fields:_payload->${name}, }},
        _status);
}


#ifdef FASTMAVLINK_SERIAL_WRITE_CHAR

FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_pack_to_serial(
    uint8_t sysid,
    uint8_t compid,
    ${{arg_fields:${array_const}${type}${array_prefix} ${name}, }},
    fmav_status_t* _status)
{
    fmav_${name_lower}_t _payload;

${{scalar_fields:    _payload.${name} = ${name_for_setting_payload};
}}
${{array_fields:    memcpy(&(_payload.${name}), ${name}, sizeof(${type})*${array_length});
}}

    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)&_payload,
        FASTMAVLINK_MSG_ID_${name},
        FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_${name}_CRCEXTRA,
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_msg_${name_lower}_encode_to_serial(
    uint8_t sysid,
    uint8_t compid,
    const fmav_${name_lower}_t* _payload,
    fmav_status_t* _status)
{
    return fmav_finalize_serial(
        sysid,
        compid,
        (uint8_t*)_payload,
        FASTMAVLINK_MSG_ID_${name},
        FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX,
        FASTMAVLINK_MSG_${name}_CRCEXTRA,
        _status);
}
#endif


//----------------------------------------
//-- Message ${name} decode routines, for receiving
//----------------------------------------
// For these functions to work correctly, the msg payload must be zero-filled.
// Call the helper fmav_msg_zerofill() if needed, or set FASTMAVLINK_ALWAYS_ZEROFILL to 1
// Note that the parse functions do zero-fill the msg payload, but that message generator functions
// do not. This means that for the msg obtained from parsing the below functions can safely be used,
// but that this is not so for the msg obtained from pack/encode functions.

FASTMAVLINK_FUNCTION_DECORATOR void fmav_msg_${name_lower}_decode(fmav_${name_lower}_t* payload, const fmav_message_t* msg)
{
#if FASTMAVLINK_ALWAYS_ZEROFILL
    if (msg->len < FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX) {
        memcpy(payload, msg->payload, msg->len);
        // ensure that returned payload is zero-filled
        memset(&(((uint8_t*)payload)[msg->len]), 0, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX - msg->len);
    } else {
        // note: msg->len can be larger than PAYLOAD_LEN_MAX if the message has unknown extensions
        memcpy(payload, msg->payload, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
    }
#else
    // this requires that msg payload had been zero-filled before
    memcpy(payload, msg->payload, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
#endif
}


${{scalar_fields:FASTMAVLINK_FUNCTION_DECORATOR ${type} fmav_msg_${name_lower}_get_field_${name}(const fmav_message_t* msg)
{
    ${type} r;
    memcpy(&r, &(msg->payload[${payload_offset}]), sizeof(${type}));
    return r;
}


}}


${{array_fields:FASTMAVLINK_FUNCTION_DECORATOR ${type}* fmav_msg_${name_lower}_get_field_${name}_ptr(const fmav_message_t* msg)
{
    return (${type}*)&(msg->payload[${payload_offset}]);
}


FASTMAVLINK_FUNCTION_DECORATOR ${type} fmav_msg_${name_lower}_get_field_${name}(uint16_t index, const fmav_message_t* msg)
{
    if (index >= FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM) return 0;
    return ((${type}*)&(msg->payload[${payload_offset}]))[index];
}


}}


//----------------------------------------
//-- Pymavlink wrappers
//----------------------------------------
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED

#define MAVLINK_MSG_ID_${name}  ${id}

#define mavlink_${name_lower}_t  fmav_${name_lower}_t

#define MAVLINK_MSG_ID_${name}_LEN  ${payload_max_length}
#define MAVLINK_MSG_ID_${name}_MIN_LEN  ${payload_min_length}
#define MAVLINK_MSG_ID_${id}_LEN  ${payload_max_length}
#define MAVLINK_MSG_ID_${id}_MIN_LEN  ${payload_min_length}

#define MAVLINK_MSG_ID_${name}_CRC  ${crc_extra}
#define MAVLINK_MSG_ID_${id}_CRC  ${crc_extra}

${{array_fields:#define MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN ${array_length}
}}


#if MAVLINK_COMM_NUM_BUFFERS > 0

FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_${name_lower}_pack(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    ${{arg_fields:${array_const}${type}${array_prefix} ${name}, }})
{
    fmav_status_t* _status = mavlink_get_channel_status(MAVLINK_COMM_0);
    return fmav_msg_${name_lower}_pack(
        _msg, sysid, compid,
        ${{arg_fields:${name}, }},
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_${name_lower}_encode(
    uint8_t sysid,
    uint8_t compid,
    mavlink_message_t* _msg,
    const mavlink_${name_lower}_t* _payload)
{
    return mavlink_msg_${name_lower}_pack(
        sysid,
        compid,
        _msg,
        ${{arg_fields:_payload->${name}, }});
}

#endif


FASTMAVLINK_FUNCTION_DECORATOR uint16_t mavlink_msg_${name_lower}_pack_txbuf(
    char* _buf,
    fmav_status_t* _status,
    uint8_t sysid,
    uint8_t compid,
    ${{arg_fields:${array_const}${type}${array_prefix} ${name}, }})
{
    return fmav_msg_${name_lower}_pack_to_frame_buf(
        (uint8_t*)_buf,
        sysid,
        compid,
        ${{arg_fields:${name}, }},
        _status);
}


FASTMAVLINK_FUNCTION_DECORATOR void mavlink_msg_${name_lower}_decode(const mavlink_message_t* msg, mavlink_${name_lower}_t* payload)
{
    fmav_msg_${name_lower}_decode(payload, msg);
}

#endif // FASTMAVLINK_PYMAVLINK_ENABLED


#endif // FASTMAVLINK_MSG_${name}_H

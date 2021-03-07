//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_MSG_${name}_H
#define FASTMAVLINK_TEST_MSG_${name}_H


uint8_t run_test_msg_${name_lower}_one(void)
{
    char _s[1024];

    SPRINTF(_s, sizeof(_s), "\n"); PRINT(_s);
    SPRINTF(_s, sizeof(_s), "Test ${name}\n"); PRINT(_s);

    if (MAVLINK_MSG_ID_${name}_LEN != FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX) {
        SPRINTF(_s, sizeof(_s), "payload max len: FAIL (%i %i)\n",
            MAVLINK_MSG_ID_${name}_LEN,
            FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
        PRINT(_s);
        return 0;
    }
${{array_fields:    if (MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN != FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM) {
        SPRINTF(_s, sizeof(_s), "payload field %s num: FAIL (%i %i)\n",
            "${name}",
            MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN,
            FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM);
        PRINT(_s);
        return 0;
    }
    if (MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN*(int)sizeof(${type}) != FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN) {
        SPRINTF(_s, sizeof(_s), "payload field %s len (num*size): FAIL (%i %i)\n",
            "${name}",
            MAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN*(int)sizeof(${type}),
            FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN);
        PRINT(_s);
        return 0;
    }
}}


    //----------------------
    // header values

    // header fields
    uint8_t _seq = rand_uint8_t();
    uint8_t sysid = rand_uint8_t();
    uint8_t compid = rand_uint8_t();


    //----------------------
    // PACKING
    //----------------------

    // payload fields
${{scalar_fields:    ${type} ${name} = randp_${type}();
}}
${{array_fields:    ${type} ${name}[FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM];
    for(int i = 0; i < FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM; i++) ${name}[i] = randp_${type}();
}}

    SPRINTF(_s, sizeof(_s), "  %i %i %i ,", _seq, sysid, compid); PRINT(_s);
${{scalar_fields:    SPRINTF(_s, sizeof(_s), MESSAGE_FIELD_FORMAT_${type}, ${name}); PRINT(_s);
}}
    SPRINTF(_s, sizeof(_s), "\n"); PRINT(_s);
${{array_fields:    SPRINTF(_s, sizeof(_s), "  ("); PRINT(_s);
    for(int i = 0; i < FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_NUM; i++) {
        SPRINTF(_s, sizeof(_s), MESSAGE_FIELD_FORMAT_${type}, ${name}[i]);
        PRINT(_s);
    }
    SPRINTF(_s, sizeof(_s), " )\n"); PRINT(_s);
}}

#if FASTMAVLINK_MSG_ID_${name} == FASTMAVLINK_MSG_ID_HEARTBEAT
    mavlink_version = FASTMAVLINK_MAVLINK_VERSION;
#endif

   //----------------------
   // pymavlink-mavgen: create reference

    mavlink_status_t *pymav_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    pymav_status->current_tx_seq = _seq;
    pymav_status->flags = 0; // MAVLink v2 out
    pymav_status->signing = NULL; // no signing

    int pymav_len;
    mavlink_message_t pymav_msg;
    int pymav_frame_len;
    uint8_t pymav_frame[1024];
    mavlink_${name_lower}_t pymav_payload;
  
    memset(&pymav_msg, 10, sizeof(pymav_msg));
    memset(pymav_frame, 11, sizeof(pymav_frame));
    memset(&pymav_payload, 12, sizeof(pymav_payload));

    pymav_len = mavlink_msg_${name_lower}_pack(
        sysid,
        compid,
        &pymav_msg,
        ${{arg_fields:${name}, }}
        );

    pymav_frame_len = mavlink_msg_to_send_buffer(pymav_frame, &pymav_msg);

    mavlink_msg_${name_lower}_decode(&pymav_msg, &pymav_payload);


    //----------------------
    // prepare fastMavlink

    fmav_status_t _status;

    fmav_${name_lower}_t _payload;
${{scalar_fields:    _payload.${name} = ${name};
}}
${{array_fields:    memcpy(&(_payload.${name}), ${name}, FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN);
}}

    int _len;
    fmav_message_t _msg;
    uint8_t _frame[1024];

    int is_ok;
    int is_ok_res = 1; // the return value


    //----------------------
    // fastMavlink:  test fmav_msg_xxx_pack()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_pack()\n"); PRINT(_s);

    _status.tx_seq = _seq;

    memset(&_msg, 13, sizeof(_msg));

    _len = fmav_msg_${name_lower}_pack(
        &_msg,
        sysid,
        compid,
        ${{arg_fields:${name}, }},
        &_status);

    PRINT_TEST(_len == pymav_len, "len");
    is_ok = compare_message_pymav_fmav(&pymav_msg, &_msg, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
    PRINT_MESSAGES_PYMAV_FMAV(is_ok, "message", &pymav_msg, &_msg, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);

    if (!is_ok) is_ok_res = 0;

    //----------------------
    // fastMavlink:  test fmav_msg_xxx_encode()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_encode()\n"); PRINT(_s);

    _status.tx_seq = _seq;

    memset(&_msg, 14, sizeof(_msg));

    _len = fmav_msg_${name_lower}_encode(&_msg, sysid, compid, &_payload, &_status);

    PRINT_TEST(_len == pymav_len, "len");
    is_ok = compare_message_pymav_fmav(&pymav_msg, &_msg, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
    PRINT_MESSAGES_PYMAV_FMAV(is_ok, "message", &pymav_msg, &_msg, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);

    if (!is_ok) is_ok_res = 0;

    //----------------------
    // fastMavlink:  test fmav_msg_xxx_pack_to_frame_buf()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_pack_to_frame_buf()\n"); PRINT(_s);

    _status.tx_seq = _seq;

    memset(_frame, 15, sizeof(_frame));

    _len = fmav_msg_${name_lower}_pack_to_frame_buf(
        _frame,
        sysid,
        compid,
        ${{arg_fields:${name}, }},
        &_status);

    PRINT_TEST(_len == pymav_len, "len");
    is_ok = compare_frame_buf(pymav_frame, _frame, _len); // only check for actual length
    PRINT_FRAMES(is_ok, "frame", pymav_frame, pymav_frame_len, _frame, _len);

    if (!is_ok) is_ok_res = 0;

    //----------------------
    // fastMavlink:  test fmav_msg_xxx_encode_to_frame_buf()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_encode_to_frame_buf()\n"); PRINT(_s);

    _status.tx_seq = _seq;

    memset(_frame, 16, sizeof(_frame));

    _len = fmav_msg_${name_lower}_encode_to_frame_buf(_frame, sysid, compid, &_payload, &_status);

    PRINT_TEST(_len == pymav_len, "len");
    is_ok = compare_frame_buf(pymav_frame, _frame, _len); // only check for actual length
    PRINT_FRAMES(is_ok, "frame", pymav_frame, pymav_frame_len, _frame, _len);

    if (!is_ok) is_ok_res = 0;


    //----------------------
    // fastMavlink: UNPACKING
    // we could do these type of tests:
    // 1. use pymavlink-mavgen to create a reference, as before
    // 2. use the fastMavlink msg from the above, assuming the above tests pass
    // 3. convert the pymavlink-mavgen msg to the fastMavlink msg
    // let's do 2.
  
    fmav_${name_lower}_t _payload_res;
    memset(&_payload_res, 16, sizeof(_payload_res));
  
    int field_is_ok;
  
    //----------------------
    // fastMavlink:  test fmav_msg_xxx_decode()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_decode()\n"); PRINT(_s);
  
    fmav_msg_${name_lower}_decode(&_payload_res, &_msg);

    is_ok = 1;

    //is_ok = (memcmp(&_payload_res, &_payload, sizeof(_payload)) == 0) ? 1 : 0;
    //PRINT_TEST(is_ok, "payload");
${{scalar_fields:    if (_payload_res.${name} != _payload.${name}) field_is_ok = is_ok = 0; else field_is_ok = 1;
    PRINT_TEST(field_is_ok, "field ${name}");
}}
${{array_fields:    if (memcmp(_payload_res.${name}, _payload.${name}, FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN) != 0) field_is_ok = is_ok = 0; else field_is_ok = 1;
    PRINT_TEST(field_is_ok, "field ${name}");
}}
    PRINT_PAYLOADS(is_ok, "payload", (uint8_t*)&_payload, (uint8_t*)&_payload_res, FASTMAVLINK_MSG_${name}_PAYLOAD_LEN_MAX);
               
    if (!is_ok) is_ok_res = 0;

    //----------------------
    // fastMavlink:  test fmav_msg_xxx_get_field_yyy()
    SPRINTF(_s, sizeof(_s), "  fmav_msg_${name_lower}_get_field_xxx()\n"); PRINT(_s);

    is_ok = 1;

${{scalar_fields:    ${type} ${name}_res = fmav_msg_${name_lower}_get_field_${name}(&_msg);
    if (${name}_res != _payload.${name}) field_is_ok = is_ok = 0; else field_is_ok = 1;
    PRINT_TEST(field_is_ok, "field ${name}");
}}
${{array_fields:    ${type}* ${name}_res = fmav_msg_${name_lower}_get_field_${name}_ptr(&_msg);
    if (memcmp(${name}_res, _payload.${name}, FASTMAVLINK_MSG_${msg_name}_FIELD_${name_upper}_LEN)) field_is_ok = is_ok = 0; else field_is_ok = 1;
    PRINT_TEST(field_is_ok, "field ${name}");
}}

    if (!is_ok) is_ok_res = 0;

    return is_ok_res;
}


#endif // FASTMAVLINK_TEST_MSG_${name}_H

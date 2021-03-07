//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_${basename_upper}_H
#define FASTMAVLINK_TEST_${basename_upper}_H

#ifdef __cplusplus
extern "C" {
#endif


// this is needed since pymavlink-mavgen and fastMavlink generate exactly the same enums
// (which ought to be so since they are defined by the dialect xml)
#define FASTMAVLINK_TEST_EXCLUDE_ENUMS

#include "../../c_library/${basename}/${basename}.h"


#define MESSAGE_FIELD_FORMAT_uint8_t  " %i"
#define MESSAGE_FIELD_FORMAT_uint16_t " %i"
#define MESSAGE_FIELD_FORMAT_uint32_t " %i"
#define MESSAGE_FIELD_FORMAT_uint64_t " %lli"
#define MESSAGE_FIELD_FORMAT_int8_t   " %i"
#define MESSAGE_FIELD_FORMAT_int16_t  " %i"
#define MESSAGE_FIELD_FORMAT_int32_t  " %i"
#define MESSAGE_FIELD_FORMAT_int64_t  " %lli"
#define MESSAGE_FIELD_FORMAT_char     " %i"
#define MESSAGE_FIELD_FORMAT_float    " %f"
#define MESSAGE_FIELD_FORMAT_double   " %g"


// forward declaration of what's in fastmavlink_test_functions.h
// some environments like Arduino need this
int compare_frame_buf(uint8_t* frame1, uint8_t* frame2, uint8_t len);
int compare_message(fmav_message_t* msg1, fmav_message_t* msg2, uint8_t payload_max_len);
int compare_message_pymav_fmav(mavlink_message_t* pymav_msg, fmav_message_t* msg, uint8_t payload_max_len);
void PRINT(char* s);
void PRINT_TEST(int is_ok, char* s);
void PRINT_FRAMES(int is_ok, char* s, uint8_t* frame1, uint16_t frame1_len, uint8_t* frame2, uint16_t frame2_len);
void PRINT_PAYLOADS(int is_ok, char* s, uint8_t* payload1, uint8_t* payload2, uint8_t payload_len);
void PRINT_MESSAGES(int is_ok, char* s, fmav_message_t* msg1, fmav_message_t* msg2, uint8_t payload_max_len);
void PRINT_MESSAGES_PYMAV_FMAV(int is_ok, char* s, mavlink_message_t* pymav_msg, fmav_message_t* msg, uint8_t payload_max_len);

#include "../fastmavlink_test_config.h"
#include "../fastmavlink_test_functions.h"


//------------------------------
//-- Message definitions
//------------------------------

${{messages:#include "./test_msg_${name_lower}.h"
}}


//------------------------------
//-- Test runner messages
//------------------------------

uint8_t run_test_${basename}_msg_one_by_msgid(uint32_t msgid)
{
    switch (msgid) {
${{messages:    case FASTMAVLINK_MSG_ID_${name}:
        return run_test_msg_${name_lower}_one();
}}
    }

    return 1; // unknown message, should never happen
}


uint16_t fmav_${basename}_msg_encode(uint32_t msgid, fmav_message_t* msg, uint8_t sysid, uint8_t compid, const uint8_t* payload, fmav_status_t* status)
{
    switch (msgid) {
${{messages:    case FASTMAVLINK_MSG_ID_${name}:        
        return fmav_msg_${name_lower}_encode(msg, sysid, compid, (const fmav_${name_lower}_t*)payload, status);
}}
    }

    return 0; // unknown message, should never happen
}


uint16_t pymav_${basename}_msg_encode(uint32_t msgid, uint8_t sysid, uint8_t compid, mavlink_message_t* msg, const uint8_t* payload)
{
    switch (msgid) {
${{messages:    case FASTMAVLINK_MSG_ID_${name}:        
        return mavlink_msg_${name_lower}_encode(sysid, compid, msg, (const mavlink_${name_lower}_t*)payload);
}}
    }

    return 0; // unknown message, should never happen
}


//------------------------------
//-- Dialect includes
//------------------------------

#ifndef FASTMAVLINK_TEST_DIALECT_TEST_RUNNER
#define FASTMAVLINK_TEST_DIALECT_TEST_RUNNER
#define FASTMAVLINK_TEST_INCLUDE_${basename_upper}_TEST_RUNNER
#endif


${{include_list:#include "../${name}/test_${name}.h"
}}


//------------------------------
//-- Test runner dialect
//------------------------------

#ifdef FASTMAVLINK_TEST_INCLUDE_${basename_upper}_TEST_RUNNER

uint8_t run_test_msg_one_by_msgid(uint32_t msgid)
{
    // there can't be duplicate msgids, so jumping out after an error is ok
    if (!run_test_${basename}_msg_one_by_msgid(msgid)) return 0;
${{include_list:    if (!run_test_${name}_msg_one_by_msgid(msgid)) return 0;
}}
    return 1;
}


uint16_t fmav_msg_encode(uint32_t msgid, fmav_message_t* msg, uint8_t sysid, uint8_t compid, const uint8_t* payload, fmav_status_t* status)
{
    uint16_t len;
    len = fmav_${basename}_msg_encode(msgid, msg, sysid, compid, payload, status);
    if (len > 0) return len;
${{include_list:    len = fmav_${name}_msg_encode(msgid, msg, sysid, compid, payload, status);
    if (len > 0) return len;
}}
    return 0; // unknown message, should never happen
}


uint16_t pymav_msg_encode(uint32_t msgid, uint8_t sysid, uint8_t compid, mavlink_message_t* msg, const uint8_t* payload)
{
    uint16_t len;
    len = pymav_${basename}_msg_encode(msgid, sysid, compid, msg, payload);
    if (len > 0) return len;
${{include_list:    len = pymav_${name}_msg_encode(msgid, sysid, compid, msg, payload);
    if (len > 0) return len;
}}
    return 0; // unknown message, should never happen
}

#endif


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_TEST_${basename_upper}_H

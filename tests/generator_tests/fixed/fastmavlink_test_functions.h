//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

//------------------------------
// test_messages
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_FUNCTIONS_H
#define FASTMAVLINK_TEST_FUNCTIONS_H

#include <stdint.h>

#include "fastmavlink_test_config.h"


//------------------------------
// compare
//------------------------------

int compare_frame_buf(uint8_t* frame1, uint8_t* frame2, uint8_t len)
{
    int is_ok = 1;

    for (int i = 0; i < len; i++) {
        if (frame1[i] != frame2[i]) is_ok = 0;
    }

    return is_ok;
}


int compare_message(fmav_message_t* msg1, fmav_message_t* msg2, uint8_t payload_max_len)
{
    int is_ok = 1;

    if (msg1->magic != msg2->magic) { is_ok = 0; PRINT("    magic: FAIL\n"); }
    if (msg1->len != msg2->len) { is_ok = 0; PRINT("    len: FAIL\n"); }
    if (msg1->incompat_flags != msg2->incompat_flags) { is_ok = 0; PRINT("    incompat_flags: FAIL\n"); }
    if (msg1->compat_flags != msg2->compat_flags) { is_ok = 0; PRINT("    compat_flags: FAIL\n"); }
    if (msg1->seq != msg2->seq) { is_ok = 0; PRINT("    seq: FAIL\n"); }
    if (msg1->sysid != msg2->sysid) { is_ok = 0; PRINT("    sysid: FAIL\n"); }
    if (msg1->compid != msg2->compid) { is_ok = 0; PRINT("    compid: FAIL\n"); }
    if (msg1->msgid != msg2->msgid) { is_ok = 0; PRINT("    msgid: FAIL\n"); }
    if (msg1->checksum != msg2->checksum) { is_ok = 0; PRINT("    checksum: FAIL\n"); }

    int payload_is_ok = 1;
    for (int i = 0; i < payload_max_len; i++) {
        if (msg1->payload[i] != msg2->payload[i]) payload_is_ok = 0;
    }
    //if (!payload_is_ok) { is_ok = 0; PRINT("    payload: FAIL\n"); }
    if (!payload_is_ok) is_ok = 0;
    PRINT_TEST(payload_is_ok, "payload");

    return is_ok;
}


int compare_message_pymav_fmav(mavlink_message_t* pymav_msg, fmav_message_t* msg, uint8_t payload_max_len)
{
    int is_ok = 1;

    if (pymav_msg->magic != msg->magic) { is_ok = 0; PRINT("    magic: FAIL\n"); }
    if (pymav_msg->len != msg->len) { is_ok = 0; PRINT("    len: FAIL\n"); }
    if (pymav_msg->incompat_flags != msg->incompat_flags) { is_ok = 0; PRINT("    incompat_flags: FAIL\n"); }
    if (pymav_msg->compat_flags != msg->compat_flags) { is_ok = 0; PRINT("    compat_flags: FAIL\n"); }
    if (pymav_msg->seq != msg->seq) { is_ok = 0; PRINT("    seq: FAIL\n"); }
    if (pymav_msg->sysid != msg->sysid) { is_ok = 0; PRINT("    sysid: FAIL\n"); }
    if (pymav_msg->compid != msg->compid) { is_ok = 0; PRINT("    compid: FAIL\n"); }
    if (pymav_msg->msgid != msg->msgid) { is_ok = 0; PRINT("    msgid: FAIL\n"); }
    if (pymav_msg->checksum != msg->checksum) { is_ok = 0; PRINT("    checksum: FAIL\n"); }

    uint8_t* pymav_msg_payload = (uint8_t*)pymav_msg->payload64;

    // ATTENTION:
    // pymavlink-mavgen does NOT fill the payload completely, but cuts trailing zeros!
    // fastMavlink however does
    // hence, if we use payload_max_len to compare payloads, false errors are ferquent
    //int len = payload_max_len;
    int len = msg->len;

    int payload_is_ok = 1;
    for (int i = 0; i < len; i++) {
        if (pymav_msg_payload[i] != msg->payload[i]) payload_is_ok = 0;
    }
    //if (!payload_is_ok) { is_ok = 0; PRINT("    payload: FAIL\n"); }
    if (!payload_is_ok) is_ok = 0;
    PRINT_TEST(payload_is_ok, "payload");

    // fastMavlink does zero-fill the payload
    // so let's check this explicitely
    int payload_zerofill_is_ok = 1;
    for (int i = len; i < payload_max_len; i++) {
        if (msg->payload[i] != 0) payload_zerofill_is_ok = 0;
    }
    //if (!payload_zerofill_is_ok) { is_ok = 0; PRINT("    payload zero-fill: FAIL\n"); }
    if (!payload_zerofill_is_ok) is_ok = 0;
    PRINT_TEST(payload_zerofill_is_ok, "payload zero fill");

    return is_ok;
}


//------------------------------
// printers
//------------------------------

int print_only_errors = 1;


void set_print_only_errors(int flag)
{
    print_only_errors = flag;
}


char global_s[FASTMAVLINK_TEST_GLOBAL_S_LEN];
char helper_s[FASTMAVLINK_TEST_HELPER_S_LEN];


void PRINT_RESET(void)
{
    global_s[0] = '\0';
}


void PRINT(char* s)
{
    STRCAT(global_s, sizeof(global_s), s);
    if (print_only_errors) return;
    PRINTF(s);
}


void PRINT_TEST(int is_ok, char* s)
{
    if (is_ok) {
        SPRINTF(helper_s, sizeof(helper_s), "    %s: OK\n", s);
    }else{
        SPRINTF(helper_s, sizeof(helper_s), "    %s: FAIL\n", s);
    }
    PRINT(helper_s);
}


void PRINT_FRAMES(int is_ok, char* s, uint8_t* frame1, uint16_t frame1_len, uint8_t* frame2, uint16_t frame2_len)
{
    PRINT_TEST(is_ok, s);

    if (is_ok) return;

    SPRINTF(helper_s, sizeof(helper_s), "    1("); PRINT(helper_s);
    for (int i = 0; i < frame1_len; i++) {
        if (i == FASTMAVLINK_HEADER_V2_LEN) { SPRINTF(helper_s, sizeof(helper_s), " |"); PRINT(helper_s); }
        if (i == FASTMAVLINK_HEADER_V2_LEN+frame1[1]) { SPRINTF(helper_s, sizeof(helper_s), " |"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", frame1[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "    2("); PRINT(helper_s);
    for (int i = 0; i < frame2_len; i++) {
        if (i == FASTMAVLINK_HEADER_V2_LEN) { SPRINTF(helper_s, sizeof(helper_s), " |"); PRINT(helper_s); }
        if (i == FASTMAVLINK_HEADER_V2_LEN+frame2[1]) { SPRINTF(helper_s, sizeof(helper_s), " |"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", frame2[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);
}


void PRINT_PAYLOADS(int is_ok, char* s, uint8_t* payload1, uint8_t* payload2, uint8_t payload_len)
{
    PRINT_TEST(is_ok, s);

    if (is_ok) return;

    SPRINTF(helper_s, sizeof(helper_s), "      1("); PRINT(helper_s);
    for (int i = 0; i < payload_len; i++) {
        SPRINTF(helper_s, sizeof(helper_s), " %3i", payload1[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "      2("); PRINT(helper_s);
    for (int i = 0; i < payload_len; i++) {
        SPRINTF(helper_s, sizeof(helper_s), " %3i", payload2[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);
}


void PRINT_MESSAGES(int is_ok, char* s, fmav_message_t* msg1, fmav_message_t* msg2, uint8_t payload_max_len)
{
    PRINT_TEST(is_ok, s);

    if (is_ok) return;

    SPRINTF(helper_s, sizeof(helper_s), "      magic: %i %i\n", msg1->magic, msg2->magic); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      len: %i %i (max: %i)\n", msg1->len, msg2->len, payload_max_len); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      incompat_flags: %i %i\n", msg1->incompat_flags, msg2->incompat_flags); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      compat_flags: %i %i\n", msg1->compat_flags, msg2->compat_flags); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      seq: %i %i\n", msg1->seq, msg2->seq); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      sysid: %i %i\n", msg1->sysid, msg2->sysid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      compid: %i %i\n", msg1->compid, msg2->compid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      msgid: %i %i\n", msg1->msgid, msg2->msgid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      checksum: %i %i\n", msg1->checksum, msg2->checksum); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "      1("); PRINT(helper_s);
    for (int i = 0; i < payload_max_len; i++) {
        if (i == msg1->len) { SPRINTF(helper_s, sizeof(helper_s), " ,"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", msg1->payload[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "      2("); PRINT(helper_s);
    for (int i = 0; i < payload_max_len; i++) {
        if (i == msg2->len) { SPRINTF(helper_s, sizeof(helper_s), " ,"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", msg2->payload[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);
}


void PRINT_MESSAGES_PYMAV_FMAV(int is_ok, char* s, mavlink_message_t* pymav_msg, fmav_message_t* msg, uint8_t payload_max_len)
{
    PRINT_TEST(is_ok, s);

    if (is_ok) return;

    SPRINTF(helper_s, sizeof(helper_s), "      magic: %i %i\n", pymav_msg->magic, msg->magic); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      len: %i %i (max: %i)\n", pymav_msg->len, msg->len, payload_max_len); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      incompat_flags: %i %i\n", pymav_msg->incompat_flags, msg->incompat_flags); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      compat_flags: %i %i\n", pymav_msg->compat_flags, msg->compat_flags); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      seq: %i %i\n", pymav_msg->seq, msg->seq); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      sysid: %i %i\n", pymav_msg->sysid, msg->sysid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      compid: %i %i\n", pymav_msg->compid, msg->compid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      msgid: %i %i\n", pymav_msg->msgid, msg->msgid); PRINT(helper_s);
    SPRINTF(helper_s, sizeof(helper_s), "      checksum: %i %i\n", pymav_msg->checksum, msg->checksum); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "      p("); PRINT(helper_s);
    for (int i = 0; i < payload_max_len; i++) {
        if (i == pymav_msg->len) { SPRINTF(helper_s, sizeof(helper_s), " ,"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", ((uint8_t*)pymav_msg->payload64)[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);

    SPRINTF(helper_s, sizeof(helper_s), "      f("); PRINT(helper_s);
    for (int i = 0; i < payload_max_len; i++) {
        if (i == msg->len) { SPRINTF(helper_s, sizeof(helper_s), " ,"); PRINT(helper_s); }
        SPRINTF(helper_s, sizeof(helper_s), " %3i", msg->payload[i]);
        PRINT(helper_s);
    }
    SPRINTF(helper_s, sizeof(helper_s), " )\n"); PRINT(helper_s);
}


#endif // FASTMAVLINK_TEST_FUNCTIONS_H
//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// fastMavlink Test Suite
// for Microsoft Visual Studio C
// parser test
//------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <stdint.h>


//------------------------------
// test
//------------------------------

#define RUNS  10 // 0 means indefinitely

#define MESSAGES_STREAM_NUM  32


//------------------------------
// random value generators
// needs to be included before dialects are included

#include "test_c_library/fastmavlink_test_random_generators.h"


//------------------------------
// includes

// to silence some double to float conversion warnings from thw pymavlink-mavgen library
#pragma warning(disable : 4244) 

//#include "pymavlink_c_library_v2/minimal/mavlink.h"
//#include "test_c_library/minimal/test_minimal.h"

//#include "pymavlink_c_library_v2/common/mavlink.h"
//#include "test_c_library/common/test_common.h"

//#include "pymavlink_c_library_v2/ardupilotmega/mavlink.h"
//#include "test_c_library/ardupilotmega/test_ardupilotmega.h"

#include "pymavlink_c_library_v2/all/mavlink.h"
#include "test_c_library/all/test_all.h"


//------------------------------
// here we go
//------------------------------

// test parsers, method 1
// generates byte stream of N messages randomly selected from dialect
// validates byte stream agains pymavlink-mavgen
// plays it to the parser
// parses and checks against the known sequence of messages


#define BYTE_STREAM_LEN  16000 // (MESSAGES_STREAM_NUM * FASTMAVLINK_FRAME_LEN_MAX)


uint32_t msgidstream[MESSAGES_STREAM_NUM];
fmav_message_t msgstream[MESSAGES_STREAM_NUM];

int bytestream_len;
uint8_t bytestream[BYTE_STREAM_LEN];


int generate_byte_stream_one_message(int* msgindex, int* streamindex)
{
    int msg_num = fmav_get_message_entry_num();

    // pick msgid randomly from dialect

    int msg_i = rand_int(0, msg_num-1);
    const fmav_message_entry_t* msg_entry = fmav_get_message_entry_by_index(msg_i);
    if (!msg_entry) { printf("SHIT1\n"); return 0; } // uups, msgid is not know, should not happen
    uint32_t msgid = msg_entry->msgid;
    printf("msgid: %i\n", msgid);

    // random header & payload

    uint8_t seq = rand_uint8_t();
    uint8_t sysid = rand_uint8_t();
    uint8_t compid = rand_uint8_t();

    uint8_t payload[FASTMAVLINK_PAYLOAD_LEN_MAX];
    for(uint8_t i = 0; i < FASTMAVLINK_PAYLOAD_LEN_MAX; i++) payload[i] = rand_uint8_t();

    // fastmavlink: create msg and frame

    fmav_status_t status;
    status.tx_seq = seq;

    fmav_message_t msg;
    int frame_len;
    uint8_t frame[FASTMAVLINK_FRAME_LEN_MAX];

    if (!fmav_msg_encode(msgid, &msg, sysid, compid, payload, &status)) { printf("SHIT2\n"); return 0; };
    frame_len = fmav_msg_to_frame_buf(frame, &msg);
    
    // pymavlink-mavgen: create reference frame 

    mavlink_status_t* pymav_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    pymav_status->current_tx_seq = seq;
    pymav_status->flags = 0; // MAVLink v2 out
    pymav_status->signing = NULL; // no signing

    mavlink_message_t pymav_msg;
    int pymav_frame_len;
    uint8_t pymav_frame[296];

    if (!pymav_msg_encode(msgid, sysid, compid, &pymav_msg, payload)) { printf("SHIT3\n"); return 0; }
    pymav_frame_len = mavlink_msg_to_send_buffer(pymav_frame, &pymav_msg);

    // validate fastmavlink against pymavlink-mavgen frame 

    PRINT_TEST(frame_len == pymav_frame_len, "frame len");
    int is_ok = compare_frame_buf(pymav_frame, frame, frame_len);
    PRINT_FRAMES(is_ok, "frame", pymav_frame, pymav_frame_len, frame, frame_len);
    if (!is_ok) { printf("SHIT4\n"); return 0; }

    // all ok, check available memory

    if (*msgindex >= MESSAGES_STREAM_NUM) { printf("SHIT5\n"); return 0; }
    if (*streamindex + frame_len >= BYTE_STREAM_LEN) { printf("SHIT6\n"); return 0; }

    // all ok, store

    msgstream[*msgindex] = msg;
    (*msgindex)++;

    for (uint16_t i = 0; i < frame_len; i++) {
        bytestream[*streamindex] = frame[i];
        (*streamindex)++;
    }

    return 1;
}


int generate_byte_stream(void)
{
    int msgindex = 0;
    int streamindex = 0;

    for (int n = 0; n < MESSAGES_STREAM_NUM; n++) {
        if (!generate_byte_stream_one_message(&msgindex, &streamindex)) {
             printf("SHIT7\n"); return 0;
        };
    }

    bytestream_len = streamindex;
    return 1;
}


fmav_status_t status;
fmav_message_t rx_msg;
uint8_t rx_buf[296];


int parse_byte_stream_one_message_test_msg(int* msgindex, fmav_message_t* rx_msg)
{
    printf("tx msgid: %i\n", rx_msg->msgid);

    fmav_message_t* msg = &(msgstream[*msgindex]);
    (*msgindex)++;

    const fmav_message_entry_t* msg_entry = fmav_get_message_entry(msg->msgid);
    if (!msg_entry) { printf("SHIT10\n"); return 0; } // uups, msgid is not know, should not happen

    int is_ok = compare_message(rx_msg, msg, msg_entry->payload_max_len);
    PRINT_MESSAGES(is_ok, "message", rx_msg, msg, msg_entry->payload_max_len);
    if (!is_ok) { printf("SHIT11\n"); return 0; }

    return 1;
}


int parse_byte_stream_one_message_w_parser1(int* msgindex, int* streamindex)
{
    while (*streamindex < bytestream_len) {
        uint8_t c = bytestream[*streamindex];
        (*streamindex)++;

        fmav_result_t result;
        uint8_t res = fmav_parse_to_frame_buf(&result, rx_buf, &status, c);
        if (res == FASTMAVLINK_PARSE_RESULT_OK) {
            res = fmav_check_frame_buf(&result, rx_buf);
            if (res == FASTMAVLINK_PARSE_RESULT_OK) {
                fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf);
                return parse_byte_stream_one_message_test_msg(msgindex, &rx_msg);
            }
        }
    }

    printf("SHIT12\n");
    return 0;
}


int parse_byte_stream_one_message_w_parser2(int* msgindex, int* streamindex)
{
    while (*streamindex < bytestream_len) {
        uint8_t c = bytestream[*streamindex];
        (*streamindex)++;

        // parser 2
        if (fmav_parse_to_msg(&rx_msg, &status, c) == FASTMAVLINK_PARSE_RESULT_OK) {
            return parse_byte_stream_one_message_test_msg(msgindex, &rx_msg);
        }
    }

    printf("SHIT13\n");
    return 0;
}


int parse_byte_stream(void)
{
    // parser 1
    int msgindex = 0;
    int streamindex = 0;

    for (int n = 0; n < MESSAGES_STREAM_NUM; n++) {
        if (!parse_byte_stream_one_message_w_parser1(&msgindex, &streamindex)) {
            printf("SHIT14\n"); return 0;
        };
    }

    // parser 2
    msgindex = 0;
    streamindex = 0;

    for (int n = 0; n < MESSAGES_STREAM_NUM; n++) {
        if (!parse_byte_stream_one_message_w_parser2(&msgindex, &streamindex)) {
            printf("SHIT15\n"); return 0;
        };
    }

    return 1;
}


void run_test_parser(void) 
{
    int is_ok;

    printf("generate\n");
    PRINT_RESET();
    is_ok = generate_byte_stream();
    if (!is_ok) return;

    printf("\nparse\n");
    PRINT_RESET();
    is_ok = parse_byte_stream();
    if (!is_ok) return;

    printf("\n");
}


//------------------------------
// main
//------------------------------

int main(int argc, char *argv[])
{
    srand((int)time(NULL)); // initialization, should only be called once.
    int r = rand(); // returns a pseudo-random integer between 0 and RAND_MAX.

    fmav_init();
    fmav_status_reset(&status);

    set_print_only_errors(0);

    int msg_num = fmav_get_message_entry_num();
    printf("msg num: %i\n", msg_num);

    int runs = 0;
    while (1) {
        if (RUNS && (runs >= RUNS)) break;
        printf("loop: %i\n", runs++);
        run_test_parser();
    }

    printf("Calculations finished.\n");
    return 0;
}

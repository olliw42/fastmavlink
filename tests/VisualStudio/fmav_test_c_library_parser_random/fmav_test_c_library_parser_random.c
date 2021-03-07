//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// fastMavlink Test Suite
// for Microsoft Visual Studio C
// parser random test
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

#include "pymavlink_c_library_v2/minimal/mavlink.h"
#include "test_c_library/minimal/test_minimal.h"

//#include "pymavlink_c_library_v2/common/mavlink.h"
//#include "test_c_library/common/test_common.h"

//#include "pymavlink_c_library_v2/ardupilotmega/mavlink.h"
//#include "test_c_library/ardupilotmega/test_ardupilotmega.h"

//#include "pymavlink_c_library_v2/all/mavlink.h"
//#include "test_c_library/all/test_all.h"


//------------------------------
// here we go
//------------------------------

// test parsers, method 2
// generates byte stream of N frames, which are created randomly, while adhering to MAVLink 
// plays it to the parser
// parses and checks against the known sequence of frames


#define BYTE_STREAM_LEN  16000 // (MESSAGES_STREAM_NUM * FASTMAVLINK_FRAME_LEN_MAX)


typedef uint8_t frame_t[FASTMAVLINK_FRAME_LEN_MAX];
frame_t framestream[MESSAGES_STREAM_NUM];

fmav_message_t msgstream[MESSAGES_STREAM_NUM];

int bytestream_len;
uint8_t bytestream[BYTE_STREAM_LEN];


int generate_byte_stream_one_frame(int* msgindex, int* streamindex)
{
    // random header & payload

    frame_t frame;
    frame[0] = FASTMAVLINK_MAGIC_V2;
    for (uint16_t i = 1; i < FASTMAVLINK_FRAME_LEN_MAX; i++) frame[i] = rand_uint8_t();
    
    uint32_t msgid = frame[7] + ((uint32_t)frame[8] << 8) + ((uint32_t)frame[9] << 16);
    printf("msgid: %i\n", msgid);

    uint16_t crc_extra;
    uint8_t payload_max_len;
    const fmav_message_entry_t* msg_entry = fmav_get_message_entry(msgid);
    if (msg_entry) {
        crc_extra = msg_entry->crc_extra;
        payload_max_len = msg_entry->payload_max_len;
    }
    else {
        // if message is not known, we don't know a crc extra, so use a random one
        crc_extra = rand_uint16_t();
        payload_max_len = rand_uint8_t();
        if (payload_max_len < frame[1]) payload_max_len = frame[1];
    }

    uint8_t len = fmav_payload_len_wo_trailing_zeros(&(frame[10]), payload_max_len);
    frame[1] = len;

    uint16_t crc = fmav_crc_calculate(&(frame[1]), FASTMAVLINK_HEADER_V2_LEN - 1);
    fmav_crc_accumulate_buf(&crc, &frame[10], len);
    fmav_crc_accumulate(&crc, crc_extra);

    frame[len + FASTMAVLINK_HEADER_V2_LEN] = (uint8_t)crc;
    frame[len + FASTMAVLINK_HEADER_V2_LEN + 1] = (uint8_t)(crc >> 8);

    uint16_t frame_len = len + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
    if (frame[2] & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
        frame_len += FASTMAVLINK_SIGNATURE_LEN;
    }

    // all ok, store

    memcpy(&(framestream[*msgindex]), frame, frame_len);
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
        if (!generate_byte_stream_one_frame(&msgindex, &streamindex)) {
            printf("SHIT 1\n"); return 0;
        };
    }

    bytestream_len = streamindex;
    return 1;
}


fmav_status_t status;
fmav_message_t rx_msg;
uint8_t rx_buf[296];


int parse_byte_stream_one_message_w_parser1(int* msgindex, int* streamindex)
{
    while (*streamindex < bytestream_len) {
        uint8_t c = bytestream[*streamindex];
        (*streamindex)++;

        fmav_result_t result;
        uint8_t res = fmav_parse_to_frame_buf(&result, rx_buf, &status, c);
        if (res == FASTMAVLINK_PARSE_RESULT_OK) {
            res = fmav_check_frame_buf(&result, rx_buf);
            if (res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res == FASTMAVLINK_PARSE_RESULT_OK) {
                
                uint32_t msgid = rx_buf[7] + ((uint32_t)rx_buf[8] << 8) + ((uint32_t)rx_buf[9] << 16);
                printf("msgid: %i\n", msgid);

                frame_t frame;
                memcpy(frame, &(framestream[*msgindex]), FASTMAVLINK_FRAME_LEN_MAX);
                (*msgindex)++;

                uint16_t frame_len = frame[1] + FASTMAVLINK_HEADER_V2_LEN + FASTMAVLINK_CHECKSUM_LEN;
                if (frame[2] & FASTMAVLINK_INCOMPAT_FLAGS_SIGNED) {
                    frame_len += FASTMAVLINK_SIGNATURE_LEN;
                }

                PRINT_TEST(result.frame_len == frame_len, "frame len");
                int is_ok = compare_frame_buf(rx_buf, frame, frame_len);
                PRINT_FRAMES(is_ok, "frame", rx_buf, result.frame_len, frame, frame_len);
                if (!is_ok) { printf("SHIT 10\n"); return 0; }

                if (res == FASTMAVLINK_PARSE_RESULT_OK) {
                    fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf);
                    printf("     msgid is known\n");
                }
            }

            return 1;
        }
    }

    printf("SHIT 11\n");
    return 0;
}


int parse_byte_stream_one_message_w_parser2(int* msgindex, int* streamindex)
{
    while (*streamindex < bytestream_len) {
        uint8_t c = bytestream[*streamindex];
        (*streamindex)++;

        // parser 2
        if (fmav_parse_to_msg(&rx_msg, &status, c) == FASTMAVLINK_PARSE_RESULT_OK) {
            printf("     msgid: %i\n", rx_msg.msgid);
        }

        return 1;
    }

    printf("SHIT 12\n");
    return 0;
}


int parse_byte_stream(void)
{
    // parser 1
    int msgindex = 0;
    int streamindex = 0;

    for (int n = 0; n < MESSAGES_STREAM_NUM; n++) {
        if (!parse_byte_stream_one_message_w_parser1(&msgindex, &streamindex)) {
            printf("SHIT 13\n"); return 0;
        };
    }

    // parser 2
/*
    msgindex = 0;
    streamindex = 0;

    for (int n = 0; n < MESSAGES_STREAM_NUM; n++) {
        if (!parse_byte_stream_one_message_w_parser2(&msgindex, &streamindex)) {
            printf("SHIT 14\n"); return 0;
        };
    }
*/
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

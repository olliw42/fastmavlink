//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// fastMavlink Test Suite
// for Microsoft Visual Studio C
// messages test
//------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <stdint.h>


//------------------------------
// test
//------------------------------

#define RUNS_PER_MESSAGE  10000


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

void run_test(void) 
{
  int test_total_cnt = 0;
  int error_total_cnt = 0;

  int msg_num = fmav_get_message_entry_num();
  //printf("msg num: %i\n", msg_num);

  for(int i = 0; i < msg_num; i++) {
    const fmav_message_entry_t* msg_entry = fmav_get_message_entry_by_index(i);
    uint32_t msgid = msg_entry->msgid;
    printf("msgid: %i\n", msgid);

    int msg_is_ok = 1;
    for(int n = 0; n < RUNS_PER_MESSAGE; n++) {
      PRINT_RESET();

      int is_ok = run_test_msg_one_by_msgid(msgid);
      test_total_cnt ++;
      if (!is_ok) error_total_cnt++;

      if (!is_ok) {
        printf(global_s); printf("\n"); 
        msg_is_ok = 0;
      }
    }
    if (msg_is_ok) { printf("OK\n"); }
  }

  printf("\n");
  printf("Result\n");
  printf("  message tested: %i\n", msg_num);
  printf("  tests total: %i\n", test_total_cnt);
  printf("  errors total: %i\n", error_total_cnt);

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

    //set_print_only_errors(0);

    run_test();

    printf("Calculations finished.\n");

    return 0;
}

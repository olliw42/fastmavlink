//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
// https://github.com/olliw42/fastmavlink/
//------------------------------
// fastMavlink Test Suite
// for Arduino
// messages test
//------------------------------
// Licence: This code is free and open and you can use it
// in whatever way you want. It is provided as is with no
// implied or expressed warranty of any kind.
//------------------------------

// this is extremely flash hungry, so it works only with minimal.xml on a bluepill :(

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>


#define SERIAL  Serial1
#define SERIAL_BAUD  115200 // 57600 or 115200 are usually good choices

#define BLINK_PIN  PA0
uint8_t blink = 0;

#define LED_TOGGLE   {digitalWrite(BLINK_PIN, (blink) ? HIGH : LOW);  blink = (blink) ? 0 : 1;}


//------------------------------
// test
//------------------------------

#define RUNS_PER_MESSAGE  1


//------------------------------
// random value generators
// needs to be included before dialects are included

#include "test_c_library/fastmavlink_test_random_generators.h"


#define FASTMAVLINK_TEST_GLOBAL_S_LEN  8000
#define SPRINTF(s,l,...) sprintf(s,__VA_ARGS__)
#define STRCAT(s,l,a) strcat(s,a)

#define PRINTF(s) SERIAL.write(s)


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

int test_total_cnt = 0;
int error_total_cnt = 0;

int msg_num = fmav_get_message_entry_num();
int msg_index = 0;


void run_test(void) 
{
  //printf("msg num: %i\n", msg_num);

  const fmav_message_entry_t* msg_entry = fmav_get_message_entry_by_index(msg_index);
  msg_index++;
  if (msg_index >= msg_num) msg_index = 0;

  uint32_t msgid = msg_entry->msgid;
  SERIAL.print("msgid: "); SERIAL.println(msgid);

  for(int n = 0; n < RUNS_PER_MESSAGE; n++) {
    PRINT_RESET();

    int is_ok = run_test_msg_one_by_msgid(msgid);
    test_total_cnt ++;
    if (!is_ok) error_total_cnt++;

    if (!is_ok){ PRINTF(global_s); PRINTF("\n"); } else { PRINTF("OK\n"); }

    if (!is_ok) while(1){} // let's be brutal and halt if we have an error !!!
  }

  PRINTF("\n");
}


//------------------------------
// Default Arduino setup() and loop() functions
//------------------------------

void setup()
{
  SERIAL.begin(SERIAL_BAUD);

  pinMode(BLINK_PIN, OUTPUT);

  fmav_init(); // let's always call it, even if it currently may not do anything
  
  //set_print_only_errors(0);
  
  srand((int)time(NULL)); // initialization, should only be called once.
  int r = rand(); // returns a pseudo-random integer between 0 and RAND_MAX.
}


void loop()
{
  if (SERIAL.available() > 0) return; // wait for serial buffer to be empty, to avoid buf overflow
  run_test();
  LED_TOGGLE;
}

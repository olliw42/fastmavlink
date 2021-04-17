//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
// https://github.com/olliw42/fastmavlink/
//------------------------------
// fastMavlink Example
// One Link - One Component
// The better2 implemenation.
// For details see fastMavlink github repo.
// For Arduino IDE.
//------------------------------
// Licence: This code is free and open and you can use it
// in whatever way you want. It is provided as is with no
// implied or expressed warranty of any kind.
//------------------------------
// Do this before you start:
// Copy the fastMavlink c_library folder into the sketch folder.
//------------------------------


//------------------------------
// Implement the interface assumed by the fastMavlink examples
//------------------------------
// Attention: The Arduino's buffer for serial write can be very limited (the docs say 64 bytes!),
// and can be too small for really doing MAVLink. The serial_has_space() function may hence trigger
// often and effectively block the code from working. In that case you may work around by letting it
// return true always

// Depending on board and core and setup, the main serial might be Serial, Serial1, ...
// Choose what's correct for you.

#define SERIAL  Serial1
#define SERIAL_BAUD  57600 // 57600 or 115200 are usually good choices


uint16_t serial_available(void)
{
    uint16_t available = SERIAL.available();
    return (available > 0) ? available : 0;
}

void serial_read_char(uint8_t* c)
{
    *c = SERIAL.read();
}

uint8_t serial_has_space(uint16_t count)
{
    return (SERIAL.availableForWrite() >= count) ? 1 : 0;
}

void serial_write_buf(uint8_t* buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) SERIAL.write(buf[i]);
}

uint32_t get_time_ms()
{
    return millis();
}


// let's do this in addition, have a LED on a pin to visually see some action
#define BLINK_PIN  PA0
uint8_t blink = 0;

#define LED_TOGGLE   {digitalWrite(BLINK_PIN, (blink) ? HIGH : LOW);  blink = (blink) ? 0 : 1;}


//------------------------------
// Include the fastMavlink library C code
//------------------------------
// For this example, common.xml is a good choice for the dialect,
// but choose whichever dialect you prefer.

#define FASTMAVLINK_SERIAL_WRITE_CHAR
void fmav_serial_write_char(uint8_t c)
{
    SERIAL.write(c); 
}


// The "..../common/common.h" is the way how fastMavlink wants it to be.
// If you would do ".../common/mavlink.h" like you would for the pymavlink-mavgen
// library, you would enable fastMavlink's pymavlink-mavgen mimicry. The code would
// still work, but we want to do fastMavlink here :).

#include "c_library/common/common.h"

// Arduino IDE does some automatic changes to the code. This prevents it inserting
// a function prototype in the wrong place.

void handleMessage(fmav_message_t* msg);


//------------------------------
// Here it comes, the example code
//------------------------------

uint8_t my_sysid = 1; // match it to the sys id of your autopilot

uint8_t my_compid = MAV_COMP_ID_PERIPHERAL; // match it to what your component wants to be

// this is needed to keep various info
fmav_status_t status;

// this holds the received message, and is also the working buffer for the parser
fmav_message_t msg;

// some variables we need

uint8_t send_statustext = 0;

uint32_t tlast_ms = 0;


// send the "Hello World" STATUSTEXT message
// returns 1 if successful, 0 else
uint8_t sendStatustext(void)
{
  // We use here the payload structure and the xxx_encode_to_serial() function.
  // This is for demonstrational purposes, but in case of STATUSTEXT it also makes sense
  // as we need some buffer to deal with the text array anyways.

  fmav_statustext_t payload;

  payload.severity = MAV_SEVERITY_INFO;
  memset(&payload.text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  memcpy(&payload.text, "Hello World", 11);
  payload.id = 0;
  payload.chunk_seq = 0;
  
  // This is an example where the serial's transmit buffer might be too small, and 
  // serial_has_space() to prevent things to work. In my case on a bluepill it does
  // so, and I hence have to comment this check out.
  // The method of using xxx_encode_to_frame_buf(), as in the "better" example, which
  // gives the actual frame length, which here is much shorter than the mx length, is
  // preferred.
  
  // if (!serial_has_space(FASTMAVLINK_MSG_STATUSTEXT_FRAME_LEN_MAX)) return 0;

  fmav_msg_statustext_encode_to_serial(my_sysid, my_compid, &payload, &status);

  return 1; // we have successfully send it, so tell that
}


// send a HEARTBEAT message
// returns 1 if successful, 0 else
uint8_t sendHeartbeat(void)
{
  // Here we don't use the payload structure and xxx_encode() function, but rather pack the data
  // directly into the serials' transmit buffer using the xxx_pack_to_serial() function. Just to
  // show also this. This is however by no really much better RAM wise than the technique used in
  // the "better" example, since a payload structure is put on stack inside the function. 

  if (!serial_has_space(FASTMAVLINK_MSG_HEARTBEAT_FRAME_LEN_MAX)) return 0;

  fmav_msg_heartbeat_pack_to_serial(
    my_sysid, my_compid,
    MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE,
    &status);

  return 1; // we have successfully send it, so tell that
}


// our message handler
void handleMessage(fmav_message_t* msg)
{
  switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_HEARTBEAT: {

      // We use here the payload structure and the xxx_decode() function.
      // This can stress the stack, since for some messages the payload can be large.
      // Despite that it often might be the best option. Here we shall be fine with it.

      fmav_heartbeat_t payload;
      fmav_msg_heartbeat_decode(&payload, msg);

      // Here you now can do something with the data in the payload.
      // We do this: When we observe the autopilot, we simply trigger the emission of
      // a "Hello World" statustext message.

      // This is a major flaw of MAVLink, there is no defined unambiguous procedure for
      // identifying the nature of a component. So we do some simple heuristics here
      // (which may be too simplistic for more realistic applications).

      // un-comment this if you want to detect the flight controller
      // if (payload.autopilot != MAV_AUTOPILOT_INVALID && msg->compid == MAV_COMP_ID_AUTOPILOT1) {
      // un-comment this if you want to detect the GCS
      if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
        send_statustext = 1;
      }
      }break;
  }
}


// this should be called repeatedly
void spinOnce(void)
{
  // let's first check and do incoming messages

  uint16_t available = serial_available();
  for (uint16_t i = 0; i < available; i++) {
    uint8_t c;
    serial_read_char(&c);

    uint8_t res = fmav_parse_to_msg(&msg, &status, c);

    if (res == FASTMAVLINK_PARSE_RESULT_OK) {
      if (fmav_msg_is_for_me(my_sysid, my_compid, &msg)) {
        handleMessage(&msg);
      }
    }
  }

  // let's now send a STATUSTEXT message, if it has been triggered

  if (send_statustext) {
    if (sendStatustext()) {
      send_statustext = 0; // we have successfully send it, so clear that flag
    }
  }

  // let's now send a HEARTBEAT message, to tell the world we exist

  uint32_t tnow_ms = get_time_ms();
  if ((tnow_ms - tlast_ms) > 1000) {
    if (sendHeartbeat()) {
      tlast_ms += 1000; // we have successfully send it, so do it again later
      LED_TOGGLE;
    }
  }
}


//------------------------------
// Default Arduino setup() and loop() functions
//------------------------------

void setup()
{
  SERIAL.begin(SERIAL_BAUD);

  pinMode(BLINK_PIN, OUTPUT);

  fmav_init(); // let's always call it, even if it currently may not do anything
}


void loop()
{
  spinOnce();
}

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
// https://github.com/olliw42/fastmavlink/
//------------------------------
// fastMavlink Example
// One Link - One Component
// The detailed/elementary implemenation.
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

// The "..../common/common.h" is the way how fastMavlink wants it to be.
// If you would do ".../common/mavlink.h" like you would for the pymavlink-mavgen
// library, you would enable fastMavlink's pymavlink-mavgen mimicry. The code would
// still work, but we want to do fastMavlink here :).

#include "c_library/common/common.h"

// Arduino IDE does some automatic changes to the code. This prevents it inserting
// a function prototype in the wrong place.

uint8_t isForMe(fmav_message_t* msg);
void handleMessage(fmav_message_t* msg);


//------------------------------
// Here it comes, the example code
//------------------------------

uint8_t my_sysid = 1; // match it to the sys id of your autopilot

uint8_t my_compid = MAV_COMP_ID_PERIPHERAL; // match it to what your component wants to be

// this is needed to keep various info
fmav_status_t status;

// in this example we use only elementary functions, and hence need a parse buffer
// it should be at least of size FASTMAVLINK_FRAME_LEN_MAX, we keep here some extra bytes
uint8_t rx_buf[296];

// we could put it on the stack, but since it is a large data field
// it may be less stressfull to define it explicitly here
// we however can use it for both receiving and sending as it is needed only temporarily
fmav_message_t msg;

// we could put it on the stack, but since it is a large data field
// it may be less stressfull to define it explicitly here
uint8_t tx_buf[296];

// some variables we need

uint8_t send_statustext = 0;

uint32_t tlast_ms = 0;


// check the message's target ids to determine if the message is for us
// returns 0: not for us, 1: is for us
uint8_t isForMe(fmav_message_t* msg)
{
  // The message either has no target_sysid or is broadcast, so accept
  if (msg->target_sysid == 0) return 1;

  // The message has a target_sysid but it is not ours, so reject
  if (msg->target_sysid != my_sysid) return 0;

  // The message either has no target_compid or is broadcast, so accept
  if (msg->target_compid == 0) return 1;

  // The message has a target_compid and it is ours, so accept
  if (msg->target_compid == my_compid) return 1;

  // The message has a target_compid but it is not ours, so reject
  return 0;
}


// send the "Hello World" STATUSTEXT message
// returns 1 if successful, 0 else
uint8_t sendStatustext(void)
{
  // We use here the payload structure and the xxx_encode() function.
  // This is for demonstrational purposes, but in case of STATUSTEXT it also makes sense
  // as we need some buffer to deal with the text array anyways.

  fmav_statustext_t payload;

  payload.severity = MAV_SEVERITY_INFO;
  memset(&payload.text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  memcpy(&payload.text, "Hello World", 11);
  payload.id = 0;
  payload.chunk_seq = 0;

  uint16_t count = fmav_msg_statustext_encode(&msg, my_sysid, my_compid, &payload, &status);

  if (serial_has_space(count)) {
    fmav_msg_to_frame_buf(tx_buf, &msg);
    serial_write_buf(tx_buf, count);
    return 1; // we have successfully send it, so tell that
  }

  return 0;
}


// send a HEARTBEAT message
// returns 1 if successful, 0 else
uint8_t sendHeartbeat(void)
{
  // Here we don't use the payload structure and xxx_encode() function, but rather pack the data
  // directly into the message structure using the xxx_pack() function. Just to show also this.

  uint16_t count = fmav_msg_heartbeat_pack(
                        &msg,
                        my_sysid,
                        my_compid,
                        MAV_TYPE_GENERIC, // type, this is a major flaw of MAVLink, there is no suitable type for us
                        MAV_AUTOPILOT_INVALID, // autopilot, we aren't one, so that's what we should use
                        MAV_MODE_FLAG_SAFETY_ARMED, // base_mode, we are always armed
                        0, // custom_mode, we don't have any
                        MAV_STATE_ACTIVE, // system_status, we are always active
                        &status);

  if (serial_has_space(count)) {
    fmav_msg_to_frame_buf(tx_buf, &msg);
    serial_write_buf(tx_buf, count);
    return 1; // we have successfully send it, so tell that
  }

  return 0;
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

    fmav_result_t result;
    uint8_t res = fmav_parse_to_frame_buf(&result, rx_buf, &status, c);

    // fmav_parse_to_frame_buf() is a very fast parser, as it just aims at catching a frame, with no sanity checks.
    // Result can be PARSE_RESULT_NONE, PARSE_RESULT_HAS_HEADER, or PARSE_RESULT_OK
    // The result is also stored in result.res, so we do not really need res and could have used result.res instead.
    // We also could have also written
    // if (fmav_parse_to_frame_buf(&result, rx_buf, &status, c) == FASTMAVLINK_PARSE_RESULT_OK) {}.

    if (res == FASTMAVLINK_PARSE_RESULT_OK) {
      res = fmav_check_frame_buf(&result, rx_buf);

      // fmav_check_frame_buf() does sanity checks (checksum), and also determines meta data such as target ids, which
      // all is relatively costly to do.
      // Result can be PARSE_RESULT_MSGID_UNKNOWN, PARSE_RESULT_LENGTH_ERROR, PARSE_RESULT_CRC_ERROR,
      // PARSE_RESULT_SIGNATURE_ERROR, or PARSE_RESULT_OK
      // We could check for other results than OK, but we are here only interested in those messages
      // which we know and possibly want to digest.
      // Again the result is also stored in result.res, and we do not really need res, but could have used result.res or
      // written if (fmav_check_frame_buf(&result, tx_buf) == FASTMAVLINK_PARSE_RESULT_OK) {}.

      if (res == FASTMAVLINK_PARSE_RESULT_OK) {
        fmav_frame_buf_to_msg(&msg, &result, rx_buf);

        // fmav_frame_buf_to_msg() copies the data as well as some meta data into the message structure.
        // It assumes that the data in the working buffer is valid, i.e., that the above checks have been passed.

        if (isForMe(&msg)) { handleMessage(&msg); }
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

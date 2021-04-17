//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
// https://github.com/olliw42/fastmavlink/
//------------------------------
// fastMavlink Example
// Parameters
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

void handleMessage(fmav_message_t* msg);


//------------------------------
// Here it comes, the example code
//------------------------------

// Define the parameters
// FASTMAVLINK_PARAM_NUM and the fmav_param_list[] must be defined
// before fastmavlink_parameters.h is included.
// The parameters are not stored permanently, that's beyond the reach of this
// example.

uint8_t  Pu8 = 100;
int8_t   Ps8 = -2;
uint16_t Pu16 = 500;
int16_t  Ps16 = 123;
uint32_t Pu32 = 500;
int32_t  Ps32 = 123;
float    Pf   = 0.011;

// We use a macro generator to create fmav_param_list[]

#define FASTMAVLINK_PARAM_LIST \
    X( Pu8,    UINT8,  P_U8)\
    X( Ps8,    INT8,   P_S8)\
    X( Pu16,   UINT16, P_U16)\
    X( Ps16,   INT16,  P_S16)\
    X( Pu32,   UINT32, P_U32)\
    X( Ps32,   INT32,  P_S32)\
    X( Pf,     REAL32, P_FLOAT)\

#define UINT8   uint8_t
#define INT8    int8_t
#define UINT16  uint16_t
#define INT16   int16_t
#define UINT32  uint32_t
#define INT32   int32_t
#define REAL32  float

#define FASTMAVLINK_PARAM_NUM  7

const fmav_param_entry_t fmav_param_list[FASTMAVLINK_PARAM_NUM] = {
    #define X(p,t,n) {(t*)&(p), MAV_PARAM_TYPE_##t, #n },
    FASTMAVLINK_PARAM_LIST
    #undef X
};

#include "c_library/lib/fastmavlink_parameters.h"


// Helpers to send the parameter list

uint8_t send_param_value = 0;
uint16_t send_param_value_index;


void param_send_param_value_trigger(uint16_t index)
{
    send_param_value = 1;
    send_param_value_index = index;
}


#define PARAM_SEND_STREAM_RATE_MS  200

uint16_t param_send_next_index = FASTMAVLINK_PARAM_NUM;
uint32_t param_send_tlast_ms = 0;


void param_send_stream_start(void)
{
    param_send_next_index = 0;
    param_send_tlast_ms = get_time_ms();
}


void param_send_stream_do(void)
{
    if (param_send_next_index >= FASTMAVLINK_PARAM_NUM) return; // nothing to do

    uint32_t tnow_ms = get_time_ms();
    if ((tnow_ms - param_send_tlast_ms) > PARAM_SEND_STREAM_RATE_MS) {
        param_send_param_value_trigger(param_send_next_index);
        param_send_tlast_ms += PARAM_SEND_STREAM_RATE_MS;
        param_send_next_index++;
    }
}


// Set things up
// Much of this you have seen aleady in the example "One Link - One Component".

uint8_t my_sysid = 1; // match it to the sys id of your autopilot

uint8_t my_compid = MAV_COMP_ID_PERIPHERAL; // match it to what your component wants to be

fmav_status_t status;
fmav_message_t msg;
uint8_t tx_buf[296];

// some variables we need

uint8_t send_statustext = 0;
uint32_t tlast_ms = 0;


// send the "Hello World" STATUSTEXT message
// returns 1 if successful, 0 else
uint8_t sendStatustext(void)
{
  fmav_statustext_t payload;

  payload.severity = MAV_SEVERITY_INFO;
  memset(&payload.text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
  memcpy(&payload.text, "Hello World", 11);
  payload.id = 0;
  payload.chunk_seq = 0;

  uint16_t count = fmav_msg_statustext_encode_to_frame_buf(tx_buf, my_sysid, my_compid, &payload, &status);

  if (serial_has_space(count)) {
    serial_write_buf(tx_buf, count);
    return 1; // we have successfully send it, so tell that
  }

  return 0;
}


// send a HEARTBEAT message
// returns 1 if successful, 0 else
uint8_t sendHeartbeat(void)
{
  uint16_t count = fmav_msg_heartbeat_pack_to_frame_buf(
    tx_buf, my_sysid, my_compid,
    MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE,
    &status);

  if (serial_has_space(count)) {
    serial_write_buf(tx_buf, count);
    return 1; // we have successfully send it, so tell that
  }

  return 0;
}


// send a PARAM_VALUE message
// returns 1 if successful, 0 else
uint8_t sendParamValue(uint16_t index)
{
  // For sending a parameter entry it is most convenient to us the xxx_encode()
  // set of functions.

  fmav_param_value_t payload;
  if (!fmav_param_get_param_value(&payload, index)) return 0;

  uint16_t count = fmav_msg_param_value_encode_to_frame_buf(tx_buf, my_sysid, my_compid, &payload, &status);

  if (serial_has_space(count)) {
    serial_write_buf(tx_buf, count);
    return 1;
  }

  return 0;
}


// our message handler
void handleMessage(fmav_message_t* msg)
{
  switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_HEARTBEAT: {
      fmav_heartbeat_t payload;
      fmav_msg_heartbeat_decode(&payload, msg);

      // un-comment this if you want to detect the flight controller
      // if (payload.autopilot != MAV_AUTOPILOT_INVALID && msg->compid == MAV_COMP_ID_AUTOPILOT1) {
      // un-comment this if you want to detect the GCS
      if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
        send_statustext = 1;
      }
      }break;

    // Here they come, the handlers for the PARAM messages
      
    case FASTMAVLINK_MSG_ID_PARAM_REQUEST_READ:{
      fmav_param_request_read_t payload;
      fmav_msg_param_request_read_decode(&payload, msg);
      uint16_t index;
      if (fmav_param_do_param_request_read(&index, &payload)) {
        param_send_param_value_trigger(index);
      }
      }break;

    case FASTMAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      param_send_stream_start();
      }break;

    case FASTMAVLINK_MSG_ID_PARAM_SET:{
      fmav_param_set_t payload;
      fmav_msg_param_set_decode(&payload, msg);
      uint16_t index;
      if (fmav_param_do_param_set(&index, &payload)) {
        fmav_param_set_value(index, payload.param_value);
        param_send_param_value_trigger(index);
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

  // let's do params if stream requested
  
  param_send_stream_do();

  // let's now send PARAM_VALUE message, if it has been triggered, and no other message is pending

  if (send_param_value) {
    if (sendParamValue(send_param_value_index)) {
      send_param_value = 0; // we have successfully send it, so clear that flag
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

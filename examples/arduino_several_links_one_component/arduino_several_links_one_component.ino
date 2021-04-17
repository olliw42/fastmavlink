//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
// https://github.com/olliw42/fastmavlink/
//------------------------------
// fastMavlink Example
// Several Links - One Component: Component with Router Capabilities
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

#define SERIAL1  Serial1
#define SERIAL1_BAUD  57600 // 57600 or 115200 are usually good choices

#define SERIAL2  Serial2
#define SERIAL2_BAUD  57600 // 57600 or 115200 are usually good choices


uint16_t serial1_available(void)
{
    uint16_t available = SERIAL1.available();
    return (available > 0) ? available : 0;
}

void serial1_read_char(uint8_t* c)
{
    *c = SERIAL1.read();
}

uint8_t serial1_has_space(uint16_t count)
{
    return (SERIAL1.availableForWrite() >= count) ? 1 : 0;
}

void serial1_write_buf(uint8_t* buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) SERIAL1.write(buf[i]);
}


uint16_t serial2_available(void)
{
    uint16_t available = SERIAL2.available();
    return (available > 0) ? available : 0;
}

void serial2_read_char(uint8_t* c)
{
    *c = SERIAL2.read();
}

uint8_t serial2_has_space(uint16_t count)
{
    return (SERIAL2.availableForWrite() >= count) ? 1 : 0;
}

void serial2_write_buf(uint8_t* buf, uint16_t len)
{
    for(uint16_t i = 0; i < len; i++) SERIAL2.write(buf[i]);
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


// The ".../common/common.h" is the way how fastMavlink wants it to be.
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

// Set up the router
// We need three links, one for the component, and two for the two serials.
// link 0 = component
// link 1 = serial1
// link 2 = serial2

#define FASTMAVLINK_ROUTER_LINKS_MAX        3
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX   12

#include "c_library/lib/fastmavlink_router.h"


// Define the parameters
// This you have seen aleady in the example "Parameters"
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
// Some fields have to be multiplied however, for handling the three links

uint8_t my_sysid = 1; // match it to the sys id of your autopilot

uint8_t my_compid = MAV_COMP_ID_PERIPHERAL; // match it to what your component wants to be

// We need a status for each each link
// For link 1 and 2 it is required to keep the parser state for receiving
// For link 0 it is required to keep the seq for sending
fmav_status_t status0, status1, status2;

// We need receive working buffer for the two serials
uint8_t rx_buf1[296], rx_buf2[296];

// Receive message structure for the component
fmav_message_t rx_msg;

// Send  message structure for the component
fmav_message_t tx_msg;
uint8_t tx_msg_available = 0;

// This could be put on the stack, but we want to have it explicit
uint8_t tx_buf[296];

// some variables we need

uint8_t send_statustext = 0;
uint32_t tlast_ms = 0;


// generate the "Hello World" STATUSTEXT message
void generateStatustext(void)
{
    fmav_statustext_t payload;

    payload.severity = MAV_SEVERITY_INFO;
    memset(&payload.text, 0, FASTMAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
    memcpy(&payload.text, "Hello World", 11);
    payload.id = 0;
    payload.chunk_seq = 0;

    fmav_msg_statustext_encode(&tx_msg, my_sysid, my_compid, &payload, &status0);
    tx_msg_available = 1;
}


// generate a HEARTBEAT message
void generateHeartbeat(void)
{
    fmav_msg_heartbeat_pack(
        &tx_msg, my_sysid, my_compid,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE,
        &status0);
    tx_msg_available = 1;
}


// generate a PARAM_VALUE message
void generateParamValue(uint16_t index)
{
    fmav_param_value_t payload;
    if (!fmav_param_get_param_value(&payload, index)) return;

    fmav_msg_param_value_encode(&tx_msg, my_sysid, my_compid, &payload, &status0);
    tx_msg_available = 1;
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

    // read serial1
    // en detail
    uint16_t available1 = serial1_available();
    for (uint16_t i = 0; i < available1; i++) {
        uint8_t c;
        serial1_read_char(&c);

        fmav_result_t result;
        uint8_t res;
        
        // can return PARSE_RESULT_NONE, PARSE_RESULT_HAS_HEADER, or PARSE_RESULT_OK
        res = fmav_parse_to_frame_buf(&result, rx_buf1, &status1, c);

        // a complete frame has been received, not yet validated
        if (res == FASTMAVLINK_PARSE_RESULT_OK) {
          
            // can return MSGID_UNKNOWN, LENGTH_ERROR, CRC_ERROR, SIGNATURE_ERROR, or OK
            res = fmav_check_frame_buf(&result, rx_buf1);

            // we want to forward also unknown messages
            if (res == FASTMAVLINK_PARSE_RESULT_MSGID_UNKNOWN || res == FASTMAVLINK_PARSE_RESULT_OK) {

                // let's determine if it is targeted for any link
                fmav_router_handle_message(1, &result);

                // if it is for link 1, send the frame buffer
                if (fmav_router_send_to_link(1)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!

                // if it is for link 2, send the frame buffer
                if (fmav_router_send_to_link(2)) { serial2_write_buf(rx_buf1, result.frame_len); }

                // if it is a known message and is for us, handle the message
                if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {

                     // we first need to convertthe frame buffer to a message structure
                    fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf1);

                    // now we are ready to handle the received message
                    handleMessage(&rx_msg);
                }
            } 
        }
    }

    // read serial2
    // uses fmav_parse_and_check_to_frame_buf() for compactness
    uint16_t available2 = serial2_available();
    for (uint16_t i = 0; i < available2; i++) {
        uint8_t c;
        serial2_read_char(&c);
        fmav_result_t result;
        if (fmav_parse_and_check_to_frame_buf(&result, rx_buf2, &status2, c)) {
            fmav_router_handle_message(2, &result);
            if (fmav_router_send_to_link(1)) { serial1_write_buf(rx_buf1, result.frame_len); }
            if (fmav_router_send_to_link(2)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {
                fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf2);
                handleMessage(&rx_msg);
            }
        }
    }

    // let's do params if stream requested
  
    param_send_stream_do();

    // let's now send PARAM_VALUE message, if it has been triggered, and no other message is pending

    if (!tx_msg_available && send_param_value) {
        generateParamValue(send_param_value_index);
        send_param_value = 0;
    }

    // let's now send a STATUSTEXT message, if it has been triggered, and no other message is pending

    if (!tx_msg_available && send_statustext) {
        generateStatustext();
        send_statustext = 0;
    }

    // let's now send a HEARTBEAT message, if no other message is pending, to tell the world we exist

    uint32_t tnow_ms = get_time_ms();
    if (!tx_msg_available && (tnow_ms - tlast_ms) > 1000) {
        generateHeartbeat();
        tlast_ms += 1000; // we have successfully send it, so do it again later
        LED_TOGGLE;
    }

    // send out pending message
    
    if (tx_msg_available) {
        fmav_router_handle_message_by_msg(0, &tx_msg);
        if (fmav_router_send_to_link(1) || fmav_router_send_to_link(2)) {
            uint16_t count = fmav_msg_to_frame_buf(tx_buf, &tx_msg);
            if (serial1_has_space(count) && serial1_has_space(count)) {
                if (fmav_router_send_to_link(1)) serial1_write_buf(tx_buf, count);
                if (fmav_router_send_to_link(2)) serial2_write_buf(tx_buf, count);
                tx_msg_available = 0;
            }
        } else {
            tx_msg_available = 0; //message is targeted at unknown component
        }
    }
}


//------------------------------
// Default Arduino setup() and loop() functions
//------------------------------

void setup()
{
    SERIAL1.begin(SERIAL1_BAUD);
    SERIAL2.begin(SERIAL2_BAUD);

    pinMode(BLINK_PIN, OUTPUT);

    fmav_init(); // let's always call it, even if it currently may not do anything

    fmav_status_reset(&status0);
    fmav_status_reset(&status1);
    fmav_status_reset(&status2);
    
    fmav_router_reset();

    // This is important, otherwise the router would not know about us.
    fmav_router_add_ourself(my_sysid, my_compid);
}


void loop()
{
    spinOnce();
}

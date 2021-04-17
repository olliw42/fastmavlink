
# Example: One Link - One Component #

This is the simplest and probably most typical type of application. It consists of an electronic device which should behave as a MAVLink component and which is connected to the MAVLink network via a serial connection.

The examples below implement a component, which sends out a `HEARTBEAT` message at intervals of 1 Hz, and listens to incoming `HEARTBEAT` messages from a flight controller or GCS (you need to select) and sends out a "Hello World" `STATUSTEXT` message as response.

In order to check its functioning, you can connect to a GCS like MissionPlanner. (the component does not have parameters, but MissionPlanner tries for quite some time to get them, so just close the dialog which MissionPlanner has opened)

Two implementations are provided. The first is most detailed in the sense that it implements each discrete step. It thus shows which tasks are involved in receiving, handling, and sending, and may provide the background for more complicated applications. For the given application (one link - one component), the possible simplifications and improvements are discussed, and based on this the second implementation is presented.

Arduino sketches are included for the code examples (I have tested them on a STM32F103 bluepill).


## Implementation Using Elementary Functions ##

Please inspect the code, and especially the comments.


```C
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
    }
  }
}
```

See the Arduino sketch [arduino_one_link_one_component_elementary.ino](arduino_one_link_one_component_elementary/arduino_one_link_one_component_elementary.ino) for this implementation.

The example is interesting as it shows in detail each and every single step which is involved, and might be helpful for approaching more complicated applications. For the given purpose it is however only of educational value, since the implementation can be significantly improved in several places:


- Instead of `fmav_parse_to_frame_buf()` and `fmav_check_frame_buf()` we could use the higher-level function `fmav_parse_and_check_to_frame_buf()`, which does the same. This code part would then much simplify and become


```C
    fmav_result_t result;
    if (fmav_parse_and_check_to_frame_buf(&result, rx_buf, &status, c)) {
      fmav_frame_buf_to_msg(&msg, &result, rx_buf);
      if (isForMe(&msg)) { handleMessage(&msg); }
    }
```

- The check of the target IDs of the received message done with `isForMe()` could be moved up a bit, to after `fmav_check_frame_buf()`, since the target IDs are already available at this point in the result structure. This would reduce CPU load since the subsequent `fmav_frame_buf_to_msg()` would not be done for all received messages but only for those which are targeted at us. With a suitably adapted function `isForMe(fmav_result_t* res)` the code part would then become (this technique will not be applicable in the below, but is mentioned here for education)


```C
    fmav_result_t result;
    if (fmav_parse_and_check_to_frame_buf(&result, rx_buf, &status, c)) {
      if (isForMe(&result)) {
        fmav_frame_buf_to_msg(&msg, &result, rx_buf);
        handleMessage(&msg);
      }
    }
```

- Since we do not need the frame itself but just the message structure, the parser could directly parse into the message structure. That is, instead of the `rx_buf` working buffer one can use a message structure. This is also somewhat better on CPU cycles as it avoids back-and-forth copying. Note that we need to check again the result, unlike with the higher-level functions. The code would much simplify to


```C
    uint8_t res = fmav_parse_to_msg(&msg, &status, c);
    if (res == FASTMAVLINK_PARSE_RESULT_OK) {
      if (isForMe(&msg)) { handleMessage(&msg); }
    }
```

- Similarly we do not need the message structure when sending, and could pack the data directly into the `tx_buf` working buffer, which avoids the intermediate step via a message structure. The code part for sending the `HEARTBAT` message would then become (and we can do so similarly for sending the 'STATUSTEXT' message)

```C
  uint16_t count = fmav_msg_heartbeat_pack_to_frame_buf(
                        tx_buf,
                        my_sysid,
                        my_compid,
                        MAV_TYPE_GENERIC,
                        MAV_AUTOPILOT_INVALID,
                        MAV_MODE_FLAG_SAFETY_ARMED,
                        0,
                        MAV_STATE_ACTIVE,
                        &status);

  if (serial_has_space(count)) {
    serial_write(tx_buf, count);
    return 1;
  }

  return 0;
```

- We can use the helper function `fmav_msg_is_for_me()` instead of implementing `isForMe()` manually.

- Another improvement, which often can provide quite significant gains in flash and CPU cycles, but which may not be so obvious at first, is this: The meta data such as the extra crc and other info which is needed in order to check the correctness of an received message and determine its target IDs is stored in flash as an array `FASTMAVLINK_MESSAGE_CRCS`, which has to be searched to find the meta data for the received message. Per default this array includes the meta data of all messages of the dialect. However, in many applications one is interested only in a relatively small subset of these messages, and thus only the meta data for these messages have to be in that array. This brings two advantages: Since the array is smaller less flash is consumed, and for the same reason the search will be faster. This can be achieved by inserting the following lines before the dialect (here common.xml) is included:

```C
  #include "path_to_code_generator_output/common/common_msg_entries.h"
  #define FASTMAVLINK_MESSAGE_CRCS {\
    FASTMAVLINK_MSG_ENTRY_HEARTBEAT,\
  }
```

## Better Implementation ##

The example can thus also be achieved by this code (with most comments removed for brevity):


```C
uint8_t my_sysid = 1;

uint8_t my_compid = MAV_COMP_ID_PERIPHERAL;

// this is needed to keep various info
fmav_status_t status;

// this holds the received message, and is also the working buffer for the parser
fmav_message_t msg;

// we could put it on the stack, but since it is a large data field
// it may be less stressfull to define it explicitly here
uint8_t tx_buf[296];

// some variables we need

uint8_t send_statustext = 0;

uint32_t tlast_ms = 0;

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
    return 1;
  }

  return 0;
}

uint8_t sendHeartbeat(void)
{
  uint16_t count = fmav_msg_heartbeat_pack_to_frame_buf(
    tx_buf, my_sysid, my_compid,
    MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_SAFETY_ARMED, 0, MAV_STATE_ACTIVE,
    &status);

  if (serial_has_space(count)) {
    serial_write_buf(tx_buf, count);
    return 1;
  }

  return 0;
}

void handleMessage(fmav_message_t* msg)
{
  switch (msg->msgid) {
    case FASTMAVLINK_MSG_ID_HEARTBEAT: {
      fmav_heartbeat_t payload;
      fmav_msg_heartbeat_decode(&payload, msg);
      // if (payload.autopilot != MAV_AUTOPILOT_INVALID && msg->compid == MAV_COMP_ID_AUTOPILOT1) {
      if (payload.autopilot == MAV_AUTOPILOT_INVALID && payload.type == MAV_TYPE_GCS) {
        send_statustext = 1;
      }
      }break;
  }
}

void spinOnce(void)
{
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

  if (send_statustext) {
    if (sendStatustext()) {
      send_statustext = 0;
    }
  }

  uint32_t tnow_ms = get_time_ms();
  if ((tnow_ms - tlast_ms) > 1000) {
    if (sendHeartbeat()) {
      tlast_ms += 1000;
    }
  }
}
```

We have reduced the RAM footprint, have reduced the stack load somewhat, and have saved CPU cycles. And the code has become cleaner. Not that bad.

See the Arduino sketch [arduino_one_link_one_component_better.ino](arduino_one_link_one_component_better/arduino_one_link_one_component_better.ino) for this implementation.


## Further Possible Improvements ##

The "better" implementation may not yet seem perfect in these points:

- In `handleMessage()` a payload structure is used to digest the data in the message. Some messages can have large payloads (up to 255 bytes!) and this thus can stress the stack. As alternative it is possible to instead directly extract the data from the message structure, which can be more economical. FastMavlink is supporting this via the `fmav_msg_xxx_get_field_yyy()` functions, but in my experience it is rarely usefull. It should be considered if the values of only few fields of a large payload are needed. An example could be `AUTOPILOT_VERSION`, where one is often interested only in the `capabilities` field. Otherwise, decoding the payload is nearly always more efficient, and cleaner.

- In the sending functions `sendStatustext()` and `sendHeartbeat()`, one could test for sufficient space in the serial transmit buffer via `serial_has_space()` before the message is packed, which would avoid the effort for packing in cases the serial transmit buffer is full. However, one would then have to test for the maximum length of the message frame, which can be overly restrictive, since the actual length can be smaller than the maximal length due to the trailing zero byte trunction in the MAVLink v2 protocol. Examples would be the `STATUSTEXT` message, but also the `BATTERY_STATUS` message and others. In many cases these tend to be significantly shorter. Furthermore, in carefully designed implementations it should happen only occasionally that the serial transmit buffer is out of space.

- In the sending functions `sendStatustext()` and `sendHeartbeat()`, the `tx_buf` buffer could be avoided, if the data of the message would be directly packed into the serial's transmit buffer. This indeed looks very interesting. However, for any reasonably generic MAVLink C library the payload has to be placed into an intermediate payload structure anyways, so that there is not much gain RAM/stack wise, it is just "hidden" from the users eyes. Furthermore, in my experience, except for simple applications, it tends to create more complications than it is helpful.

See the Arduino sketch [arduino_one_link_one_component_better2.ino](arduino_one_link_one_component_better2/arduino_one_link_one_component_better2.ino) for an implementation considering the last two points.

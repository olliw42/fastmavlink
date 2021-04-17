
## Example: Several Links - One Component: Component with Routing Capabilities ##

This example implements a component with two serial ports, which both can be connected to the MAVLink network, using the tool box provided by `fastmavlink_router.h`. 

It builds on the [One Link - One Component](/examples/arduino_one_link_one_component) and [Parameters](/examples/arduino_parameters) examples. It implements a component, which sends out a `HEARTBEAT` message at intervals of 1 Hz, and listens to incoming `HEARTBEAT` messages from a flight controller or GCS (you need to select) and sends out a "Hello World" `STATUSTEXT` message as response, and reacts to the `PARAM_REQUEST_READ`, `PARAM_REQUEST_LIST`, and `PARAM_SET` messages. In addition it routes messages between the two serial ports.

In order to check its functioning, you can connect either serial to a GCS like MissionPlanner and the other to a flight controller.

The Arduino sketch [arduino_several_links_one_component.ino](arduino_several_links_one_component.ino) is included (I have tested it on a STM32F103 bluepill).


## Basic Principles and Code: Receiving and Parsing ##

In principle, also for a router all one needs to do is to receive a message, handle it, and send it out, very much like for a component, and all this were discussed already before in the [One Link - One Component](/examples/arduino_one_link_one_component) example. However, there are some catches which makes the receiving/parsing of a message different for a router.

The key difference is that a parser for a router needs to be able to parse (and forward) both known and unknown messages. This is not only mandated by the MAVLink standard ([here](https://mavlink.io/en/guide/routing.html)), but in fact also by common sense. As discussed in the [One Link - One Component](/examples/arduino_one_link_one_component) example, each message comes with some meta data, such as the extra crc but also the target ID locations, which are needed in order to validate the message and determine its target IDs, and which are stored in an extra array in flash. A "known" message is a message for which there is an entry in this array, while for an "unknown" message there is not. Hence, an unknown message cannot be validated nor can its target IDs be determined (that's a huge flaw of the MAVLink protocol design, but it is how it is). This is not a problem for a component, since by definition a component only can and wants to handle known messages. However, a parser for a router needs to also handle unknown messages, and forward also these. If the router wouldn't, it would be like that you could send a pdf document per email, but not a xml document, unless you convince an international body to include that data format, and so on for all new file formats, which obviously would be super annoying. Unfortunately, some MAVLink routers only handle known messages, such as the MAVLink router implemented in ArduPilot.

Another difference should be speed. The parser for a router should be optimized for speed, in order to reduce latency, but also CPU cycles, which can be important since a router obviously has to handle much more traffic than a component. The key trick here is to avoid all steps, which are not needed in order to parse and forward a message, and are only needed for when a component wants to handle them. For instance, it is not neccessary to pack the received frame into a message structure, and unpack it back into a frame for sending, since the router simply can send out the original frame again (this is actually also mandated by the MAVLink standard). That's how good and fast routers should do it, [mavlink-routerd](https://github.com/mavlink-router/mavlink-router) would be an example, and fastMavlink too ;)

In fastMavlink, these points are realized by a careful separation of the discrete receiving/parsing steps into discrete functions:

- `fmav_parse_to_frame_buf()`: This parses the received bytes into a working buffer, as fastly as possible, i.e., without doing any additional work like calculating a checksum. As result it can return `PARSE_RESULT_NONE`, `PARSE_RESULT_HAS_HEADER`, or `PARSE_RESULT_OK`, where `PARSE_RESULT_OK` means a complete frame has been received (that's what we are interested in).

- `fmav_check_frame_buf()`: This checks the data in the working buffer, as fastly as possible. It first determines if the message is known or unknown. If unknown, it jumps out and returns `PARSE_RESULT_MSGID_UNKNOWN`. If known, it checks the data, e.g., if the checksum is correct. If not, it jumps out and returns an error result, such as `PARSE_RESULT_CRC_ERROR`. If the checks were passed, it determines the target IDs, and returns `PARSE_RESULT_OK`.

Note that for a component, only frames with a known message ID and which have passed the checks are of relevance, i.e., only frames with result `PARSE_RESULT_OK` would be further processed, while for a router both frames with result `PARSE_RESULT_MSGID_UNKNOWN` and `PARSE_RESULT_OK` should be further processed. Note also that it all happens on the working buffer, no packing/repacking into other structures is needed nor done.

- `fmav_frame_buf_to_msg()`: This converts the data in the working buffer into a message structure. This step is not needed for a router, but is needed when a component wants to handle the message.

Equipped with this understanding, the code for a router which has to handle three serial ports (= link 0, 1, 2) could look as follows:


```C
    // link 0 = serial1
    // link 1 = serial2
    // link 2 = serial3

    // read serial1
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
                fmav_router_handle_message(0, &result);

                // if it is for link 0, send the frame buffer
                if (fmav_router_send_to_link(0)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!

                // if it is for link 1, send the frame buffer
                if (fmav_router_send_to_link(1)) { serial2_write_buf(rx_buf1, result.frame_len); }

                // if it is for link 2, send the frame buffer
                if (fmav_router_send_to_link(2)) { serial3_write_buf(rx_buf1, result.frame_len); }
            } 
        }
    }
```

Similar pieces of code need to be added for reading the other two serial ports. Note that for link 0 (= serial1 here) frames received on link 0 shall not be forwarded to link 0, and similarly for the other links (serials).

We here are however interested in a component with routing capabilities, and the code for handling messages needs to be added in. By default, the fastMavlink router library assumes that the component is on link 0, so that serial1 is now link 1, and so on. The code for a component with three serial ports (= link 1, 2, 3) then becomes:

```C
    // link 0 = component
    // link 1 = serial1
    // link 2 = serial2
    // link 3 = serial3

    // read serial1
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

                // if it is for link 3, send the frame buffer
                if (fmav_router_send_to_link(3)) { serial3_write_buf(rx_buf1, result.frame_len); }

                // if it is a known message and is for us, handle the message
                if (res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {

                     // we first need to convert the frame buffer to a message structure
                    fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf1);

                    // now we are ready to handle the received message
                    handleMessage(&rx_msg);
                }
            } 
        }
    }
```

This detailed code should demonstrate how it all works. The code can however be simplified quite a bit by using the higher-level wrapper function `fmav_parse_and_check_to_frame_buf()`, which combines `fmav_parse_to_frame_buf()` and `fmav_check_frame_buf()` into one function, which returns 0 or 1. The same code fragment when becomes (with some comments omitted):

```C
    // link 0 = component
    // link 1 = serial1
    // link 2 = serial2
    // link 3 = serial3

    // read serial1
    uint16_t available1 = serial1_available();
    for (uint16_t i = 0; i < available1; i++) {
        uint8_t c;
        serial1_read_char(&c);
        fmav_result_t result;
        if (fmav_parse_and_check_to_frame_buf(&result, rx_buf1, &status1, c)) {
            fmav_router_handle_message(1, &result);
            if (fmav_router_send_to_link(1)) {} // WE DO NOT REFLECT, SO THIS MUST NEVER HAPPEN !!
            if (fmav_router_send_to_link(2)) { serial2_write_buf(rx_buf1, result.frame_len); }
            if (fmav_router_send_to_link(3)) { serial3_write_buf(rx_buf1, result.frame_len); }
            if (result.res == FASTMAVLINK_PARSE_RESULT_OK && fmav_router_send_to_link(0)) {
                fmav_frame_buf_to_msg(&rx_msg, &result, rx_buf1);
                handleMessage(&rx_msg);
            } 
        }
    }
```

The router capability is fully implemented at this point of affairs. However, the component may also want to send out messages, which will be discussed in the next section. 


## Basic Principles and Code: Sending ##

As said, the component usually also wants to send out messages, and if it is only a `HEARTBEAT` to announce its existence. In contrast to the situation for [One Link - One Component](/examples/arduino_one_link_one_component), the message however also has to potentially be send to all serial ports, depening on its target IDs. This code part is very dependend on how the handling of outgoing messages is actually implemented, so the following code should just give the flair of it (again three serial ports = link 1, 2, 3 are assumed):

```C
    // send out pending message
    if (tx_msg_available) {

        // let's determine if the message is targeted for any link
        // remind that link 0 is us
        fmav_router_handle_message_by_msg(0, &tx_msg);

        // we only need to and should do anything if it should go to any serial port
        if (fmav_router_send_to_link(1) || fmav_router_send_to_link(2) || fmav_router_send_to_link(3)) {

            // we need to convert the message structure into a frame buffer
            // this also tells us the length of the frame
            uint16_t count = fmav_msg_to_frame_buf(tx_buf, &tx_msg);

            // we only send it out if it can be send to all ports it should go to
            if (serial1_has_space(count) && serial2_has_space(count) && serial3_has_space(count)) {

                // now we can send them to all ports it should go to
                if (fmav_router_send_to_link(1)) serial1_write_buf(tx_buf, count);
                if (fmav_router_send_to_link(2)) serial2_write_buf(tx_buf, count);
                if (fmav_router_send_to_link(3)) serial3_write_buf(tx_buf, count);

                tx_msg_available = 0;  // we have successfully send it, so clear flag
            }
        } else {
            tx_msg_available = 0; // message is targeted at an unknown component, so clear flag
        }
    }
```

## Pain Points ##

The proper scheduling of the handling and sending of messages can be a challenge and the scheme presented in the above code fragments is somewhat simplistic.

- The router is given priority over the component, by the fact that it does not check has_space() but simply puts the received frame into the serial transmit buffers. These buffers should thus be large enough to hold at least as many frames as there are links.

- It can happen that handleMessage() is called several times before the component is given a change to send out own messages. This can lead to a loss of outgoing messages without further precautions. A remedy is to implement a FIFO for the outgoing messages. Another simple precaution is to ensure by some additonal flags that handleMessage() is only called once per loop (pay attention to not give any serial port priority).

- A component's outgoing message is simply postponed until it can be send out on all serial ports. This, theoretically, can starve out the component if there is lots of traffic on the serial ports, since has_space() may not become true sufficiently often.

The last two points obviously apply only to the componet part, but not the routing part.

The problems at hand here are very similar to those in Ethernet routers, and solutions developped there may be taken over with benefit. However, in practice, simple schemes do work well in nearly all situations.


## The fastMavlink Router Library ##

Although it was used already in the above, the fastMavlink router library `lib/fastmavlink_router.h` is worth some words. The library has to ackomplish two main tasks:

- It needs to maintain a list of components (identified by its source sysid and compid), which it has seen on each link. This is important in order to be able to determine which message has to go to which link, and to which link it should not go. An exemption is given by the `HEARTBEAT` message: It must be send to all links, irrespective on whether a component was seen on the link or not (also few other messages may require special treatmeant such as `RADIO_STATUS`). Various philosophies for detecting a component on a link are possible (the MAVLink standard is not very precise here). For instance, one can accept any message with a given source sysid-compid as evidence for the presence of the component. Another approach is to only look for the `HEARTBEAT` messages. Also, various mechanisms are possible to clear a component from the list, e.g., a timeout or a change of link or simply just never (the MAVLink standard specifies looking for `SYSTEM_TIME`). The fastMavlink router library currently looks for the `HEARTBEAT`message and never clears a component from the list (it would be however simple to extend the library to other schemes). 

- For a given message, the library needs to provide a mechanism which allows us to determine to which links the message should go to. This is generally determined from the target IDs of the message, with the exception of some messages which need special handling such as the `HEARTBEAT` message.

To set up the fastMavlink router library, one needs to define the tokens `FASTMAVLINK_ROUTER_LINKS_MAX`, which specifies the number of links, and `FASTMAVLINK_ROUTER_COMPONENTS_MAX`, which specifies the size of the component list, before the library is included. For instance:

```C
#define FASTMAVLINK_ROUTER_LINKS_MAX  4
#define FASTMAVLINK_ROUTER_COMPONENTS_MAX  12
#include "path_to_code_generator_output/lib/fastmavlink_router.h"
```

For any received message, one needs to call the function `fmav_router_handle_message()` or `fmav_router_handle_message_by_msg()`, which maintains the component list. These functions also fill an internal array, which can be accessed via the function `fmav_router_send_to_link()`. The usage of these functions is shown in the code examples in the above.


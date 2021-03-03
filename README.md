
# The fastMavlink Library #

FastMavlink is designed to be the most lightweight and most performant [MAVLink](https://mavlink.io/en/) C library, additionally providing novel features.

It may not reach the goals to 100.00%, not now and not then, but as compared to the standard [pymavlink-mavgen](https://github.com/ArduPilot/pymavlink) library it provides much improved performance, in terms of CPU time, flash, and RAM footprint, and capabilities.

This is not achieved by some magic vodoo coding tricks, but simply by a careful design which avoids superfluous calculations, obscure data fields, and unnecessary use of RAM and stack.

To give an example: In order to parse a message, determine if it is targeted at the application, and send a response out to the proper link, the pymavlink-mavgen library requires you to search three (3!) times for the target id pairs which correspond to the respective message id. The fastMavlink library only requires one search, and this is the minimum which is logically needed. Searching is a comparatively costly process, and avoiding unnecessary searches obviously relates to a boost in performance. 

Some of the drawbacks of the pymavlink-mavgen library were listed and addressed in [mavlink/#1127](https://github.com/mavlink/mavlink/pull/1127), which can serve to further demonstrate the point. However, fastMavlink's C code is not simply an improved version of pymavlink-mavgen's, but pretty much a complete rewrite from scratch. It is therefore also clean, logically structured, and cruft were removed. It inherits some ideas however, such as the header-only design and organization into dialect subfolders, and code for few basic functions.

In addition, the fastMavlink C library provides features not provided otherwise but which are quite missed. For instance, it has optimized routines for use in MAVLink routers as well as a MAVLink router library. It offers a pymavlink-mavgen mimicry capability, which can make changing to fastMavlink easy.

Lastly, the C code is generated using a Python generator from the MAVLink protocol XML definition files, as it is common with MAVLink. The code generator is based off pymavlink-mavgen's, but has been massively renovated, cleaned up, and more logically structured. It also provides new capabilities. For instance, it provides consistent code generation across included dialects, which removes the hickups possible with pymavlink-mavgen. It therefore is well prepared to provide the means for the upcoming MAVLink governance policy, such as message overwrite for development and testing.

You don't believe all this can be true, you think it must be exaggerated? Well, when please check it out and judge :)

The fastMavlink library is used in two projects of mine, the [STorM32 gimbal controller](http://www.olliw.eu/2013/storm32bgc/) and the [MAVLink for OpenTx](http://www.olliw.eu/2020/olliwtelem/) projects. So, it can claim some maturity. Yet, obviously, the software is offered as is with no explicit or implied warranty, and there is plenty of room to further improve, extend and advance it. Suggestions are welcome.

## Licence ##

Yet to be determined. It will be a permissive licence, along the ideas of the original MAVLink project. That is, the fastMavlink C code library canbe used in a closed source application without copyright issues and without any limitations.

## Limitations ##

The parser can read MAVLink v1 and v2 messages, including the signing packet, and forward them, but it cannot decode signed messages.

Messages can be generated and emitted only in MAVLink v2 format, and without signature. (It would be easy to extend the library to allow sending v1 messages, but, frankly, there should be really no need for sending v1 messages nowadays)


## C Code Usage ##

Please see the following chapter on the [C Code Architecture](#c-code-architecture) for a general description. 

For examples please go to [Examples](examples/).


In order to use the dialect dialect.xml, include 

```C
#include "path_to_code_generator_output/dialect/dialect.h"
```

into your project. Note that it is not `".../dialect/mavlink.h"` as it would be for pymavlink-mavgen. If you do so with fastMavlink, it would enable the [pymavlink-mavgen mimicry](#pymavlink-mavgen-mimicry).


## C Code Architecture ##

The C code was architected along the line of the tasks which need to be covered for reading/parsing, handling, and sending MAVLink messages. That is, for each discrete task a function exists, which can then be combined to more user-friendly and easy-to-use higher-level functions, some of which are provided too. This way, only the tasks have to be executed as they are needed for the particular application at hand, allowing to minimize CPU time, RAM and stack usage.

For instance, parsing a message for a MAVLink component with routing capabilities involves these discrete tasks: 

- parsing the received bytes into a working buffer, only considering the information in its header, mainly the len field (= very fast parsing)
- checking if the message ID is known, checksum is ok, and determining the target IDs
- forwarding the data in the working buffer to the proper links (no extra effort needed)
- converting the data in the working buffer into a message structure holding the required and relevant information, which can be passed on to the component's message handler

This ensures a most fast parsing and a minimal effort (= CPU time) for forwarding of known and unknown messages, and the proper design of the message structure minimizes the effort (= CPU time) for handling. The discrete tasks are represented by corresponding functions, namely `fmav_parse_to_frame_buf()`, `fmav_check_frame_buf()`, and `fmav_frame_buf_to_msg()`, which can be called in sequence when (and only when) needed. But also the higher-level functions `fmav_parse_and_check_to_frame_buf()` and `fmav_parse_to_msg_wbuf()` are available, which wrap these steps. One also can do it in one "big" step, and directly parse the received bytes into a message structure (i.e., the intermediate working buffer is not needed) with the function `fmav_parse_to_msg()`, which would be preferred then the working buffer content is not needed.

In the following the discrete tasks shall be analyzed, as this should help much to understand the fabric of the fastMavlink library.


### Receiving/Parsing

Overview of primitive tasks:

| 1 |   | 2 |   | 3 |   | 4 |   | 5 |   | 6 |
|---|---|---|---|---|---|---|---|---|---|---|
|Rx |->| buf |->| check |->| msg_t |->| payload_t |->| data |

Receiving/parsing can be disected into (up to) 6 discrete steps: 

The received byte (Rx) is parsed into a working buffer (buf), the information in which is then checked (check), and if good converted into a message structure (msg_t), which can be passed on to the message handler. The message handler typically will decode the payload into a payload structure (payload_t), in order to access the data in the individual message fields (data). Importantly, it is not necessary that each step is explicitely executed, and in fact this is usually not optimal. For instance, in simpler applications one may want to parse the received bytes (Rx) directly into a message structure (msg_t), i.e., do Rx -> msg_t, and this would reduce RAM footprint as the working buffer (buf) is then not needed.

#### fmav_parse_to_frame_buf():
1 -> 2, Rx -> buf
- takes c of Rx and fills buf
- parses as economically as possible
- located in fastmavlink_functions.h


#### fmav_check_frame_buf():
2 -> 3, buf -> check
- checks if msgid known, length ok, crc ok, retrieves target ids
- located in fastmavlink_functions.h


#### fmav_frame_buf_to_msg():
3 -> 4, check -> msg_t
- basically copy and fill, with keeping relevant information
- located in fastmavlink_functions.h


#### fmav_parse_to_msg():
1 -> 4, Rx  ->  msg_t
- parses directly from Rx into msg structure
- located in fastmavlink_functions.h


#### fmav_msg_xxx_decode():
4 -> 5, msg_t -> payload_t
- located in mavlink_msg_xxx.h


#### fmav_msg_xxx_get_field_yyy():
4 -> 6, msg_t -> sata
- located in mavlink_msg_xxx.h


#### fmav_parse_and_check_to_frame_buf():
1 -> 2 -> 3, Rx -> buf -> check
- wrapper to the first two steps
- located in fastmavlink_functions.h


#### fmav_parse_to_msg_wbuf():
1 -> 2 -> 3 -> 4, Rx -> buf -> check -> msg_t
- wrapper to the first three steps
- located in fastmavlink_functions.h



### Sending

Overview of primitive tasks:

| 1 |   | 2 |   | 3 |   | 4 |   | 5 |
|---|---|---|---|---|---|---|---|---|
|data|->| payload_t |->| msg_t |->| buf |->| Tx |

Sending can be disected into (up to) 5 discrete steps:

The data in the message fields (data) is encoded into a payload structure (payload_t), which is then packed into a message structure (msg_t), then converted into a working buffer (buf), which can be directly send out (Tx). As before, it is not necessary that each step is explicitely executed, and in fact this is usually not optimal. For instance, in simpler applications one may want to pack the data for the message fields (data) directly into the working buffer (buf), i.e., do data -> buf, and this would reduce RAM and/or stack footprint as neither the payload structure (payload_t) nor the message structure (msg_t) need to be involved, and it also would save CPU cycles.


#### fmav_msg_xxx_pack():
1 -> 3, data  ->  msg_t
- located in mavlink_msg_xxx.h


#### fmav_msg_xxx_encode():
2 -> 3, payload_t -> msg_t
- located in mavlink_msg_xxx.h


#### fmav_msg_to_frame_buf():
3 -> 4, msg_t  ->   buf
- located in fastmavlink_functions.h


#### fmav_msg_xxx_pack_to_frame_buf():
1 -> 4, data ->  buf
- located in mavlink_msg_xxx.h


#### fmav_msg_xxx_encode_to_frame_buf():
2 -> 4, payload_t ->  buf
- located in mavlink_msg_xxx.h


#### fmav_msg_xxx_pack_to_serial():
1 -> 2 -> 5, data -> payload_t ->  Tx
- user has to provide a function fmav_serial_write_char()
- is not directly packing into the Tx buffer but is using payload_t on stack
- located in mavlink_msg_xxx.h


#### fmav_msg_xxx_encode_to_serial():
2 -> 5, payload_t -> Tx
- user has to provide a function fmav_serial_write_char()
- located in mavlink_msg_xxx.h


#### fmav_msg_to_serial():
3 -> 5, msg_t -> Tx
- user has to provide a function fmav_serial_write_char()
- located in fastmavlink_functions.h


## Pymavlink-mavgen Mimicry ##

The fastMavlink C code library includes function wrappers which mimic those of the pymavlink-mavgen library. This allows us to easily port to fastMavlink, with no or little effort in many cases. The mimicry is activated by including

```C
#include "path_to_code_generator_output/dialect/mavlink.h"
```

instead of `".../dialect/dialect.h"`. This defines the token `FASTMAVLINK_PYMAVLINK_ENABLED`, which in turn enables the related code. The mimicry works as drop-in-replacement. That is, fastMavlink's enums, structures, and functions are actually used, but simply presented with a different look. Additional work for converting to fastMavlink will typically be required if fields of pymavlink-mavgen's status and message structures are directly used, since they may not be present in fastMavlink's structures.




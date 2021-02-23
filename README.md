
# The FastMavlink Library #

FastMavlink is designed to be the most lightweight and most performant MAVLink C library, additionally providing novel features.

It may not reach the goals to 100.00%, not now and not then, but as compared to the standard pymavlink-mavgen library https://github.com/ArduPilot/pymavlink) it provides drastically improved performance, in terms of cpu time, flash, and RAM footprint, and capabilities.

This is not achieved by some magic vodoo coding tricks, but simply by a careful design which avoids superfluous calculations, obscure data fields, and unnecessary use of RAM and stack.

To give an example: In order to parse a message, determine if it is targeted at the application, and send a response out to the proper link, the pymavlink-mavgen library requires you to search three (3!) times for the target id pairs which correspond to the respective message id. The fastMavlink library only requires one search, and this is the minimum which is logically needed. Searching is a comparatively costly process, and avoiding unnecessary searches obviously relates to a boost in performance. 

Some of the drawbacks of the pymavlink-mavgen library were listed and addressed in https://github.com/mavlink/mavlink/pull/1127, which can serve to further demonstrate the point. However, fastMavlink's C code is not simply an improved version of pymavlink-mavgen's, but pretty much a complete rewrite from scratch. It is therefore also clean, logically structured, and cruft were removed. It inherits some ideas however, such as the header-only design.

In addition, the fastMavlink C library provides features not provided otherwise but which are quite missed. For instance, it has optimized routines for use in MAVLink routers as well as a MAVLink router library. It offers a pymavlink-mavgen mimicry capability, which can make changing to fastMavlink easy.

Lastly, the C code is generated using a Python generator from the MAVLink protocol XML definition files, as it is common with MAVLink. The code generator is based off pymavlink-mavgen's, but has been massively renovated, cleaned up, and more logically structured. It also provides new capabilities. For instance, it provides consistent code generation across included dialects, which removes the hickups possible with pymavlink-mavgen. It therefore is well prepared to provide the means for the upcoming MAVLink governance policy, such as message overwrite for development and testing.

You don't believe this all this can be true, you think it must be exaggerated? Well, when please check it out and judge :)

The fastMavlink library is used in two projects of mine, the STorM32 gimbal controller and the MAVLink for OpenTx projects. So, it can claim some maturity. Yet, obviously, there is plenty of room to further improve, extend and advance it. Suggestions are welcome.

## Licence ##

yet to be determined

## Limitations ##

The parser can read MAVLink v1 and v2 messages, including the signing packet, and forward them, but it cannot decode signed messages.

Messages can be generated and emitted only in MAVLink v2 format and without signature. (The limitation of only v2 messages could be easily removed, but, frankly, there should be really no need for that nowadays)

## C Code Architecture ##

The C code was architected along the line of the tasks which need to be covered for reading/parsing, handling, and sending/emitting MAVLink messages. That is, for each discrete task a function exists, which can then be combined to more user-friendly and easy-to-use higher-level functions, some of which are provided too. This way, only the tasks have to be executed as they are needed for the particular application at hand, allowing to minimize cpu time, RAM and stack usage.

For instance, parsing a message for a MAVLink component with routing capabilities involves these discrete tasks: 

- parsing the received bytes into a working buffer, only considering the information in its header, mainly the len field
- checking if the message ID is known, and determine e.g. the target IDs
- forwarding the data in the working buffer to the proper links (no extra effort needed)
- converting the data in the working buffer into a message structure holding the required information, which can be feed to the component's message handler

This ensures a most fast parsing and a minimal effort (= cpu time) for forwarding known and unknown messages, and the proper design of the message structure minimizes the effort (= cpu time) for handling. The discrete tasks are represented by corresponding discrete functions, namely `fmav_parse_to_frame_buf()`, `fmav_check_frame_buf()`, and `fmav_frame_buf_to_msg()`. But also the higher-level functions `fmav_parse_and_check_to_frame_buf()` or `fmav_parse_to_msg()` are available.

In the following the discrete tasks shall be carefully analyzed, as this should help much to understand the fabric of the fastMavlink library.


### Reading/Parsing:
----------------

Overview of primitive tasks:

1       2         3           4             5			6
Rx  ->  buf   ->  check   ->  msg_t   ->    payload_t	->	data


fmav_parse_to_frame_buf():
1   ->  2
Rx      buf
takes c of Rx and fills buf
parses as economically as possible
located in fastmavlink_functions.h


fmav_check_frame_buf():
2   ->  3
buf     check
checks if msgid known, length ok, crc ok, retrieves target ids
located in fastmavlink_functions.h


fmav_frame_buf_to_msg():
3     ->  4
check     msg_t
basically copy and fill, with keeping relevant information
located in fastmavlink_functions.h


fmav_msg_xxx_decode():
4     ->  5
msg_t     payload_t
located in mavlink_msg_xxx.h


fmav_parse_and_check_to_frame_buf():
1       3
Rx  ->  check
wrapper to the first two steps
located in fastmavlink_functions.h


fmav_parse_to_msg_wbuf():
1       4
Rx  ->  msg_t
wrapper to the first three steps
located in fastmavlink_functions.h


fmav_parse_to_msg():
1       4
Rx  ->  msg_t
parses directly from Rx into msg
located in fastmavlink_functions.h


missing:
msg_t -> data


### Sending/Emitting:
-----------------

Overview of primitive tasks:

1         2             3           4         5
data  ->  payload_t ->  msg_t  ->   buf   ->  Tx


fmav_msg_xxx_pack():
1         3
data  ->  msg_t
located in mavlink_msg_xxx.h


fmav_msg_xxx_encode():
2         	3
payload_t	->  msg_t
located in mavlink_msg_xxx.h


fmav_msg_to_frame_buf():
3           4
msg_t  ->   buf
located in fastmavlink_functions.h


fmav_msg_xxx_pack_to_frame_buf():
1         4
data  ->  buf
located in mavlink_msg_xxx.h


fmav_msg_xxx_encode_to_frame_buf():
2           4
payload_t   ->  buf
located in mavlink_msg_xxx.h


## C Code Usage ##

In order to use the dialect dialect.xml, include 

```#include "path_to_code_generator_output/dialect/dialect.h"```


## Pymavlink-mavgen Mimicry ##

The FastMavlink C code library includes function wrappers which mimic those of the pymavlink-mavgen library. This allows us to easily port to fastMavlink, with no or little effort in many cases. The mimicry is activated by including

```#include "path_to_code_generator_output/dialect/mavlink.h"```

instead of `".../dialect/dialect.h"`. This defines the token `FASTMAVLINK_PYMAVLINK_ENABLED`, which in turn enables the related code. The mimicry works as drop-in-replacement. That is, fastMavlink's enums, structs, and functions are actually used, but simply presented with a different look. Additional work for converting to fastMavlink will hence typically be required if fields of pymavlink-mavgen's status and message structs are directly used, since they are not be present in fastMavlink's structs.




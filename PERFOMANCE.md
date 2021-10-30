
# The fastMavlink Library: Efficiency and Performance #

As said in the introduction, the fastMavlink library can provide much improved performance, in terms of CPU time, flash, stack and RAM footprint. This shall be discussed to some extend in the following, in order to help to make best use of the library when resources are constraint.

First off, it should be clear that the more functionality is needed the more resources are needed, and that for each functionality some amount of resources is needed. There ain't no such thing as a free lunch :).

The library aims at achieving its goals by avoiding unneccessary buffers and allocations, providing a versatile set of functions which are optimized for the tasks, and several flags to adapt the behavior.

In the following, only the core libraries will be considered, the "higher-level" libraries such as the router and parameter libraries are not discussed.

## Stack ##

With one exception, _**all**_ functions use less than 16 bytes of stack (most of them actually use just few or none bytes).

The one exception are the `fmav_msg_XXX_pack_to_serial()` functions, which put the payload structure for the respective message onto the stack. The stack burden thus depends on the message, i.e., the message's maximal length, and can be as large as 255 bytes (FASTMAVLINK_PAYLOAD_LEN_MAX = 255). The burden can obviously be lessend by simply not sending messages with a large payload with theses functions, if this is a viable option.

_**Comment**: The `fmav_msg_XXX_pack_to_serial()` functions could be redesigned to avoid having to put the payload on stack, but the resulting code would be somewhat "ugly" and given that these functions are probably rarely used it was not done. Please raise an issue if you need this._

## RAM ##

The RAM footprint can be precisely controlled by the choice of functions which are used for receiving/parsing and generating/sending.

In general, in some way or the other, a working buffer of 280 bytes (FASTMAVLINK_FRAME_LEN_MAX = 280) plus some additional dozen bytes for holding auxiliary information are needed at minimum.

_**Comment**: The size of 280 bytes is mainly determined by the requirement of the parsers that a full MAVLINK frame must be stored. This in principle could be lessened by making the parsers ignore messages with a length larger than a configurable threshold. Such a feature could easily be added to the library. Please raise an issue if you need this._

Concerning the choice of functions to use, it is difficult to give some general recommendation, as it very much depends on the particular application. The chapter on the 'C Code Architecture' should however give the information needed in order to select the best functions for receiving/parsing and generating/sending. Further information and explanation is also available in the examples.

It is possible to get along with only the one working buffer of size 280 bytes plus the auxiliary bytes, as the application code can (often or in simpler cases) be structured such that it is used for both receiving/parsing and generating/sending. The functions `fmav_parse_to_msg()`, `fmav_msg_XXX_get_field_YYY()` and `fmav_msg_XXX_pack()` would e.g. be suitable, and would require only a `fmav_message_t` and a `fmav_status_t` structure.

In addition to the RAM required by the fastMavlink functions, RAM is often also needed for handling the serial UART, in form of receive and transmit buffers. The receive buffer can in principle be avoided by calling a parser function in the receive interrupt. In that case one probably would want to not perform the check of the message (i.e. not invoke `fmav_check_frame_buf()`) as this involves a binary search and can be slow on systems which are that resource constraint that one is considering this approach. The transmit buffer could be avoided by a blocking serial write function and using the `fmav_msg_XXX_pack_to_serial()` functions. The baudrate of MAVLink connections are typically low and thus small receive/transmit buffers can be acceptable (very resource constraint systems obviously won't be able to handle high baudrates, TANSTAAFL).

The token `FASTMAVLINK_RAM_SECTION` (see fastmavlink_config.h) is not affecting the core libraries.

## Flash ##

Besides the number of functions which are used in the application, the flash footprint is to a large extend determined by the message entry list. It is stored in the static array `_fmav_message_crcs[]` (see fastmavlink_functions.h) and is required by the parsers in order to check the CRC of a received message and extract the target ids. Since this is a huge structure, of several kB size for the common dialects, much flash (as well as CPU time) can be saved by tackling it.

Generally, and important to understand, the message entry list is only used for receiving/parsing (and routing). Therefore, all of the following does not affect or limit in any way the ability to generate/send messages.

The fastMavlink library helps minimizing the message entry list and thus reducing its flash footprint in three ways:

* It only stores data which is needed. In comparison to pymavlink-mavgen it is one byte less per message.

* It provides the token `FASTMAVLINK_MESSAGE_ENTRY_MSGID24BIT` which you can define. Its effect is that the size of a message entry is reduced by another byte such that it fits into 8 bytes. Depending on the alignment for your system and compiler this can lead to significantly reduced flash space. For instance, on STM32 systems this saves 4 bytes per message which for a typical size of a dialect of about 250-300 messages is a saving of 1 kB and more. The drawback could be a possibly somewhat slower access (but again, this depends very much on your system). For further details on usage please consult the description in fastmavlink_config.h.

* It provides the token `FASTMAVLINK_MESSAGE_CRCS` which allows you to redefine the message entry list. This allows you to include in the message entry list only those messages which you actually want to receive and digest. This can be a surprisingly small number of messages even for sophisticated MAVLink components. For instance, for the STorM32 controller, which arguably is a sophisticated MAVLink device, the list comprises only 25 messages resulting in a gain in flash of more than 3 kB. For further details on usage please consult the description in fastmavlink_config.h.

## CPU Time ##

CPU-time wise, the fastMavlink C code is not optimized in the sense that it optimizes to the cycle. The compiler would probably do a better job in optimizing to the cylce anyhow when compiling with -O3. Also, in several places a compromise had to be made between CPU-cycle and RAM, and sometimes code readability. The critical reader will thus have no difficulty to find lines to change when fighting for the last cycle is the goal. 

The code is however optimized in the sense that it avoids doing things which it logically does not have to do in order to accomplish a task, or doing a task twice, and so on. 

The result is a set of functions which break down the various tasks into suitable smallest entities, among which you can choose to get the best for your specific task. Information collected in one function is carried over to other functions by a structure of few auxiliary bytes (see `fmav_result_t` and `fmav_message_t` in fastmavlink_types.h). The fastMavlink C code is quite systematic in this point, and probably the most striking example is that the message entry list is searched only once, instead of twice (or more times) as with pymavlink-mavgen. 

While talking about the message entry list, obviously, reducing it via the `FASTMAVLINK_MESSAGE_CRCS` mechanism described before not only can save lots of flash, but also lots of CPU time, since the search through that list will then be much faster.

The fastMavlink C code also benefits from avoiding unneccessary copying of data. For instance, it avoids intermediate stack space and associated copying in the `fmav_msg_XXX_pack()` functions, in contrast to pymavlink-mavgen's analogs.








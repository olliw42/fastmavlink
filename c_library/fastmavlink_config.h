//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_CONFIG_H
#define FASTMAVLINK_CONFIG_H


#ifndef FASTMAVLINK_RAM_SECTION
  #define FASTMAVLINK_RAM_SECTION  static
#endif


#ifndef FASTMAVLINK_FUNCTION_DECORATOR
  #define FASTMAVLINK_FUNCTION_DECORATOR  static inline
#endif


#ifndef FASTMAVLINK_PACK
  #ifdef __GNUC__
    #define FASTMAVLINK_PACK( __Declaration__ ) __Declaration__ __attribute__((packed))
  #else
    #define FASTMAVLINK_PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
  #endif
#endif


#ifndef FASTMAVLINK_HEARTBEAT_MAVLINK_VERSION
  #define FASTMAVLINK_HEARTBEAT_MAVLINK_VERSION  3 // you should usually not modify this
#endif


// The receive functions, i.e. the parser, always zerofill the  payload, as this must be done for
// correct operation of the unpack/decode/get_field functions.
// However, the message generator functions, i.e. the pack/encode functions, do not need to do
// this for correct operation when used in the typical use case, which consists of generating and
// then directly sending the message. However, when unpack/decode/get_field functions are called
// on the generated messages, this can cause malfunction, as they require zerofilled payloads.
// Zerrofilling is somewhat costly.
// Hence this flag allows us to control if it should be done always or not.
// It is set to always (= 1) per default, to prevenet unexpected results for the novice. This has
// a performance cost, so disable for best performance and call fmav_msg_zerofill() if really needed.
#ifndef FASTMAVLINK_ALWAYS_ZEROFILL
  #define FASTMAVLINK_ALWAYS_ZEROFILL  1 // always zerofills payload
#endif


/*
#define FASTMAVLINK_MESSAGE_CRCS
*/


/*
#define FASTMAVLINK_SERIAL_WRITE_CHAR
*/


/*
// Allows to overwrite the build date of the library. You should usually not need this.
#define FASTMAVLINK_BUILD_DATE
*/


/*
// Allows to overwrite the version specified in the dialect xml file. You should usually
// not need this.
#define FASTMAVLINK_DIALECT_VERSION
*/


/*
// Allows to exclude the enum definitions. You should usually not need this. Can be usefull
// e.g. when fastMavlink is used together with pymavlink-mavgen, such as in tests.
#define FASTMAVLINK_EXCLUDE_ENUMS
*/

#endif // FASTMAVLINK_CONFIG_H



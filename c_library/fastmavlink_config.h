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
  #define FASTMAVLINK_HEARTBEAT_MAVLINK_VERSION  3 // you should not modify this
#endif


/*
#define FASTMAVLINK_MESSAGE_CRCS
*/

/*
#define FASTMAVLINK_SERIAL_WRITE_CHAR
*/

/*
FASTMAVLINK_DIALECT_VERSION
*/

/*
#define FASTMAVLINK_EXCLUDE_ENUMS
*/

#endif // FASTMAVLINK_CONFIG_H



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
// Allows to exclude the enum definitions. You should usually not need this. Can be usefull
// e.g. when fastMavlink is used together with pymavlink-mavgen, such as in tests.
#define FASTMAVLINK_EXCLUDE_ENUMS
*/

#endif // FASTMAVLINK_CONFIG_H



//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_ICAROUS_H
#define FASTMAVLINK_ICAROUS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Wed Oct 04 2023"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "icarous_msg_entries.h"

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MSG_ENTRIES
#endif


//------------------------------
//-- FastMavlink lib
//------------------------------

#include "../lib/fastmavlink.h"

#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../lib/fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_TEST_EXCLUDE_ENUMS

#ifndef FASTMAVLINK_HAS_ENUM_ICAROUS_TRACK_BAND_TYPES
#define FASTMAVLINK_HAS_ENUM_ICAROUS_TRACK_BAND_TYPES
typedef enum ICAROUS_TRACK_BAND_TYPES {
    ICAROUS_TRACK_BAND_TYPE_NONE = 0,  //  
    ICAROUS_TRACK_BAND_TYPE_NEAR = 1,  //  
    ICAROUS_TRACK_BAND_TYPE_RECOVERY = 2,  //  
    ICAROUS_TRACK_BAND_TYPES_ENUM_END = 3,  // end marker
} ICAROUS_TRACK_BAND_TYPES;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_ICAROUS_FMS_STATE
#define FASTMAVLINK_HAS_ENUM_ICAROUS_FMS_STATE
typedef enum ICAROUS_FMS_STATE {
    ICAROUS_FMS_STATE_IDLE = 0,  //  
    ICAROUS_FMS_STATE_TAKEOFF = 1,  //  
    ICAROUS_FMS_STATE_CLIMB = 2,  //  
    ICAROUS_FMS_STATE_CRUISE = 3,  //  
    ICAROUS_FMS_STATE_APPROACH = 4,  //  
    ICAROUS_FMS_STATE_LAND = 5,  //  
    ICAROUS_FMS_STATE_ENUM_END = 6,  // end marker
} ICAROUS_FMS_STATE;
#endif

#endif // FASTMAVLINK_DO_NOT_INCLUDE_ENUMS


//------------------------------
//-- Message definitions
//------------------------------

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #endif
#endif

#include "./mavlink_msg_icarous_heartbeat.h"
#include "./mavlink_msg_icarous_kinematic_bands.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------




#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_ICAROUS_H

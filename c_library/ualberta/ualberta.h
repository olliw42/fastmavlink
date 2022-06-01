//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_UALBERTA_H
#define FASTMAVLINK_UALBERTA_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Wed Jun 01 2022"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "ualberta_msg_entries.h"

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

#ifndef FASTMAVLINK_HAS_ENUM_UALBERTA_AUTOPILOT_MODE
#define FASTMAVLINK_HAS_ENUM_UALBERTA_AUTOPILOT_MODE
typedef enum UALBERTA_AUTOPILOT_MODE {
    MODE_MANUAL_DIRECT = 1,  // Raw input pulse widts sent to output 
    MODE_MANUAL_SCALED = 2,  // Inputs are normalized using calibration, the converted back to raw pulse widths for output 
    MODE_AUTO_PID_ATT = 3,  //  dfsdfs 
    MODE_AUTO_PID_VEL = 4,  //  dfsfds 
    MODE_AUTO_PID_POS = 5,  //  dfsdfsdfs 
    UALBERTA_AUTOPILOT_MODE_ENUM_END = 6,  // end marker
} UALBERTA_AUTOPILOT_MODE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UALBERTA_NAV_MODE
#define FASTMAVLINK_HAS_ENUM_UALBERTA_NAV_MODE
typedef enum UALBERTA_NAV_MODE {
    NAV_AHRS_INIT = 1,  //  
    NAV_AHRS = 2,  // AHRS mode 
    NAV_INS_GPS_INIT = 3,  // INS/GPS initialization mode 
    NAV_INS_GPS = 4,  // INS/GPS mode 
    UALBERTA_NAV_MODE_ENUM_END = 5,  // end marker
} UALBERTA_NAV_MODE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_UALBERTA_PILOT_MODE
#define FASTMAVLINK_HAS_ENUM_UALBERTA_PILOT_MODE
typedef enum UALBERTA_PILOT_MODE {
    PILOT_MANUAL = 1,  //  sdf 
    PILOT_AUTO = 2,  //  dfs 
    PILOT_ROTO = 3,  //  Rotomotion mode  
    UALBERTA_PILOT_MODE_ENUM_END = 4,  // end marker
} UALBERTA_PILOT_MODE;
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

#include "./mavlink_msg_nav_filter_bias.h"
#include "./mavlink_msg_radio_calibration.h"
#include "./mavlink_msg_ualberta_sys_status.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------

#include "../common/common.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_UALBERTA_H

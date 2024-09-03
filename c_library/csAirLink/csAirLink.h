//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_CSAIRLINK_H
#define FASTMAVLINK_CSAIRLINK_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Tue Sep 03 2024"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  3  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "csAirLink_msg_entries.h"

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

#ifndef FASTMAVLINK_HAS_ENUM_AIRLINK_AUTH_RESPONSE_TYPE
#define FASTMAVLINK_HAS_ENUM_AIRLINK_AUTH_RESPONSE_TYPE
typedef enum AIRLINK_AUTH_RESPONSE_TYPE {
    AIRLINK_ERROR_LOGIN_OR_PASS = 0,  // Login or password error 
    AIRLINK_AUTH_OK = 1,  // Auth successful 
    AIRLINK_AUTH_RESPONSE_TYPE_ENUM_END = 2,  // end marker
} AIRLINK_AUTH_RESPONSE_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE
#define FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE
typedef enum AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE {
    AIRLINK_HPR_PARTNER_NOT_READY = 0,  //  
    AIRLINK_HPR_PARTNER_READY = 1,  //  
    AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE_ENUM_END = 2,  // end marker
} AIRLINK_EYE_GS_HOLE_PUSH_RESP_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_IP_VERSION
#define FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_IP_VERSION
typedef enum AIRLINK_EYE_IP_VERSION {
    AIRLINK_IP_V4 = 0,  //  
    AIRLINK_IP_V6 = 1,  //  
    AIRLINK_EYE_IP_VERSION_ENUM_END = 2,  // end marker
} AIRLINK_EYE_IP_VERSION;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_HOLE_PUSH_TYPE
#define FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_HOLE_PUSH_TYPE
typedef enum AIRLINK_EYE_HOLE_PUSH_TYPE {
    AIRLINK_HP_NOT_PENETRATED = 0,  //  
    AIRLINK_HP_BROKEN = 1,  //  
    AIRLINK_EYE_HOLE_PUSH_TYPE_ENUM_END = 2,  // end marker
} AIRLINK_EYE_HOLE_PUSH_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_TURN_INIT_TYPE
#define FASTMAVLINK_HAS_ENUM_AIRLINK_EYE_TURN_INIT_TYPE
typedef enum AIRLINK_EYE_TURN_INIT_TYPE {
    AIRLINK_TURN_INIT_START = 0,  //  
    AIRLINK_TURN_INIT_OK = 1,  //  
    AIRLINK_TURN_INIT_BAD = 2,  //  
    AIRLINK_EYE_TURN_INIT_TYPE_ENUM_END = 3,  // end marker
} AIRLINK_EYE_TURN_INIT_TYPE;
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

#include "./mavlink_msg_airlink_auth.h"
#include "./mavlink_msg_airlink_auth_response.h"
#include "./mavlink_msg_airlink_eye_gs_hole_push_request.h"
#include "./mavlink_msg_airlink_eye_gs_hole_push_response.h"
#include "./mavlink_msg_airlink_eye_hp.h"
#include "./mavlink_msg_airlink_eye_turn_init.h"

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

#endif // FASTMAVLINK_CSAIRLINK_H

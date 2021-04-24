//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_DEVELOPMENT_H
#define FASTMAVLINK_DEVELOPMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Sat Apr 24 2021"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- crc, min length, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "development_msg_entries.h"

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MSG_ENTRIES
#endif


#include "../lib/fastmavlink.h"
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../lib/fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_TEST_EXCLUDE_ENUMS

#ifndef FASTMAVLINK_HAS_ENUM_WIFI_NETWORK_SECURITY
#define FASTMAVLINK_HAS_ENUM_WIFI_NETWORK_SECURITY
typedef enum WIFI_NETWORK_SECURITY {
    WIFI_NETWORK_SECURITY_UNDEFINED = 0,  // Undefined or unknown security protocol. 
    WIFI_NETWORK_SECURITY_OPEN = 1,  // Open network, no security. 
    WIFI_NETWORK_SECURITY_WEP = 2,  // WEP. 
    WIFI_NETWORK_SECURITY_WPA1 = 3,  // WPA1. 
    WIFI_NETWORK_SECURITY_WPA2 = 4,  // WPA2. 
    WIFI_NETWORK_SECURITY_WPA3 = 5,  // WPA3. 
    WIFI_NETWORK_SECURITY_ENUM_END = 6,  // end marker
} WIFI_NETWORK_SECURITY;
#endif

#endif // FASTMAVLINK_DO_NOT_INCLUDE_ENUMS


//------------------------------
//-- Message definitions
//------------------------------

#include "./mavlink_msg_mission_checksum.h"
#include "./mavlink_msg_wifi_network_info.h"


//------------------------------
//-- Dialect includes
//------------------------------

#include "../standard/standard.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_DEVELOPMENT_H

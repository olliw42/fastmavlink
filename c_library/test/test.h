//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_H
#define FASTMAVLINK_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Sun Mar 07 2021"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  3  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- crc, min length, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "test_msg_entries.h"

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MSG_ENTRIES
#endif


#include "../fastmavlink.h"
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_TEST_EXCLUDE_ENUMS



#endif // FASTMAVLINK_DO_NOT_INCLUDE_ENUMS


//------------------------------
//-- Message definitions
//------------------------------

#include "./mavlink_msg_test_types.h"


//------------------------------
//-- Dialect includes
//------------------------------




#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_TEST_H

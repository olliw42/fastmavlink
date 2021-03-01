//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_ALL_H
#define FASTMAVLINK_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Tue Mar 02 2021"
#endif

#ifdef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  3  // version as specified in xml file
#endif


//------------------------------
//-- Message credentials
//-- crc, min length, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "all_msg_entries.h"

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




//------------------------------
//-- Message definitions
//------------------------------




//------------------------------
//-- Dialect includes
//------------------------------

#include "../ardupilotmega/ardupilotmega.h"
#include "../common/common.h"
#include "../icarous/icarous.h"
#include "../minimal/minimal.h"
#include "../python_array_test/python_array_test.h"
#include "../standard/standard.h"
#include "../test/test.h"
#include "../ualberta/ualberta.h"
#include "../uAvionix/uAvionix.h"
#include "../storm32/storm32.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_ALL_H

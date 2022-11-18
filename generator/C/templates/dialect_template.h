//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_${basename_upper}_H
#define FASTMAVLINK_${basename_upper}_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "${parse_time}"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  ${version}  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "${basename}_msg_entries.h"

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

${{enums_merged:#ifndef FASTMAVLINK_HAS_ENUM_${name}
#define FASTMAVLINK_HAS_ENUM_${name}
typedef enum ${name} {
${{entries:    ${name} = ${value},  // ${description} ${{params:| ${description} }}
}}
} ${name};
#endif


}}

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

${{messages:#include "./mavlink_msg_${name_lower}.h"
}}

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------

${{include_list:#include "../${name}/${name}.h"
}}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_${basename_upper}_H

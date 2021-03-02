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
//-- crc, min length, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "${basename}_msg_entries.h"

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

${{enums_merged:#ifndef FASTMAVLINK_HAS_ENUM_${name}
#define FASTMAVLINK_HAS_ENUM_${name}
typedef enum ${name} {
${{entry:    ${name} = ${value},  // ${description} ${{params:| ${description} }}
}}
} ${name};
#endif


}}


//------------------------------
//-- Message definitions
//------------------------------

${{messages:#include "./mavlink_msg_${name_lower}.h"
}}


//------------------------------
//-- Dialect includes
//------------------------------

${{include_list:#include "../${name}/${name}.h"
}}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_${basename_upper}_H

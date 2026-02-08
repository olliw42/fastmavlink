//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_STANDARD_H
#define FASTMAVLINK_STANDARD_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Sun Feb 08 2026"
#endif

#ifndef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  0  // this is the version specified in the dialect xml file
#endif


//------------------------------
//-- Message credentials
//-- The values of msg_entry_t for all messages in the dialect.
//-- msgid, extra crc, max length, flag, target sysid offset, target compid offset
//------------------------------

#include "standard_msg_entries.h"

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

#ifndef FASTMAVLINK_HAS_ENUM_MAV_BOOL
#define FASTMAVLINK_HAS_ENUM_MAV_BOOL
typedef enum MAV_BOOL {
    MAV_BOOL_FALSE = 0,  // False. 
    MAV_BOOL_TRUE = 1,  // True. 
    MAV_BOOL_ENUM_END = 2,  // end marker
} MAV_BOOL;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_PROTOCOL_CAPABILITY
#define FASTMAVLINK_HAS_ENUM_MAV_PROTOCOL_CAPABILITY
typedef enum MAV_PROTOCOL_CAPABILITY {
    MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,  // Autopilot supports the MISSION_ITEM float message type.          Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_ITEM_INT instead.         
    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,  // Autopilot supports the new param float message type. 
    MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,  // Autopilot supports MISSION_ITEM_INT scaled integer message type.          Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).         
    MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,  // Autopilot supports COMMAND_INT scaled integer message type. 
    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE = 16,  // Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST should be set if the parameter protocol is supported.         
    MAV_PROTOCOL_CAPABILITY_FTP = 32,  // Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html. 
    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,  // Autopilot supports commanding attitude offboard. 
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,  // Autopilot supports commanding position and velocity targets in local NED frame. 
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,  // Autopilot supports commanding position and velocity targets in global scaled integers. 
    MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,  // Autopilot supports terrain protocol / data handling. 
    MAV_PROTOCOL_CAPABILITY_RESERVED3 = 1024,  // Reserved for future use. 
    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,  // Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination). 
    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,  // Autopilot supports onboard compass calibration. 
    MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,  // Autopilot supports MAVLink version 2. 
    MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,  // Autopilot supports mission fence protocol. 
    MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,  // Autopilot supports mission rally point protocol. 
    MAV_PROTOCOL_CAPABILITY_RESERVED2 = 65536,  // Reserved for future use. 
    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST = 131072,  // Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if the parameter protocol is supported.         
    MAV_PROTOCOL_CAPABILITY_COMPONENT_IMPLEMENTS_GIMBAL_MANAGER = 262144,  // This component implements/is a gimbal manager. This means the GIMBAL_MANAGER_INFORMATION, and other messages can be requested.         
    MAV_PROTOCOL_CAPABILITY_COMPONENT_ACCEPTS_GCS_CONTROL = 524288,  // Component supports locking control to a particular GCS independent of its system (via MAV_CMD_REQUEST_OPERATOR_CONTROL). 
    MAV_PROTOCOL_CAPABILITY_GRIPPER = 1048576,  // Autopilot has a connected gripper. MAVLink Grippers would set MAV_TYPE_GRIPPER instead. 
    MAV_PROTOCOL_CAPABILITY_ENUM_END = 1048577,  // end marker
} MAV_PROTOCOL_CAPABILITY;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_FIRMWARE_VERSION_TYPE
#define FASTMAVLINK_HAS_ENUM_FIRMWARE_VERSION_TYPE
typedef enum FIRMWARE_VERSION_TYPE {
    FIRMWARE_VERSION_TYPE_DEV = 0,  // development release 
    FIRMWARE_VERSION_TYPE_ALPHA = 64,  // alpha release 
    FIRMWARE_VERSION_TYPE_BETA = 128,  // beta release 
    FIRMWARE_VERSION_TYPE_RC = 192,  // release candidate 
    FIRMWARE_VERSION_TYPE_OFFICIAL = 255,  // official stable release 
    FIRMWARE_VERSION_TYPE_ENUM_END = 256,  // end marker
} FIRMWARE_VERSION_TYPE;
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

#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_autopilot_version.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------

#include "../minimal/minimal.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_STANDARD_H

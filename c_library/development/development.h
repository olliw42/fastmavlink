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

#include "development_msg_entries.h"

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


#ifndef FASTMAVLINK_HAS_ENUM_AIRSPEED_SENSOR_TYPE
#define FASTMAVLINK_HAS_ENUM_AIRSPEED_SENSOR_TYPE
typedef enum AIRSPEED_SENSOR_TYPE {
    AIRSPEED_SENSOR_TYPE_UNKNOWN = 0,  // Airspeed sensor type unknown/not supplied. 
    AIRSPEED_SENSOR_TYPE_DIFFERENTIAL = 1,  // Differential airspeed sensor 
    AIRSPEED_SENSOR_TYPE_MASS_FLOW = 2,  // Mass-flow airspeed sensor. 
    AIRSPEED_SENSOR_TYPE_WINDVANE = 3,  // Windvane airspeed sensor. 
    AIRSPEED_SENSOR_TYPE_SYNTHETIC = 4,  // Synthetic/calculated airspeed. 
    AIRSPEED_SENSOR_TYPE_ENUM_END = 5,  // end marker
} AIRSPEED_SENSOR_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_PARAM_TRANSACTION_TRANSPORT
#define FASTMAVLINK_HAS_ENUM_PARAM_TRANSACTION_TRANSPORT
typedef enum PARAM_TRANSACTION_TRANSPORT {
    PARAM_TRANSACTION_TRANSPORT_PARAM = 0,  // Transaction over param transport. 
    PARAM_TRANSACTION_TRANSPORT_PARAM_EXT = 1,  // Transaction over param_ext transport. 
    PARAM_TRANSACTION_TRANSPORT_ENUM_END = 2,  // end marker
} PARAM_TRANSACTION_TRANSPORT;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_PARAM_TRANSACTION_ACTION
#define FASTMAVLINK_HAS_ENUM_PARAM_TRANSACTION_ACTION
typedef enum PARAM_TRANSACTION_ACTION {
    PARAM_TRANSACTION_ACTION_START = 0,  // Commit the current parameter transaction. 
    PARAM_TRANSACTION_ACTION_COMMIT = 1,  // Commit the current parameter transaction. 
    PARAM_TRANSACTION_ACTION_CANCEL = 2,  // Cancel the current parameter transaction. 
    PARAM_TRANSACTION_ACTION_ENUM_END = 3,  // end marker
} PARAM_TRANSACTION_ACTION;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STANDARD_MODE
#define FASTMAVLINK_HAS_ENUM_MAV_STANDARD_MODE
typedef enum MAV_STANDARD_MODE {
    MAV_STANDARD_MODE_NON_STANDARD = 0,  // Non standard mode.          This may be used when reporting the mode if the current flight mode is not a standard mode.         
    MAV_STANDARD_MODE_POSITION_HOLD = 1,  // Position mode (manual).          Position-controlled and stabilized manual mode.          When sticks are released vehicles return to their level-flight orientation and hold both position and altitude against wind and external forces.          This mode can only be set by vehicles that can hold a fixed position.          Multicopter (MC) vehicles actively brake and hold both position and altitude against wind and external forces.          Hybrid MC/FW ("VTOL") vehicles first transition to multicopter mode (if needed) but otherwise behave in the same way as MC vehicles.          Fixed-wing (FW) vehicles must not support this mode.          Other vehicle types must not support this mode (this may be revisited through the PR process).         
    MAV_STANDARD_MODE_ORBIT = 2,  // Orbit (manual).          Position-controlled and stabilized manual mode.          The vehicle circles around a fixed setpoint in the horizontal plane at a particular radius, altitude, and direction.          Flight stacks may further allow manual control over the setpoint position, radius, direction, speed, and/or altitude of the circle, but this is not mandated.          Flight stacks may support the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) for changing the orbit parameters.          MC and FW vehicles may support this mode.          Hybrid MC/FW ("VTOL") vehicles may support this mode in MC/FW or both modes; if the mode is not supported by the current configuration the vehicle should transition to the supported configuration.          Other vehicle types must not support this mode (this may be revisited through the PR process).         
    MAV_STANDARD_MODE_CRUISE = 3,  // Cruise mode (manual).          Position-controlled and stabilized manual mode.          When sticks are released vehicles return to their level-flight orientation and hold their original track against wind and external forces.          Fixed-wing (FW) vehicles level orientation and maintain current track and altitude against wind and external forces.          Hybrid MC/FW ("VTOL") vehicles first transition to FW mode (if needed) but otherwise behave in the same way as MC vehicles.          Multicopter (MC) vehicles must not support this mode.          Other vehicle types must not support this mode (this may be revisited through the PR process).         
    MAV_STANDARD_MODE_ALTITUDE_HOLD = 4,  // Altitude hold (manual).          Altitude-controlled and stabilized manual mode.          When sticks are released vehicles return to their level-flight orientation and hold their altitude.          MC vehicles continue with existing momentum and may move with wind (or other external forces).          FW vehicles continue with current heading, but may be moved off-track by wind.          Hybrid MC/FW ("VTOL") vehicles behave according to their current configuration/mode (FW or MC).          Other vehicle types must not support this mode (this may be revisited through the PR process).         
    MAV_STANDARD_MODE_RETURN_HOME = 5,  // Return home mode (auto).          Automatic mode that returns vehicle to home via a safe flight path.          It may also automatically land the vehicle (i.e. RTL).          The precise flight path and landing behaviour depend on vehicle configuration and type.         
    MAV_STANDARD_MODE_SAFE_RECOVERY = 6,  // Safe recovery mode (auto).          Automatic mode that takes vehicle to a predefined safe location via a safe flight path (rally point or mission defined landing) .          It may also automatically land the vehicle.          The precise return location, flight path, and landing behaviour depend on vehicle configuration and type.         
    MAV_STANDARD_MODE_MISSION = 7,  // Mission mode (automatic).          Automatic mode that executes MAVLink missions.          Missions are executed from the current waypoint as soon as the mode is enabled.         
    MAV_STANDARD_MODE_LAND = 8,  // Land mode (auto).          Automatic mode that lands the vehicle at the current location.          The precise landing behaviour depends on vehicle configuration and type.         
    MAV_STANDARD_MODE_TAKEOFF = 9,  // Takeoff mode (auto).          Automatic takeoff mode.          The precise takeoff behaviour depends on vehicle configuration and type.         
    MAV_STANDARD_MODE_ENUM_END = 10,  // end marker
} MAV_STANDARD_MODE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_CMD
#define FASTMAVLINK_HAS_ENUM_MAV_CMD
typedef enum MAV_CMD {
    MAV_CMD_DO_FIGURE_EIGHT = 35,  // Fly a figure eight path as defined by the parameters.          Set parameters to NaN/INT32_MAX (as appropriate) to use system-default values.          The command is intended for fixed wing vehicles (and VTOL hybrids flying in fixed-wing mode), allowing POI tracking for gimbals that don't support infinite rotation.          This command only defines the flight path. Speed should be set independently (use e.g. MAV_CMD_DO_CHANGE_SPEED).          Yaw and other degrees of freedom are not specified, and will be flight-stack specific (on vehicles where they can be controlled independent of the heading).         | Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.        NaN: The radius will be set to 2.5 times the minor radius and direction is clockwise.        Must be greater or equal to two times the minor radius for feasible values. | Minor axis radius of the figure eight. Defines the radius of the two circles that make up the figure. Negative value has no effect.        NaN: The radius will be set to the default loiter radius. | Reserved (default:NaN) | Orientation of the figure eight major axis with respect to true north (range: [-pi,pi]). NaN: use default orientation aligned to true north. | Center point latitude/X coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.        INT32_MAX or NaN: Use current vehicle position, or current center if already loitering. | Center point longitude/Y coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.        INT32_MAX or NaN: Use current vehicle position, or current center if already loitering. | Center point altitude MSL/Z coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.        INT32_MAX or NaN: Use current vehicle altitude.
    MAV_CMD_PARAM_TRANSACTION = 900,  // Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. | Action to be performed (start, commit, cancel, etc.) | Possible transport layers to set and get parameters via mavlink during a parameter transaction. | Identifier for a specific transaction. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_FENCE_BREACH_ACTION = 5010,  // Sets the action on geofence breach.          If sent using the command protocol this sets the system-default geofence action.          As part of a mission protocol plan it sets the fence action for the next complete geofence definition *after* the command.          Note: A fence action defined in a plan will override the default system setting (even if the system-default is `FENCE_ACTION_NONE`).          Note: Every geofence in a plan can have its own action; if no fence action is defined for a particular fence the system-default will be used.          Note: The flight stack should reject a plan or command that uses a geofence action that it does not support and send a STATUSTEXT with the reason.         | Fence action on breach. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_ENUM_END = 5011,  // end marker
} MAV_CMD;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_CMD
#define FASTMAVLINK_HAS_ENUM_MAV_CMD
typedef enum MAV_CMD {
    MAV_CMD_DO_UPGRADE = 247,  // Request a target system to start an upgrade of one (or all) of its components.          For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller.          The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation.          Command protocol information: https://mavlink.io/en/services/command.html. | Component id of the component to be upgraded. If set to 0, all components should be upgraded. | 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed. | Reserved | Reserved | Reserved | Reserved | WIP: upgrade progress report rate (can be used for more granular control).
    MAV_CMD_GROUP_START = 301,  // Define start of a group of mission items. When control reaches this command a GROUP_START message is emitted.          The end of a group is marked using MAV_CMD_GROUP_END with the same group id.          Group ids are expected, but not required, to iterate sequentially.          Groups can be nested. | Mission-unique group id.          Group id is limited because only 24 bit integer can be stored in 32 bit float. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_GROUP_END = 302,  // Define end of a group of mission items. When control reaches this command a GROUP_END message is emitted.          The start of the group is marked is marked using MAV_CMD_GROUP_START with the same group id.          Group ids are expected, but not required, to iterate sequentially.          Groups can be nested. | Mission-unique group id.          Group id is limited because only 24 bit integer can be stored in 32 bit float. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DO_SET_STANDARD_MODE = 262,  // Enable the specified standard MAVLink mode.          If the mode is not supported the vehicle should ACK with MAV_RESULT_FAILED.         | The mode to set. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_ENUM_END = 263,  // end marker
} MAV_CMD;
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

#include "./mavlink_msg_param_ack_transaction.h"
#include "./mavlink_msg_mission_changed.h"
#include "./mavlink_msg_mission_checksum.h"
#include "./mavlink_msg_airspeed.h"
#include "./mavlink_msg_wifi_network_info.h"
#include "./mavlink_msg_figure_eight_execution_status.h"
#include "./mavlink_msg_component_information_basic.h"
#include "./mavlink_msg_group_start.h"
#include "./mavlink_msg_group_end.h"
#include "./mavlink_msg_available_modes.h"
#include "./mavlink_msg_current_mode.h"

#ifdef FASTMAVLINK_IGNORE_WADDRESSOFPACKEDMEMBER
  #if defined __GNUC__ && __GNUC__ >= 9
    #pragma GCC diagnostic pop
  #endif
#endif


//------------------------------
//-- Dialect includes
//------------------------------

#include "../standard/standard.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_DEVELOPMENT_H

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
#define FASTMAVLINK_BUILD_DATE  "Fri Oct 01 2021"
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


#ifndef FASTMAVLINK_HAS_ENUM_MAV_CMD
#define FASTMAVLINK_HAS_ENUM_MAV_CMD
typedef enum MAV_CMD {
    MAV_CMD_PARAM_TRANSACTION = 900,  // Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. | Action to be performed (start, commit, cancel, etc.) | Possible transport layers to set and get parameters via mavlink during a parameter transaction. | Identifier for a specific transaction. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_FENCE_BREACH_ACTION = 5010,  // Sets the action on geofence breach.          If sent using the command protocol this sets the system-default geofence action.          As part of a mission protocol plan it sets the fence action for the next complete geofence definition *after* the command.          Note: A fence action defined in a plan will override the default system setting (even if the system-default is `FENCE_ACTION_NONE`).          Note: Every geofence in a plan can have its own action; if no fence action is defined for a particular fence the system-default will be used.          Note: The flight stack should reject a plan or command that uses a geofence action that it does not support and send a STATUSTEXT with the reason.         | Fence action on breach. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_ENUM_END = 5011,  // end marker
} MAV_CMD;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_CELLULAR_NETWORK_RADIO_TYPE
#define FASTMAVLINK_HAS_ENUM_CELLULAR_NETWORK_RADIO_TYPE
typedef enum CELLULAR_NETWORK_RADIO_TYPE {
    CELLULAR_NETWORK_RADIO_TYPE_NONE = 0,  //  
    CELLULAR_NETWORK_RADIO_TYPE_GSM = 1,  //  
    CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2,  //  
    CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3,  //  
    CELLULAR_NETWORK_RADIO_TYPE_LTE = 4,  //  
    CELLULAR_NETWORK_RADIO_TYPE_ENUM_END = 5,  // end marker
} CELLULAR_NETWORK_RADIO_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_CELLULAR_STATUS_FLAG
#define FASTMAVLINK_HAS_ENUM_CELLULAR_STATUS_FLAG
typedef enum CELLULAR_STATUS_FLAG {
    CELLULAR_STATUS_FLAG_UNKNOWN = 0,  // State unknown or not reportable. 
    CELLULAR_STATUS_FLAG_FAILED = 1,  // Modem is unusable 
    CELLULAR_STATUS_FLAG_INITIALIZING = 2,  // Modem is being initialized 
    CELLULAR_STATUS_FLAG_LOCKED = 3,  // Modem is locked 
    CELLULAR_STATUS_FLAG_DISABLED = 4,  // Modem is not enabled and is powered down 
    CELLULAR_STATUS_FLAG_DISABLING = 5,  // Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state 
    CELLULAR_STATUS_FLAG_ENABLING = 6,  // Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state 
    CELLULAR_STATUS_FLAG_ENABLED = 7,  // Modem is enabled and powered on but not registered with a network provider and not available for data connections 
    CELLULAR_STATUS_FLAG_SEARCHING = 8,  // Modem is searching for a network provider to register 
    CELLULAR_STATUS_FLAG_REGISTERED = 9,  // Modem is registered with a network provider, and data connections and messaging may be available for use 
    CELLULAR_STATUS_FLAG_DISCONNECTING = 10,  // Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated 
    CELLULAR_STATUS_FLAG_CONNECTING = 11,  // Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered 
    CELLULAR_STATUS_FLAG_CONNECTED = 12,  // One or more packet data bearers is active and connected 
    CELLULAR_STATUS_FLAG_ENUM_END = 13,  // end marker
} CELLULAR_STATUS_FLAG;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_CELLULAR_NETWORK_FAILED_REASON
#define FASTMAVLINK_HAS_ENUM_CELLULAR_NETWORK_FAILED_REASON
typedef enum CELLULAR_NETWORK_FAILED_REASON {
    CELLULAR_NETWORK_FAILED_REASON_NONE = 0,  // No error 
    CELLULAR_NETWORK_FAILED_REASON_UNKNOWN = 1,  // Error state is unknown 
    CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING = 2,  // SIM is required for the modem but missing 
    CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR = 3,  // SIM is available, but not usuable for connection 
    CELLULAR_NETWORK_FAILED_REASON_ENUM_END = 4,  // end marker
} CELLULAR_NETWORK_FAILED_REASON;
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
#include "./mavlink_msg_cellular_status.h"

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

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_MSG_ENTRIES_H
#define FASTMAVLINK_MSG_ENTRIES_H


//------------------------------
//-- The values of msg_entry_t for all messages in the dialect.
//------------------------------
 
#define FASTMAVLINK_MSG_ENTRY_ICAROUS_HEARTBEAT  {42000, 227, 1, 0, 0, 0}
#define FASTMAVLINK_MSG_ENTRY_ICAROUS_KINEMATIC_BANDS  {42001, 239, 46, 0, 0, 0}


/*------------------------------
 * If only relatively few MAVLink messages are used, efficiency can
 * be much improved, both memory and computational time wise, by
 * commenting out below all those message entries which are not used,
 * and to write in the user's code:
 *
 * #define FASTMAVLINK_MESSAGE_CRCS  FASTMAVLINK_MESSAGE_ENTRIES 
 *
 * Alternatively, the above defines can be used to define one's own
 * FASTMAVLINK_MESSAGE_CRCS. It is then MOST important to keep the sequence
 * in order since otherwise the default binary search will fail. E.g.:
 * 
 * #include "pathtofastmavlink/thedialect/fmav_msg_entries.h"
 * #define FASTMAVLINK_MESSAGE_CRCS {\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_READ,\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_REQUEST_LIST,\
 *     FASTMAVLINK_MSG_ENTRY_PARAM_SET,\
 *     FASTMAVLINK_MSG_ENTRY_COMMAND_LONG,\
 *     FASTMAVLINK_MSG_ENTRY_AUTOPILOT_VERSION_REQUEST }
 ------------------------------*/
 
#define FASTMAVLINK_MSG_ENTRIES {\
  FASTMAVLINK_MSG_ENTRY_ICAROUS_HEARTBEAT,\
  FASTMAVLINK_MSG_ENTRY_ICAROUS_KINEMATIC_BANDS\
}


#endif // FASTMAVLINK_MSG_ENTRIES_H

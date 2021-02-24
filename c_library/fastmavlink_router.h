//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
//
// requires that these are defined intehoutside:
//   FASTMAVLINK_ROUTER_LINKS_MAX
//   FASTMAVLINK_ROUTER_COMPONENTS_MAX
//
// void fmav_router_reset(void)
// void fmav_router_handle_message_by_id(
//     uint8_t link_of_msg,
//     uint8_t msgid,
//     uint8_t sysid, uint8_t compid,
//     uint8_t target_sysid, uint8_t target_compid
//     )
// void fmav_router_handle_message(uint8_t link_of_msg, fmav_result_t* result)
// void fmav_router_handle_message_by_msg(uint8_t link_of_msg, fmav_message_t* msg)
// uint8_t fmav_router_send_to_link(uint8_t link)
// void fmav_router_add_ourself(uint8_t sysid, uint8_t compid)
// void fmav_router_clearout_link(uint8_t link)
//------------------------------

#pragma once
#ifndef FASTMAVLINK_ROUTER_H
#define FASTMAVLINK_ROUTER_H

#ifdef __cplusplus
extern "C" {
#endif


#include "fastmavlink_config.h"
#include "fastmavlink_types.h"


//------------------------------
//-- Structures & fields
//------------------------------

typedef struct _fmav_router_component_item {
    uint8_t valid;
    uint8_t sysid;
    uint8_t compid;
    uint8_t link; // 0 is ourself, 1 = COMM0, 2 = COMM1, ...
} fmav_router_component_item;


FASTMAVLINK_RAM_SECTION fmav_router_component_item _fmav_router_component_list[FASTMAVLINK_ROUTER_COMPONENTS_MAX];

FASTMAVLINK_RAM_SECTION uint8_t _fmav_router_send_to_link[FASTMAVLINK_ROUTER_LINKS_MAX];


//------------------------------
//-- Helpers
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_accept(uint8_t target_link, uint8_t target_sysid, uint8_t target_compid)
{
    // go through all components on the link and see if the one we are targeting at is there
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue;

        if (_fmav_router_component_list[i].link != target_link) continue; //not our link

        if (target_sysid == 0) { //target link has seen at least one component, and target_sysid is broadcast, so ok
          return 1;
        }

        if (_fmav_router_component_list[i].sysid != target_sysid) continue; //not our system

        if (target_compid == 0) { //target_sysid is on the link, and target_compid is broadcast
          return 1;
        }

        if (_fmav_router_component_list[i].compid == target_compid) { //target_sysid and target_compid is on the link
          return 1;
        }
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_find_component(uint8_t sysid, uint8_t compid)
{
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (_fmav_router_component_list[i].valid &&
            _fmav_router_component_list[i].sysid == sysid &&
            _fmav_router_component_list[i].compid == compid) {
            return 1;
        }
    }
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_add_component(uint8_t link, uint8_t sysid, uint8_t compid)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return false;

    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (_fmav_router_component_list[i].valid) continue; //already occupied
        _fmav_router_component_list[i].valid = 1;
        _fmav_router_component_list[i].link = link;
        _fmav_router_component_list[i].sysid = sysid;
        _fmav_router_component_list[i].compid = compid;
        return 1;
    }
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_find_or_add_component(uint8_t link, uint8_t sysid, uint8_t compid)
{
    if (fmav_router_find_component(sysid, compid)) return 1;
    // not found, so try to add
    if (fmav_router_add_component(link, sysid, compid)) return 1;
    // could not be added
    return 0;
}


//------------------------------
//-- API
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_reset(void)
{
    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        _fmav_router_component_list[i].valid = 0;
        _fmav_router_component_list[i].link = 0;
        _fmav_router_component_list[i].sysid = 0;
        _fmav_router_component_list[i].compid = 0;
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message_by_id(
    uint8_t link_of_msg,
    uint8_t msgid,
    uint8_t sysid, uint8_t compid,
    uint8_t target_sysid, uint8_t target_compid
    )
{
    if (link_of_msg >= FASTMAVLINK_ROUTER_LINKS_MAX) {
        for(uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
            _fmav_router_send_to_link[link] = 0;
        }
        return;
    }

    // keep list of available components by spying heartbeats
    // heartbeats are always send to all links
    if (msgid == FASTMAVLINK_MSG_ID_HEARTBEAT) {
        for(uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
            _fmav_router_send_to_link[link] = 1;
        }
        _fmav_router_send_to_link[link_of_msg] = 0; //origin of msg, don't reflect it back
        fmav_router_find_or_add_component(link_of_msg, sysid, compid);
        return;
    }

    // determine to which links it has to be send
    for (uint8_t link = 0; link < FASTMAVLINK_ROUTER_LINKS_MAX; link++) {
        _fmav_router_send_to_link[link] = 0;

        if (link == link_of_msg) continue; //origin of msg, don't reflect it back

        if (fmav_router_accept(link, target_sysid, target_compid)) {
          _fmav_router_send_to_link[link] = 1;
          continue;
        }
    }
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message(uint8_t link_of_msg, fmav_result_t* result)
{
    fmav_router_handle_message_by_id(
        link_of_msg,
        result->msgid,
        result->sysid,
        result->compid,
        result->target_sysid,
        result->target_compid);
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_handle_message_by_msg(uint8_t link_of_msg, fmav_message_t* msg)
{
    fmav_router_handle_message_by_id(
        link_of_msg,
        msg->msgid,
        msg->sysid,
        msg->compid,
        msg->target_sysid,
        msg->target_compid);
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_router_send_to_link(uint8_t link)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return 0;
    return _fmav_router_send_to_link[link];
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_add_ourself(uint8_t sysid, uint8_t compid)
{
    fmav_router_add_component(0, sysid, compid); //we always add ourself as link 0
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_router_clearout_link(uint8_t link)
{
    if (link >= FASTMAVLINK_ROUTER_LINKS_MAX) return;

    for (uint8_t i = 0; i < FASTMAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!_fmav_router_component_list[i].valid) continue; //empty entry
        if (_fmav_router_component_list[i].link == link) { //clear out
            _fmav_router_component_list[i].valid = 0;
            _fmav_router_component_list[i].link = 0;
            _fmav_router_component_list[i].sysid = 0;
            _fmav_router_component_list[i].compid = 0;
        }
    }
}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_ROUTER_H
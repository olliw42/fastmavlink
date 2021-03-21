//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
//
// uint8_t fmav_param_get_param_union(fmav_param_union_t* param_union, uint16_t index)
// uint8_t fmav_param_set_value(uint16_t index, float value)
// uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
// uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
// uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
//------------------------------

#pragma once
#ifndef FASTMAVLINK_PARAMETERS_H
#define FASTMAVLINK_FUNCTIONS_H

#ifndef FASTMAVLINK_PARAM_NUM
#error For fastmavlink_parameters.h, FASTMAVLINK_PARAM_NUM needs to be defined
#endif

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "../fastmavlink_config.h"
#include "fastmavlink_types.h"


//------------------------------
//-- Defines
//------------------------------

#define FASTMAVLINK_PARAM_NAME_LEN  16


//------------------------------
//-- Support functions
//------------------------------

// retuns >= 0 if found, -1 else
FASTMAVLINK_FUNCTION_DECORATOR int16_t fmav_param_find_index(char* name)
{
    char s[FASTMAVLINK_PARAM_NAME_LEN+1]; // +1 to have room to convert to C string

    memcpy(s, name, FASTMAVLINK_PARAM_NAME_LEN);
    s[FASTMAVLINK_PARAM_NAME_LEN] = '\0'; // make it a terminated C string also in case of 16

    for (uint16_t i = 0; i < FASTMAVLINK_PARAM_NUM; i++) {
        if (!strcmp(s, fmav_param_list[i].name)) return i;
    }

    return -1;
}


//------------------------------
//-- Handler fucntions
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_union(fmav_param_union_t* param_union, uint16_t index)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    param_union->type = fmav_param_list[index].type;

    param_union->p_uint32 = 0; // this fills them all with 0
    switch( param_union->type ){
        case MAV_PARAM_TYPE_UINT8:  param_union->p_uint8  = *((uint8_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT8:   param_union->p_int8   = *((int8_t*)  (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT16: param_union->p_uint16 = *((uint16_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT16:  param_union->p_int16  = *((int16_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT32: param_union->p_uint32 = *((uint32_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT32:  param_union->p_int32  = *((int32_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_REAL32: param_union->p_float  = *((float*)   (fmav_param_list[index].ptr)); return 1;
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_set_value(uint16_t index, float value)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    fmav_param_union_t param_entry;
    param_entry.p_float = value;
    
    switch( fmav_param_list[index].type ){
        case MAV_PARAM_TYPE_UINT8:  *((uint8_t*) (fmav_param_list[index].ptr)) = param_entry.p_uint8;  return 1;
        case MAV_PARAM_TYPE_INT8:   *((int8_t*)  (fmav_param_list[index].ptr)) = param_entry.p_int8;   return 1;
        case MAV_PARAM_TYPE_UINT16: *((uint16_t*)(fmav_param_list[index].ptr)) = param_entry.p_uint16; return 1;
        case MAV_PARAM_TYPE_INT16:  *((int16_t*) (fmav_param_list[index].ptr)) = param_entry.p_int16;  return 1;
        case MAV_PARAM_TYPE_UINT32: *((uint32_t*)(fmav_param_list[index].ptr)) = param_entry.p_uint32; return 1;
        case MAV_PARAM_TYPE_INT32:  *((int32_t*) (fmav_param_list[index].ptr)) = param_entry.p_int32;  return 1;
        case MAV_PARAM_TYPE_REAL32: *((float*)   (fmav_param_list[index].ptr)) = param_entry.p_float;  return 1;
    }
    
    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    fmav_param_union_t value;
    if (!fmav_param_get_param_union(&value, index)) return 0;

    payload->param_value = value.p_float;
    payload->param_count = FASTMAVLINK_PARAM_NUM;
    payload->param_index = index;
    payload->param_type = fmav_param_list[index].type;

    memset(payload->param_id, '\0', FASTMAVLINK_PARAM_NAME_LEN); // strncpy() should do it, play it safe
    strncpy(payload->param_id, fmav_param_list[index].name, FASTMAVLINK_PARAM_NAME_LEN);

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
{
    *index = (payload->param_index < 0) ? fmav_param_find_index(payload->param_id) : payload->param_index;

    if (*index < 0) return 0; // not found

    if (*index >= FASTMAVLINK_PARAM_NUM) return 0;

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
{
    *index = fmav_param_find_index(payload->param_id);

    if (*index < 0) return 0; // not found
    
    if (payload->param_type != fmav_param_list[*index].type) return 0; // ups ...

//    fmav_param_set_value(*index, &(payload->param_value));

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_cmd_do_set_parameter(uint16_t* index, float param1)
{
    *index = param1; 

    if (*index >= FASTMAVLINK_PARAM_NUM) return 0;
    
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_PARAMETERS_H

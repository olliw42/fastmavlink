//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// API:
//
// Get and set:
//   uint8_t fmav_param_get_value(void* value, uint16_t index)
//   uint8_t fmav_param_set_value(uint16_t index, void* value)
//   uint8_t fmav_param_get_param_entry(fmav_param_union_t* param_entry, uint16_t index)
//   uint8_t fmav_param_set_param_entry(uint16_t index, fmav_param_union_t* param_entry)
//
// Handler fucntions:
//   uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
//   uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
//   uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
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


#include "fastmavlink_config.h"
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
//-- Get and set
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_value(void* value, uint16_t index)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    switch( fmav_param_list[index].type ){
        case MAV_PARAM_TYPE_UINT8:  *((uint8_t*) value) = *((uint8_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT8:   *((int8_t*)  value) = *((int8_t*)  (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT16: *((uint16_t*)value) = *((uint16_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT16:  *((int16_t*) value) = *((int16_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT32: *((uint32_t*)value) = *((uint32_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT32:  *((int32_t*) value) = *((int32_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_REAL32: *((float*)   value) = *((float*)   (fmav_param_list[index].ptr)); return 1;
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_set_value(uint16_t index, void* value)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    switch( fmav_param_list[index].type ){
        case MAV_PARAM_TYPE_UINT8:  *((uint8_t*) (fmav_param_list[index].ptr)) = *((uint8_t*) value); return 1;
        case MAV_PARAM_TYPE_INT8:   *((int8_t*)  (fmav_param_list[index].ptr)) = *((int8_t*) value);  return 1;
        case MAV_PARAM_TYPE_UINT16: *((uint16_t*)(fmav_param_list[index].ptr)) = *((uint16_t*)value); return 1;
        case MAV_PARAM_TYPE_INT16:  *((int16_t*) (fmav_param_list[index].ptr)) = *((int16_t*) value); return 1;
        case MAV_PARAM_TYPE_UINT32: *((uint32_t*)(fmav_param_list[index].ptr)) = *((uint32_t*)value); return 1;
        case MAV_PARAM_TYPE_INT32:  *((int32_t*) (fmav_param_list[index].ptr)) = *((int32_t*)value);  return 1;
        case MAV_PARAM_TYPE_REAL32: *((float*)   (fmav_param_list[index].ptr)) = *((float*)value);    return 1;
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_entry(fmav_param_union_t* param_entry, uint16_t index)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    param_entry->type = fmav_param_list[index].type;

    param_entry->p_uint32 = 0; // this fills them all with 0
    switch( param_entry->type ){
        case MAV_PARAM_TYPE_UINT8:  param_entry->p_uint8  = *((uint8_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT8:   param_entry->p_int8   = *((int8_t*)  (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT16: param_entry->p_uint16 = *((uint16_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT16:  param_entry->p_int16  = *((int16_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_UINT32: param_entry->p_uint32 = *((uint32_t*)(fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_INT32:  param_entry->p_int32  = *((int32_t*) (fmav_param_list[index].ptr)); return 1;
        case MAV_PARAM_TYPE_REAL32: param_entry->p_float  = *((float*)   (fmav_param_list[index].ptr)); return 1;
    }

    return 0;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_set_param_entry(uint16_t index, fmav_param_union_t* param_entry)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    param_entry->type = fmav_param_list[index].type;

    param_entry->p_uint32 = 0; // this fills them all with 0
    switch( param_entry->type ){
        case MAV_PARAM_TYPE_UINT8:  *((uint8_t*) (fmav_param_list[index].ptr)) = param_entry->p_uint8;  return 1;
        case MAV_PARAM_TYPE_INT8:   *((int8_t*)  (fmav_param_list[index].ptr)) = param_entry->p_int8;   return 1;
        case MAV_PARAM_TYPE_UINT16: *((uint16_t*)(fmav_param_list[index].ptr)) = param_entry->p_uint16; return 1;
        case MAV_PARAM_TYPE_INT16:  *((int16_t*) (fmav_param_list[index].ptr)) = param_entry->p_int16;  return 1;
        case MAV_PARAM_TYPE_UINT32: *((uint32_t*)(fmav_param_list[index].ptr)) = param_entry->p_uint32; return 1;
        case MAV_PARAM_TYPE_INT32:  *((int32_t*) (fmav_param_list[index].ptr)) = param_entry->p_int32;  return 1;
        case MAV_PARAM_TYPE_REAL32: *((float*)   (fmav_param_list[index].ptr)) = param_entry->p_float;  return 1;
    }

    return 0;
}


//------------------------------
//-- Handler fucntions
//------------------------------

FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_get_param_value(fmav_param_value_t* payload, uint16_t index)
{
    if (index >= FASTMAVLINK_PARAM_NUM) return 0;

    fmav_param_union_t value;
    if (!fmav_param_get_param_entry(&value, index)) return 0;

    payload->param_value = value.p_float;
    payload->param_count = FASTMAVLINK_PARAM_NUM;
    payload->param_index = index;
    payload->param_type = fmav_param_list[index].type;

    memset(payload->param_id, '\0', FASTMAVLINK_PARAM_NAME_LEN);
    uint8_t len = strlen(fmav_param_list[index].name);
    if (len > FASTMAVLINK_PARAM_NAME_LEN) len = FASTMAVLINK_PARAM_NAME_LEN;
    memcpy(payload->param_id, fmav_param_list[index].name, len);

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_request_read(uint16_t* index, fmav_param_request_read_t* payload)
{
    *index = (payload->param_index < 0) ? fmav_param_find_index(payload->param_id) : payload->param_index;

    if (*index < FASTMAVLINK_PARAM_NUM) return 0;

    return 1;
}


FASTMAVLINK_FUNCTION_DECORATOR uint8_t fmav_param_do_param_set(uint16_t* index, fmav_param_set_t* payload)
{
    *index = fmav_param_find_index(payload->param_id);

    if (*index < 0) return 0;
    if (payload->param_type != fmav_param_list[*index].type) return 0; // ups ...

    fmav_param_set_value(*index, &(payload->param_value));

    return 1;
}


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_PARAMETERS_H

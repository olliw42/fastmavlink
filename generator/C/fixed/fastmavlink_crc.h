//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------
// these function are based on the pymavlink-mavgen project
// https://github.com/ArduPilot/pymavlink/tree/master/generator

#pragma once
#ifndef FASTMAVLINK_CRC_H
#define FASTMAVLINK_CRC_H

#include <stdint.h>
#include "../fastmavlink_config.h"


#define X25_INIT_CRC 0xffff


FASTMAVLINK_FUNCTION_DECORATOR void fmav_crc_init(uint16_t* crc)
{
    *crc = X25_INIT_CRC;
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_crc_accumulate(uint16_t* crc, uint8_t data)
{
    uint8_t tmp;

    tmp = data ^ (uint8_t)(*crc & 0xff);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}


FASTMAVLINK_FUNCTION_DECORATOR void fmav_crc_accumulate_buf(uint16_t* crc, const uint8_t* buf, uint16_t len)
{
    while (len--) {
        fmav_crc_accumulate(crc, *buf++);
    }
}


FASTMAVLINK_FUNCTION_DECORATOR uint16_t fmav_crc_calculate(const uint8_t* buf, uint16_t len)
{
    uint16_t crc;

    fmav_crc_init(&crc);
    while (len--) {
        fmav_crc_accumulate(&crc, *buf++);
    }

    return crc;
}

#endif // FASTMAVLINK_CRC_H



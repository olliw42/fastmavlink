//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

//------------------------------
// test_messages
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_RANDOM_GENERATORS_H
#define FASTMAVLINK_TEST_RANDOM_GENERATORS_H


#include <stdlib.h>
#include <stdint.h>


//------------------------------
// random value generators
//------------------------------

int rand_int(int Nmin, int Nmax)
{
    return Nmin + rand() / (RAND_MAX / (Nmax - Nmin + 1) + 1);
}


int rand_uint8_t(void){ return rand_int(0,256-1); }

int rand_uint16_t(void){ return rand_int(0,65536-1); }

int rand_uint24_t(void){ return rand_int(0,16777216-1); }

int rand_uint32_t(void){ return rand_int(0,16777216); }


// random value with preference for 0, -1, MAX

int randp_int(int Nmax, int sign)
{
    int i = rand_int(1, 20);
    //printf("%i\n", i);

    if (i <= 5) return 0; // 1,2,3,4,5 => 0 => 25%

    if (i <= 7) return (sign > 0) ? Nmax : -1; // 6,7 => -1 for ints and UINTMAX for uints => 10%

    if (sign > 0) return rand_int(0, Nmax);

    return rand_int(-(Nmax + 1), Nmax);
}


int randp_uint8_t(void){ return randp_int(256-1, 1); }

int randp_uint16_t(void){ return randp_int(65536-1, 1); }

int randp_uint24_t(void){ return randp_int(16777216-1, 1); }

int randp_uint32_t(void){ return randp_int(65536-1, 1) * 1234; }

int randp_uint64_t(void){ return randp_int(16777216-1, 1) * 1234; }

int randp_int8_t(void){ return randp_int(256-1, 0); }

int randp_int16_t(void){ return randp_int(65536-1, 0); }

int randp_int24_t(void){ return randp_int(16777216-1, 0); }

int randp_int32_t(void){ return randp_int(65536-1, 0) * 1234; }

int randp_int64_t(void){ return randp_int(16777216-1, 0) * 1234; }

char randp_char(void){ return randp_int(256-1, 1); }

float randp_float(void){ return randp_int(65536-1, 1) * 0.0123f; }

double randp_double(void){ return randp_int(65536-1, 1) * 0.0123; }


#endif // FASTMAVLINK_TEST_RANDOM_GENERATORS_H



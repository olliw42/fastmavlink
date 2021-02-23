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
#define FASTMAVLINK_BUILD_DATE  "Tue Feb 23 2021"
#endif

#ifdef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  3  // version as specified in xml file
#endif


//------------------------------
//-- Message credentials
//-- crc, min length, max length, flag, target sysid offset, target compid offset
//------------------------------

#ifndef FASTMAVLINK_MESSAGE_CRCS
#define FASTMAVLINK_MESSAGE_CRCS {\
    {0, 50, 9, 9, 0, 0, 0},\
    {1, 124, 31, 31, 0, 0, 0},\
    {2, 137, 12, 12, 0, 0, 0},\
    {4, 237, 14, 14, 3, 12, 13},\
    {5, 217, 28, 28, 1, 0, 0},\
    {6, 104, 3, 3, 0, 0, 0},\
    {7, 119, 32, 32, 0, 0, 0},\
    {8, 117, 36, 36, 0, 0, 0},\
    {11, 89, 6, 6, 1, 4, 0},\
    {19, 137, 24, 24, 3, 4, 5},\
    {20, 214, 20, 20, 3, 2, 3},\
    {21, 159, 2, 2, 3, 0, 1},\
    {22, 220, 25, 25, 0, 0, 0},\
    {23, 168, 23, 23, 3, 4, 5},\
    {24, 24, 30, 52, 0, 0, 0},\
    {25, 23, 101, 101, 0, 0, 0},\
    {26, 170, 22, 24, 0, 0, 0},\
    {27, 144, 26, 29, 0, 0, 0},\
    {28, 67, 16, 16, 0, 0, 0},\
    {29, 115, 14, 16, 0, 0, 0},\
    {30, 39, 28, 28, 0, 0, 0},\
    {31, 246, 32, 48, 0, 0, 0},\
    {32, 185, 28, 28, 0, 0, 0},\
    {33, 104, 28, 28, 0, 0, 0},\
    {34, 237, 22, 22, 0, 0, 0},\
    {35, 244, 22, 22, 0, 0, 0},\
    {36, 222, 21, 37, 0, 0, 0},\
    {37, 212, 6, 7, 3, 4, 5},\
    {38, 9, 6, 7, 3, 4, 5},\
    {39, 254, 37, 38, 3, 32, 33},\
    {40, 230, 4, 5, 3, 2, 3},\
    {41, 28, 4, 4, 3, 2, 3},\
    {42, 28, 2, 2, 0, 0, 0},\
    {43, 132, 2, 3, 3, 0, 1},\
    {44, 221, 4, 5, 3, 2, 3},\
    {45, 232, 2, 3, 3, 0, 1},\
    {46, 11, 2, 2, 0, 0, 0},\
    {47, 153, 3, 4, 3, 0, 1},\
    {48, 41, 13, 21, 1, 12, 0},\
    {49, 39, 12, 20, 0, 0, 0},\
    {50, 78, 37, 37, 3, 18, 19},\
    {51, 196, 4, 5, 3, 2, 3},\
    {52, 132, 7, 7, 0, 0, 0},\
    {54, 15, 27, 27, 3, 24, 25},\
    {55, 3, 25, 25, 0, 0, 0},\
    {61, 167, 72, 72, 0, 0, 0},\
    {62, 183, 26, 26, 0, 0, 0},\
    {63, 119, 181, 181, 0, 0, 0},\
    {64, 191, 225, 225, 0, 0, 0},\
    {65, 118, 42, 42, 0, 0, 0},\
    {66, 148, 6, 6, 3, 2, 3},\
    {67, 21, 4, 4, 0, 0, 0},\
    {69, 243, 11, 11, 1, 10, 0},\
    {70, 124, 18, 38, 3, 16, 17},\
    {73, 38, 37, 38, 3, 32, 33},\
    {74, 20, 20, 20, 0, 0, 0},\
    {75, 158, 35, 35, 3, 30, 31},\
    {76, 152, 33, 33, 3, 30, 31},\
    {77, 143, 3, 10, 3, 8, 9},\
    {80, 14, 4, 4, 3, 2, 3},\
    {81, 106, 22, 22, 0, 0, 0},\
    {82, 49, 39, 39, 3, 36, 37},\
    {83, 22, 37, 37, 0, 0, 0},\
    {84, 143, 53, 53, 3, 50, 51},\
    {85, 140, 51, 51, 0, 0, 0},\
    {86, 5, 53, 53, 3, 50, 51},\
    {87, 150, 51, 51, 0, 0, 0},\
    {89, 231, 28, 28, 0, 0, 0},\
    {90, 183, 56, 56, 0, 0, 0},\
    {91, 63, 42, 42, 0, 0, 0},\
    {92, 54, 33, 33, 0, 0, 0},\
    {93, 47, 81, 81, 0, 0, 0},\
    {100, 175, 26, 34, 0, 0, 0},\
    {101, 102, 32, 117, 0, 0, 0},\
    {102, 158, 32, 117, 0, 0, 0},\
    {103, 208, 20, 57, 0, 0, 0},\
    {104, 56, 32, 116, 0, 0, 0},\
    {105, 93, 62, 63, 0, 0, 0},\
    {106, 138, 44, 44, 0, 0, 0},\
    {107, 108, 64, 65, 0, 0, 0},\
    {108, 32, 84, 84, 0, 0, 0},\
    {109, 185, 9, 9, 0, 0, 0},\
    {110, 84, 254, 254, 3, 1, 2},\
    {111, 34, 16, 16, 0, 0, 0},\
    {112, 174, 12, 12, 0, 0, 0},\
    {113, 124, 36, 39, 0, 0, 0},\
    {114, 237, 44, 44, 0, 0, 0},\
    {115, 4, 64, 64, 0, 0, 0},\
    {116, 76, 22, 24, 0, 0, 0},\
    {117, 128, 6, 6, 3, 4, 5},\
    {118, 56, 14, 14, 0, 0, 0},\
    {119, 116, 12, 12, 3, 10, 11},\
    {120, 134, 97, 97, 0, 0, 0},\
    {121, 237, 2, 2, 3, 0, 1},\
    {122, 203, 2, 2, 3, 0, 1},\
    {123, 250, 113, 113, 3, 0, 1},\
    {124, 87, 35, 37, 0, 0, 0},\
    {125, 203, 6, 6, 0, 0, 0},\
    {126, 220, 79, 79, 0, 0, 0},\
    {127, 25, 35, 35, 0, 0, 0},\
    {128, 226, 35, 35, 0, 0, 0},\
    {129, 46, 22, 24, 0, 0, 0},\
    {130, 29, 13, 13, 0, 0, 0},\
    {131, 223, 255, 255, 0, 0, 0},\
    {132, 85, 14, 39, 0, 0, 0},\
    {133, 6, 18, 18, 0, 0, 0},\
    {134, 229, 43, 43, 0, 0, 0},\
    {135, 203, 8, 8, 0, 0, 0},\
    {136, 1, 22, 22, 0, 0, 0},\
    {137, 195, 14, 16, 0, 0, 0},\
    {138, 109, 36, 120, 0, 0, 0},\
    {139, 168, 43, 43, 3, 41, 42},\
    {140, 181, 41, 41, 0, 0, 0},\
    {141, 47, 32, 32, 0, 0, 0},\
    {142, 72, 243, 243, 0, 0, 0},\
    {143, 131, 14, 16, 0, 0, 0},\
    {144, 127, 93, 93, 0, 0, 0},\
    {146, 103, 100, 100, 0, 0, 0},\
    {147, 154, 36, 54, 0, 0, 0},\
    {148, 178, 60, 78, 0, 0, 0},\
    {149, 200, 30, 60, 0, 0, 0},\
    {162, 189, 8, 9, 0, 0, 0},\
    {192, 36, 44, 54, 0, 0, 0},\
    {225, 208, 65, 65, 0, 0, 0},\
    {230, 163, 42, 42, 0, 0, 0},\
    {231, 105, 40, 40, 0, 0, 0},\
    {232, 151, 63, 65, 0, 0, 0},\
    {233, 35, 182, 182, 0, 0, 0},\
    {234, 150, 40, 40, 0, 0, 0},\
    {235, 179, 42, 42, 0, 0, 0},\
    {241, 90, 32, 32, 0, 0, 0},\
    {242, 104, 52, 60, 0, 0, 0},\
    {243, 85, 53, 61, 1, 52, 0},\
    {244, 95, 6, 6, 0, 0, 0},\
    {245, 130, 2, 2, 0, 0, 0},\
    {246, 184, 38, 38, 0, 0, 0},\
    {247, 81, 19, 19, 0, 0, 0},\
    {248, 8, 254, 254, 3, 3, 4},\
    {249, 204, 36, 36, 0, 0, 0},\
    {250, 49, 30, 30, 0, 0, 0},\
    {251, 170, 18, 18, 0, 0, 0},\
    {252, 44, 18, 18, 0, 0, 0},\
    {253, 83, 51, 54, 0, 0, 0},\
    {254, 46, 9, 9, 0, 0, 0},\
    {256, 71, 42, 42, 3, 8, 9},\
    {257, 131, 9, 9, 0, 0, 0},\
    {258, 187, 32, 232, 3, 0, 1},\
    {259, 92, 235, 235, 0, 0, 0},\
    {260, 146, 5, 13, 0, 0, 0},\
    {261, 179, 27, 60, 0, 0, 0},\
    {262, 12, 18, 22, 0, 0, 0},\
    {263, 133, 255, 255, 0, 0, 0},\
    {264, 49, 28, 28, 0, 0, 0},\
    {265, 26, 16, 20, 0, 0, 0},\
    {266, 193, 255, 255, 3, 2, 3},\
    {267, 35, 255, 255, 3, 2, 3},\
    {268, 14, 4, 4, 3, 2, 3},\
    {269, 109, 213, 213, 0, 0, 0},\
    {270, 59, 19, 19, 0, 0, 0},\
    {271, 22, 52, 52, 0, 0, 0},\
    {275, 126, 31, 31, 0, 0, 0},\
    {276, 18, 49, 49, 0, 0, 0},\
    {280, 70, 33, 33, 0, 0, 0},\
    {281, 48, 13, 13, 0, 0, 0},\
    {282, 123, 35, 35, 3, 32, 33},\
    {283, 74, 144, 144, 0, 0, 0},\
    {284, 99, 32, 32, 3, 30, 31},\
    {285, 137, 40, 40, 3, 38, 39},\
    {286, 210, 53, 53, 3, 50, 51},\
    {287, 1, 23, 23, 3, 20, 21},\
    {288, 20, 23, 23, 3, 20, 21},\
    {290, 221, 42, 42, 0, 0, 0},\
    {291, 10, 57, 57, 0, 0, 0},\
    {299, 19, 96, 98, 0, 0, 0},\
    {300, 217, 22, 22, 0, 0, 0},\
    {301, 243, 58, 58, 0, 0, 0},\
    {310, 28, 17, 17, 0, 0, 0},\
    {311, 95, 116, 116, 0, 0, 0},\
    {320, 243, 20, 20, 3, 2, 3},\
    {321, 88, 2, 2, 3, 0, 1},\
    {322, 243, 149, 149, 0, 0, 0},\
    {323, 78, 147, 147, 3, 0, 1},\
    {324, 132, 146, 146, 0, 0, 0},\
    {330, 23, 158, 167, 0, 0, 0},\
    {331, 91, 230, 232, 0, 0, 0},\
    {332, 236, 239, 239, 0, 0, 0},\
    {333, 231, 109, 109, 0, 0, 0},\
    {334, 72, 10, 10, 0, 0, 0},\
    {335, 225, 24, 24, 0, 0, 0},\
    {336, 245, 84, 84, 0, 0, 0},\
    {339, 199, 5, 5, 0, 0, 0},\
    {340, 99, 70, 70, 0, 0, 0},\
    {350, 232, 20, 252, 0, 0, 0},\
    {360, 11, 25, 25, 0, 0, 0},\
    {370, 75, 87, 87, 0, 0, 0},\
    {373, 117, 42, 42, 0, 0, 0},\
    {375, 251, 140, 140, 0, 0, 0},\
    {380, 232, 20, 20, 0, 0, 0},\
    {385, 147, 133, 133, 3, 2, 3},\
    {390, 156, 238, 238, 0, 0, 0},\
    {395, 163, 156, 156, 0, 0, 0},\
    {400, 110, 254, 254, 3, 4, 5},\
    {401, 183, 6, 6, 3, 4, 5},\
    {9000, 113, 137, 137, 0, 0, 0},\
    {9005, 117, 34, 34, 0, 0, 0},\
    {12900, 114, 44, 44, 3, 0, 1},\
    {12901, 254, 59, 59, 3, 30, 31},\
    {12902, 49, 53, 53, 3, 4, 5},\
    {12903, 249, 46, 46, 3, 0, 1},\
    {12904, 203, 46, 46, 3, 20, 21},\
    {12905, 49, 43, 43, 3, 0, 1},\
    {12915, 62, 254, 254, 3, 0, 1}\
}
#endif


#include "../fastmavlink.h"
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------




//------------------------------
//-- Message definitions
//------------------------------




//------------------------------
//-- Dialect includes
//------------------------------

#include "../common/common.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_STANDARD_H

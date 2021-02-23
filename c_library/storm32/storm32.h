//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

#pragma once
#ifndef FASTMAVLINK_STORM32_H
#define FASTMAVLINK_STORM32_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FASTMAVLINK_BUILD_DATE
#define FASTMAVLINK_BUILD_DATE  "Tue Feb 23 2021"
#endif

#ifdef FASTMAVLINK_DIALECT_VERSION
#define FASTMAVLINK_DIALECT_VERSION  1  // version as specified in xml file
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
    {150, 134, 42, 42, 0, 0, 0},\
    {151, 219, 8, 8, 3, 6, 7},\
    {152, 208, 4, 8, 0, 0, 0},\
    {153, 188, 12, 12, 0, 0, 0},\
    {154, 84, 15, 15, 3, 6, 7},\
    {155, 22, 13, 13, 3, 4, 5},\
    {156, 19, 6, 6, 3, 0, 1},\
    {157, 21, 15, 15, 3, 12, 13},\
    {158, 134, 14, 14, 3, 12, 13},\
    {160, 78, 12, 12, 3, 8, 9},\
    {161, 68, 3, 3, 3, 0, 1},\
    {162, 189, 8, 9, 0, 0, 0},\
    {163, 127, 28, 28, 0, 0, 0},\
    {164, 154, 44, 44, 0, 0, 0},\
    {165, 21, 3, 3, 0, 0, 0},\
    {166, 21, 9, 9, 0, 0, 0},\
    {167, 144, 22, 22, 0, 0, 0},\
    {168, 1, 12, 12, 0, 0, 0},\
    {169, 234, 18, 18, 0, 0, 0},\
    {170, 73, 34, 34, 0, 0, 0},\
    {171, 181, 66, 66, 0, 0, 0},\
    {172, 22, 98, 98, 0, 0, 0},\
    {173, 83, 8, 8, 0, 0, 0},\
    {174, 167, 48, 48, 0, 0, 0},\
    {175, 138, 19, 19, 3, 14, 15},\
    {176, 234, 3, 3, 3, 0, 1},\
    {177, 240, 20, 20, 0, 0, 0},\
    {178, 47, 24, 24, 0, 0, 0},\
    {179, 189, 29, 29, 1, 26, 0},\
    {180, 52, 45, 47, 1, 42, 0},\
    {181, 174, 4, 4, 0, 0, 0},\
    {182, 229, 40, 40, 0, 0, 0},\
    {183, 85, 2, 2, 3, 0, 1},\
    {184, 159, 206, 206, 3, 4, 5},\
    {185, 186, 7, 7, 3, 4, 5},\
    {186, 72, 29, 29, 3, 0, 1},\
    {191, 92, 27, 27, 0, 0, 0},\
    {192, 36, 44, 54, 0, 0, 0},\
    {193, 71, 22, 26, 0, 0, 0},\
    {194, 98, 25, 25, 0, 0, 0},\
    {195, 120, 37, 37, 0, 0, 0},\
    {200, 134, 42, 42, 3, 40, 41},\
    {201, 205, 14, 14, 3, 12, 13},\
    {214, 69, 8, 8, 3, 6, 7},\
    {215, 101, 3, 3, 0, 0, 0},\
    {216, 50, 3, 3, 3, 0, 1},\
    {217, 202, 6, 6, 0, 0, 0},\
    {218, 17, 7, 7, 3, 0, 1},\
    {219, 162, 2, 2, 0, 0, 0},\
    {225, 208, 65, 65, 0, 0, 0},\
    {226, 207, 8, 8, 0, 0, 0},\
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
    {10001, 209, 20, 20, 0, 0, 0},\
    {10002, 186, 41, 41, 0, 0, 0},\
    {10003, 4, 1, 1, 0, 0, 0},\
    {11000, 134, 51, 52, 3, 4, 5},\
    {11001, 15, 135, 136, 0, 0, 0},\
    {11002, 234, 179, 180, 3, 4, 5},\
    {11003, 64, 5, 5, 0, 0, 0},\
    {11010, 46, 49, 49, 0, 0, 0},\
    {11011, 106, 44, 44, 0, 0, 0},\
    {11020, 205, 16, 16, 0, 0, 0},\
    {11030, 144, 44, 44, 0, 0, 0},\
    {11031, 133, 44, 44, 0, 0, 0},\
    {11032, 85, 44, 44, 0, 0, 0},\
    {11033, 195, 37, 37, 3, 16, 17},\
    {11034, 79, 5, 5, 0, 0, 0},\
    {11035, 128, 8, 8, 3, 4, 5},\
    {11036, 177, 34, 34, 0, 0, 0},\
    {11037, 130, 28, 28, 0, 0, 0},\
    {12900, 114, 44, 44, 3, 0, 1},\
    {12901, 254, 59, 59, 3, 30, 31},\
    {12902, 49, 53, 53, 3, 4, 5},\
    {12903, 249, 46, 46, 3, 0, 1},\
    {12904, 203, 46, 46, 3, 20, 21},\
    {12905, 49, 43, 43, 3, 0, 1},\
    {12915, 62, 254, 254, 3, 0, 1},\
    {42000, 227, 1, 1, 0, 0, 0},\
    {42001, 239, 46, 46, 0, 0, 0},\
    {60001, 186, 42, 42, 3, 40, 41},\
    {60002, 69, 32, 32, 3, 30, 31},\
    {60010, 208, 33, 33, 0, 0, 0},\
    {60011, 183, 7, 7, 0, 0, 0},\
    {60012, 99, 36, 36, 3, 32, 33},\
    {60013, 129, 24, 24, 3, 20, 21},\
    {60014, 134, 8, 8, 3, 4, 5},\
    {60015, 78, 22, 22, 3, 0, 1},\
    {60020, 202, 4, 4, 0, 0, 0}\
}
#endif


#include "../fastmavlink.h"
#ifdef FASTMAVLINK_PYMAVLINK_ENABLED
#include "../fastmavlink_pymavlink.h"
#endif


//------------------------------
//-- Enum definitons
//------------------------------

#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_TUNNEL_PAYLOAD_TYPE
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_TUNNEL_PAYLOAD_TYPE
typedef enum MAV_STORM32_TUNNEL_PAYLOAD_TYPE {
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_IN = 200,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_OUT = 201,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_IN = 202,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_OUT = 203,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_IN = 204,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_OUT = 205,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209,  // Registered for STorM32 gimbal controller. 
    MAV_STORM32_TUNNEL_PAYLOAD_TYPE_ENUM_END = 210,  // end marker
} MAV_STORM32_TUNNEL_PAYLOAD_TYPE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS
typedef enum MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS {
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1,  // Gimbal device supports a retracted position. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2,  // Gimbal device supports a horizontal, forward looking position, stabilized. Can also be used to reset the gimbal's orientation. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4,  // Gimbal device supports rotating around roll axis. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8,  // Gimbal device supports to follow a roll angle relative to the vehicle. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16,  // Gimbal device supports locking to an roll angle (generally that's the default). 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32,  // Gimbal device supports rotating around pitch axis. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64,  // Gimbal device supports to follow a pitch angle relative to the vehicle. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128,  // Gimbal device supports locking to an pitch angle (generally that's the default). 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256,  // Gimbal device supports rotating around yaw axis. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512,  // Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default). 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024,  // Gimbal device supports locking to a heading angle. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_INFINITE_YAW = 2048,  // Gimbal device supports yawing/panning infinitely (e.g. using a slip ring). 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_ABSOLUTE_YAW = 65536,  // Gimbal device supports absolute yaw angles (this usually requires support by an autopilot, and can be dynamic, i.e., go on and off during runtime). 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_HAS_RC = 131072,  // Gimbal device supports control via an RC input signal. 
    MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS_ENUM_END = 131073,  // end marker
} MAV_STORM32_GIMBAL_DEVICE_CAP_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_FLAGS
typedef enum MAV_STORM32_GIMBAL_DEVICE_FLAGS {
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_RETRACT = 1,  // Retracted safe position (no stabilization), takes presedence over NEUTRAL flag. If supported by the gimbal, the angles in the retracted position can be set in addition. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_NEUTRAL = 2,  // Neutral position (horizontal, forward looking, with stabiliziation). 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4,  // Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8,  // Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16,  // Lock yaw angle to absolute angle relative to earth (not relative to drone). When the YAW_ABSOLUTE flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute), else it is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle). 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_ABSOLUTE = 256,  // Gimbal device can accept absolute yaw angle input. This flag cannot be set, is only for reporting (attempts to set it are rejected by the gimbal device). 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE = 512,  // Yaw angle is absolute (is only accepted if CAN_ACCEPT_YAW_ABSOLUTE is set). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute), else it is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle). 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE = 1024,  // RC control. The RC input signal fed to the gimbal device exclusively controls the gimbal's orientation. Overrides RC_MIXED flag if that is also set. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_RC_MIXED = 2048,  // RC control. The RC input signal fed to the gimbal device is mixed into the gimbal's orientation. Is overriden by RC_EXCLUSIVE flag if that is also set. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_NONE = 65535,  // UINT16_MAX = ignore. 
    MAV_STORM32_GIMBAL_DEVICE_FLAGS_ENUM_END = 65536,  // end marker
} MAV_STORM32_GIMBAL_DEVICE_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS
typedef enum MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS {
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1,  // Gimbal device is limited by hardware roll limit. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2,  // Gimbal device is limited by hardware pitch limit. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4,  // Gimbal device is limited by hardware yaw limit. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8,  // There is an error with the gimbal device's encoders. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16,  // There is an error with the gimbal device's power source. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32,  // There is an error with the gimbal device's motors. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64,  // There is an error with the gimbal device's software. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128,  // There is an error with the gimbal device's communication. 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256,  // Gimbal device is currently calibrating (not an error). 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER = 32768,  // Gimbal device is not assigned to a gimbal manager (not an error). 
    MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS_ENUM_END = 32769,  // end marker
} MAV_STORM32_GIMBAL_DEVICE_ERROR_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS
typedef enum MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS {
    MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS_HAS_PROFILES = 1,  // The gimbal manager supports several profiles. 
    MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_CHANGE = 2,  // The gimbal manager supports changing the gimbal manager during run time, i.e. can be enabled/disabled. 
    MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS_ENUM_END = 3,  // end marker
} MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_FLAGS
typedef enum MAV_STORM32_GIMBAL_MANAGER_FLAGS {
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_NONE = 0,  // 0 = ignore. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_RC_ACTIVE = 1,  // Request to set RC input to active, or report RC input is active. Implies RC mixed. RC exclusive is achieved by setting all clients to inactive. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_ONBOARD_ACTIVE = 2,  // Request to set onboard/companion computer client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_AUTOPILOT_ACTIVE = 4,  // Request to set autopliot client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS_ACTIVE = 8,  // Request to set GCS client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA_ACTIVE = 16,  // Request to set camera client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS2_ACTIVE = 32,  // Request to set GCS2 client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA2_ACTIVE = 64,  // Request to set camera2 client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM_ACTIVE = 128,  // Request to set custom client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM2_ACTIVE = 256,  // Request to set custom2 client to active, or report this client is active. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_SUPERVISON = 512,  // Request supervision. This flag is only for setting, it is not reported. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_RELEASE = 1024,  // Release supervision. This flag is only for setting, it is not reported. 
    MAV_STORM32_GIMBAL_MANAGER_FLAGS_ENUM_END = 1025,  // end marker
} MAV_STORM32_GIMBAL_MANAGER_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_CLIENT
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_CLIENT
typedef enum MAV_STORM32_GIMBAL_MANAGER_CLIENT {
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_NONE = 0,  // For convenience. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_ONBOARD = 1,  // This is the onboard/companion computer client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT = 2,  // This is the autopilot client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS = 3,  // This is the GCS client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA = 4,  // This is the camera client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS2 = 5,  // This is the GCS2 client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA2 = 6,  // This is the camera2 client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM = 7,  // This is the custom client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM2 = 8,  // This is the custom2 client. 
    MAV_STORM32_GIMBAL_MANAGER_CLIENT_ENUM_END = 9,  // end marker
} MAV_STORM32_GIMBAL_MANAGER_CLIENT;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS
typedef enum MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS {
    MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS_ENABLE = 16384,  // Enable gimbal manager. This flag is only for setting, is not reported. 
    MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS_DISABLE = 32768,  // Disable gimbal manager. This flag is only for setting, is not reported. 
    MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS_ENUM_END = 32769,  // end marker
} MAV_STORM32_GIMBAL_MANAGER_SETUP_FLAGS;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_PROFILE
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_MANAGER_PROFILE
typedef enum MAV_STORM32_GIMBAL_MANAGER_PROFILE {
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_DEFAULT = 0,  // Default profile. Implementation specific. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_CUSTOM = 1,  // Custom profile. Configurable profile according to the STorM32 definition. Is configured with STORM32_GIMBAL_MANAGER_PROFIL. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_COOPERATIVE = 2,  // Default cooperative profile. Uses STorM32 custom profile with default settings to achieve cooperative behavior. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_EXCLUSIVE = 3,  // Default exclusive profile. Uses STorM32 custom profile with default settings to achieve exclusive behavior. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_COOPERATIVE = 4,  // Default priority profile with cooperative behavior for equal priority. Uses STorM32 custom profile with default settings to achieve priority-based behavior. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_EXCLUSIVE = 5,  // Default priority profile with exclusive behavior for equal priority. Uses STorM32 custom profile with default settings to achieve priority-based behavior. 
    MAV_STORM32_GIMBAL_MANAGER_PROFILE_ENUM_END = 6,  // end marker
} MAV_STORM32_GIMBAL_MANAGER_PROFILE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_ACTION
#define FASTMAVLINK_HAS_ENUM_MAV_STORM32_GIMBAL_ACTION
typedef enum MAV_STORM32_GIMBAL_ACTION {
    MAV_STORM32_GIMBAL_ACTION_RECENTER = 1,  // Trigger the gimbal device to recenter the gimbal. 
    MAV_STORM32_GIMBAL_ACTION_CALIBRATION = 2,  // Trigger the gimbal device to run a calibration. 
    MAV_STORM32_GIMBAL_ACTION_DISCOVER_MANAGER = 3,  // Trigger gimbal device to (re)discover the gimbal manager during run time. 
    MAV_STORM32_GIMBAL_ACTION_ENUM_END = 4,  // end marker
} MAV_STORM32_GIMBAL_ACTION;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_QSHOT_MODE
#define FASTMAVLINK_HAS_ENUM_MAV_QSHOT_MODE
typedef enum MAV_QSHOT_MODE {
    MAV_QSHOT_MODE_UNDEFINED = 0,  // Undefined shot mode. Can be used to determine if qshots should be used or not. 
    MAV_QSHOT_MODE_DEFAULT = 1,  // Start normal gimbal operation. Is usally used to return back from a shot. 
    MAV_QSHOT_MODE_GIMBAL_RETRACT = 2,  // Load and keep safe gimbal position and stop stabilization. 
    MAV_QSHOT_MODE_GIMBAL_NEUTRAL = 3,  // Load neutral gimbal position and keep it while stabilizing. 
    MAV_QSHOT_MODE_GIMBAL_MISSION = 4,  // Start mission with gimbal control. 
    MAV_QSHOT_MODE_GIMBAL_RC_CONTROL = 5,  // Start RC gimbal control. 
    MAV_QSHOT_MODE_POI_TARGETING = 6,  // Start gimbal tracking the point specified by Lat, Lon, Alt. 
    MAV_QSHOT_MODE_SYSID_TARGETING = 7,  // Start gimbal tracking the system with specified system ID. 
    MAV_QSHOT_MODE_CABLECAM_2POINT = 8,  // Start 2-point cable cam quick shot. 
    MAV_QSHOT_MODE_ENUM_END = 9,  // end marker
} MAV_QSHOT_MODE;
#endif


#ifndef FASTMAVLINK_HAS_ENUM_MAV_CMD
#define FASTMAVLINK_HAS_ENUM_MAV_CMD
typedef enum MAV_CMD {
    MAV_CMD_NAV_WAYPOINT = 16,  // Navigate to waypoint. | Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing) | Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached) | 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control. | Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude | Longitude | Altitude
    MAV_CMD_NAV_LOITER_UNLIM = 17,  // Loiter around this waypoint an unlimited amount of time | Empty | Empty | Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise | Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude | Longitude | Altitude
    MAV_CMD_NAV_LOITER_TURNS = 18,  // Loiter around this waypoint for X turns | Number of turns. | Leave loiter circle only once heading towards the next waypoint (0 = False) | Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise | Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour. | Latitude | Longitude | Altitude
    MAV_CMD_NAV_LOITER_TIME = 19,  // Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint. | Loiter time (only starts once Lat, Lon and Alt is reached). | Leave loiter circle only once heading towards the next waypoint (0 = False) | Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise. | Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour. | Latitude | Longitude | Altitude
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,  // Return to launch location | Empty | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_NAV_LAND = 21,  // Land at location. | Minimum target altitude if landing is aborted (0 = undefined/use system default). | Precision land mode. | Empty. | Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude. | Longitude. | Landing altitude (ground level in current frame).
    MAV_CMD_NAV_TAKEOFF = 22,  // Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. | Minimum pitch (if airspeed sensor present), desired pitch without sensor | Empty | Empty | Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude | Longitude | Altitude
    MAV_CMD_NAV_LAND_LOCAL = 23,  // Land at local position (local frame only) | Landing target number (if available) | Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land | Landing descend rate | Desired yaw angle | Y-axis position | X-axis position | Z-axis / ground level position
    MAV_CMD_NAV_TAKEOFF_LOCAL = 24,  // Takeoff from local position (local frame only) | Minimum pitch (if airspeed sensor present), desired pitch without sensor | Empty | Takeoff ascend rate | Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these | Y-axis position | X-axis position | Z-axis position
    MAV_CMD_NAV_FOLLOW = 25,  // Vehicle following, i.e. this waypoint represents the position of a moving vehicle | Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation | Ground speed of vehicle to be followed | Radius around waypoint. If positive loiter clockwise, else counter-clockwise | Desired yaw angle. | Latitude | Longitude | Altitude
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,  // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. | Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty | Empty | Empty | Empty | Empty | Desired altitude
    MAV_CMD_NAV_LOITER_TO_ALT = 31,  // Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. | Leave loiter circle only once heading towards the next waypoint (0 = False) | Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter. | Empty | Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour. | Latitude | Longitude | Altitude
    MAV_CMD_DO_FOLLOW = 32,  // Begin following a target | System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode. | Reserved | Reserved | Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home. | Altitude above home. (used if mode=2) | Reserved | Time to land in which the MAV should go to the default position hold mode after a message RX timeout.
    MAV_CMD_DO_FOLLOW_REPOSITION = 33,  // Reposition the MAV after a follow target command has been sent | Camera q1 (where 0 is on the ray from the camera to the tracking device) | Camera q2 | Camera q3 | Camera q4 | altitude offset from target | X offset from target | Y offset from target
    MAV_CMD_DO_ORBIT = 34,  // Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. | Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise. | Tangential Velocity. NaN: Vehicle configuration default. | Yaw behavior of the vehicle. | Reserved (e.g. for dynamic center beacon options) | Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting. | Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting. | Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.
    MAV_CMD_NAV_ROI = 80,  // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. | Region of interest mode. | Waypoint index/ target ID. (see MAV_ROI enum) | ROI index (allows a vehicle to manage multiple ROI's) | Empty | x the location of the fixed ROI (see MAV_FRAME) | y | z
    MAV_CMD_NAV_PATHPLANNING = 81,  // Control autonomous path planning on the MAV. | 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning | 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid | Empty | Yaw angle at goal | Latitude/X of goal | Longitude/Y of goal | Altitude/Z of goal
    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,  // Navigate to waypoint using a spline path. | Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing) | Empty | Empty | Empty | Latitude/X of goal | Longitude/Y of goal | Altitude/Z of goal
    MAV_CMD_NAV_ALTITUDE_WAIT = 83,  // Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. | Altitude. | Descent speed. | How long to wiggle the control surfaces to prevent them seizing up. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_NAV_VTOL_TAKEOFF = 84,  // Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). | Empty | Front transition heading. | Empty | Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude | Longitude | Altitude
    MAV_CMD_NAV_VTOL_LAND = 85,  // Land using VTOL mode | Empty | Empty | Approach altitude (with the same reference as the Altitude field). NaN if unspecified. | Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). | Latitude | Longitude | Altitude (ground level)
    MAV_CMD_NAV_GUIDED_ENABLE = 92,  // hand control over to an external controller | On / Off (> 0.5f on) | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_NAV_DELAY = 93,  // Delay the next navigation command a number of seconds or until a specified time | Delay (-1 to enable time-of-day fields) | hour (24h format, UTC, -1 to ignore) | minute (24h format, UTC, -1 to ignore) | second (24h format, UTC, -1 to ignore) | Empty | Empty | Empty
    MAV_CMD_NAV_PAYLOAD_PLACE = 94,  // Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. | Maximum distance to descend. | Empty | Empty | Empty | Latitude | Longitude | Altitude
    MAV_CMD_NAV_LAST = 95,  // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration | Empty | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_CONDITION_DELAY = 112,  // Delay mission state machine. | Delay | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_CONDITION_CHANGE_ALT = 113,  // Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. | Descent / Ascend rate. | Empty | Empty | Empty | Empty | Empty | Target Altitude
    MAV_CMD_CONDITION_DISTANCE = 114,  // Delay mission state machine until within desired distance of next NAV point. | Distance. | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_CONDITION_YAW = 115,  // Reach a certain target angle. | target angle, 0 is north | angular speed | direction: -1: counter clockwise, 1: clockwise | 0: absolute angle, 1: relative offset | Empty | Empty | Empty
    MAV_CMD_CONDITION_LAST = 159,  // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration | Empty | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_MODE = 176,  // Set system mode. | Mode | Custom mode - this is system specific, please refer to the individual autopilot specifications for details. | Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details. | Empty | Empty | Empty | Empty
    MAV_CMD_DO_JUMP = 177,  // Jump to the desired command in the mission list.  Repeat this action only the specified number of times | Sequence number | Repeat count | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_CHANGE_SPEED = 178,  // Change speed and/or throttle set points. | Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed) | Speed (-1 indicates no change) | Throttle (-1 indicates no change) | 0: absolute, 1: relative | Empty | Empty | Empty
    MAV_CMD_DO_SET_HOME = 179,  // Changes the home location either to the current location or a specified location. | Use current (1=use current location, 0=use specified location) | Empty | Empty | Yaw angle. NaN to use default heading | Latitude | Longitude | Altitude
    MAV_CMD_DO_SET_PARAMETER = 180,  // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. | Parameter number | Parameter value | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_RELAY = 181,  // Set a relay to a condition. | Relay instance number. | Setting. (1=on, 0=off, others possible depending on system hardware) | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_REPEAT_RELAY = 182,  // Cycle a relay on and off for a desired number of cycles with a desired period. | Relay instance number. | Cycle count. | Cycle time. | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_SERVO = 183,  // Set a servo to a desired PWM value. | Servo instance number. | Pulse Width Modulation. | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_REPEAT_SERVO = 184,  // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. | Servo instance number. | Pulse Width Modulation. | Cycle count. | Cycle time. | Empty | Empty | Empty
    MAV_CMD_DO_FLIGHTTERMINATION = 185,  // Terminate flight immediately | Flight termination activated if > 0.5 | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_CHANGE_ALTITUDE = 186,  // Change altitude set point. | Altitude. | Frame of new altitude. | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_ACTUATOR = 187,  // Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). | Actuator 1 value, scaled from [-1 to 1]. NaN to ignore. | Actuator 2 value, scaled from [-1 to 1]. NaN to ignore. | Actuator 3 value, scaled from [-1 to 1]. NaN to ignore. | Actuator 4 value, scaled from [-1 to 1]. NaN to ignore. | Actuator 5 value, scaled from [-1 to 1]. NaN to ignore. | Actuator 6 value, scaled from [-1 to 1]. NaN to ignore. | Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)
    MAV_CMD_DO_LAND_START = 189,  // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. | Empty | Empty | Empty | Empty | Latitude | Longitude | Empty
    MAV_CMD_DO_RALLY_LAND = 190,  // Mission command to perform a landing from a rally point. | Break altitude | Landing speed | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_GO_AROUND = 191,  // Mission command to safely abort an autonomous landing. | Altitude | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_REPOSITION = 192,  // Reposition the vehicle to a specific WGS84 global position. | Ground speed, less than 0 (-1) for default | Bitmask of option flags. | Reserved | Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise) | Latitude | Longitude | Altitude
    MAV_CMD_DO_PAUSE_CONTINUE = 193,  // If in a GPS controlled position mode, hold the current position or continue. | 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius. | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved
    MAV_CMD_DO_SET_REVERSE = 194,  // Set moving direction to forward or reverse. | Direction (0=Forward, 1=Reverse) | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_ROI_LOCATION = 195,  // Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message. | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals). | Empty | Empty | Empty | Latitude of ROI location | Longitude of ROI location | Altitude of ROI location
    MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,  // Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals). | Empty | Empty | Empty | Pitch offset from next waypoint, positive pitching up | Roll offset from next waypoint, positive rolling to the right | Yaw offset from next waypoint, positive yawing to the right
    MAV_CMD_DO_SET_ROI_NONE = 197,  // Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals). | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_ROI_SYSID = 198,  // Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. | System ID | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DO_SPRAYER = 199,  // Control attached liquid sprayer | 0: disable sprayer. 1: enable sprayer. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_DO_CONTROL_VIDEO = 200,  // Control onboard camera system. | Camera ID (-1 for all) | Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw | Transmission mode: 0: video stream, >0: single images every n seconds | Recording: 0: disabled, 1: enabled compressed, 2: enabled raw | Empty | Empty | Empty
    MAV_CMD_DO_SET_ROI = 201,  // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. | Region of interest mode. | Waypoint index/ target ID (depends on param 1). | Region of interest index. (allows a vehicle to manage multiple ROI's) | Empty | MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude | MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude | MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude
    MAV_CMD_DO_DIGICAM_CONFIGURE = 202,  // Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). | Modes: P, TV, AV, M, Etc. | Shutter speed: Divisor number for one second. | Aperture: F stop number. | ISO number e.g. 80, 100, 200, Etc. | Exposure type enumerator. | Command Identity. | Main engine cut-off time before camera trigger. (0 means no cut-off)
    MAV_CMD_DO_DIGICAM_CONTROL = 203,  // Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). | Session control e.g. show/hide lens | Zoom's absolute position | Zooming step value to offset zoom from the current position | Focus Locking, Unlocking or Re-locking | Shooting Command | Command Identity | Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
    MAV_CMD_DO_MOUNT_CONFIGURE = 204,  // Mission command to configure a camera or antenna mount | Mount operation mode | stabilize roll? (1 = yes, 0 = no) | stabilize pitch? (1 = yes, 0 = no) | stabilize yaw? (1 = yes, 0 = no) | roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame) | pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame) | yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
    MAV_CMD_DO_MOUNT_CONTROL = 205,  // Mission command to control a camera or antenna mount | pitch depending on mount mode (degrees or degrees/second depending on pitch input). | roll depending on mount mode (degrees or degrees/second depending on roll input). | yaw depending on mount mode (degrees or degrees/second depending on yaw input). | altitude depending on mount mode. | latitude, set if appropriate mount mode. | longitude, set if appropriate mount mode. | Mount mode.
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,  // Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. | Camera trigger distance. 0 to stop triggering. | Camera shutter integration time. -1 or 0 to ignore | Trigger camera once immediately. (0 = no trigger, 1 = trigger) | Empty | Empty | Empty | Empty
    MAV_CMD_DO_FENCE_ENABLE = 207,  // Mission command to enable the geofence | enable? (0=disable, 1=enable, 2=disable_floor_only) | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_PARACHUTE = 208,  // Mission item/command to release a parachute or enable/disable auto release. | Action | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_MOTOR_TEST = 209,  // Mission command to perform motor test. | Motor instance number. (from 1 to max number of motors on the vehicle) | Throttle type. | Throttle. | Timeout. | Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...) | Motor test order. | Empty
    MAV_CMD_DO_INVERTED_FLIGHT = 210,  // Change to/from inverted flight. | Inverted flight. (0=normal, 1=inverted) | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_GRIPPER = 211,  // Mission command to operate a gripper. | Gripper instance number. | Gripper action to perform. | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_AUTOTUNE_ENABLE = 212,  // Enable/disable autotune. | Enable (1: enable, 0:disable). | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_NAV_SET_YAW_SPEED = 213,  // Sets a desired vehicle turn angle and speed change. | Yaw angle to adjust steering by. | Speed. | Final angle. (0=absolute, 1=relative) | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,  // Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. | Camera trigger cycle time. -1 or 0 to ignore. | Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore. | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_RESUME_REPEAT_DIST = 215,  // Set the distance to be repeated on mission resume | Distance. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,  // Mission command to control a camera or antenna mount, using a quaternion as reference. | quaternion param q1, w (1 in null-rotation) | quaternion param q2, x (0 in null-rotation) | quaternion param q3, y (0 in null-rotation) | quaternion param q4, z (0 in null-rotation) | Empty | Empty | Empty
    MAV_CMD_DO_GUIDED_MASTER = 221,  // set id of master controller | System ID | Component ID | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_GUIDED_LIMITS = 222,  // Set limits for external control | Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout. | Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit. | Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit. | Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit. | Empty | Empty | Empty
    MAV_CMD_DO_ENGINE_CONTROL = 223,  // Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines | 0: Stop engine, 1:Start Engine | 0: Warm start, 1:Cold start. Controls use of choke where applicable | Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay. | Empty | Empty | Empty | Empty
    MAV_CMD_DO_SET_MISSION_CURRENT = 224,  // Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). | Mission sequence value to set | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_DO_LAST = 240,  // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration | Empty | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_PREFLIGHT_CALIBRATION = 241,  // Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. | 1: gyro calibration, 3: gyro temperature calibration | 1: magnetometer calibration | 1: ground pressure calibration | 1: radio RC calibration, 2: RC trim calibration | 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration | 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration | 1: ESC calibration, 3: barometer temperature calibration
    MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,  // Set sensor offsets. This command will be only accepted if in pre-flight mode. | Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer | X axis offset (or generic dimension 1), in the sensor's raw units | Y axis offset (or generic dimension 2), in the sensor's raw units | Z axis offset (or generic dimension 3), in the sensor's raw units | Generic dimension 4, in the sensor's raw units | Generic dimension 5, in the sensor's raw units | Generic dimension 6, in the sensor's raw units
    MAV_CMD_PREFLIGHT_UAVCAN = 243,  // Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). | 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command. | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved
    MAV_CMD_PREFLIGHT_STORAGE = 245,  // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. | Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults | Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults | Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging) | Reserved | Empty | Empty | Empty
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,  // Request the reboot or shutdown of system components. | 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded. | 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded. | WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded | WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded | Reserved (set to 0) | Reserved (set to 0) | WIP: ID (e.g. camera ID -1 for all IDs)
    MAV_CMD_DO_UPGRADE = 247,  // Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https://mavlink.io/en/services/command.html. | Component id of the component to be upgraded. If set to 0, all components should be upgraded. | 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed. | Reserved | Reserved | Reserved | Reserved | WIP: upgrade progress report rate (can be used for more granular control).
    MAV_CMD_OVERRIDE_GOTO = 252,  // Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. | MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission. | MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position. | Coordinate frame of hold point. | Desired yaw angle. | Latitude/X position. | Longitude/Y position. | Altitude/Z position.
    MAV_CMD_OBLIQUE_SURVEY = 260,  // Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. | Camera trigger distance. 0 to stop triggering. | Camera shutter integration time. 0 to ignore | The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore. | Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5). | Angle limits that the camera can be rolled to left and right of center. | Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis. | Empty
    MAV_CMD_MISSION_START = 300,  // start running a mission | first_item: the first mission item to run | last_item:  the last mission item to run (after this item is run, the mission ends) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_COMPONENT_ARM_DISARM = 400,  // Arms / Disarms a component | 0: disarm, 1: arm | 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_ILLUMINATOR_ON_OFF = 405,  // Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). | 0: Illuminators OFF, 1: Illuminators ON | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_GET_HOME_POSITION = 410,  // Request the home position from the vehicle. | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved
    MAV_CMD_INJECT_FAILURE = 420,  // Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. | The unit which is affected by the failure. | The type how the failure manifests itself. | Instance affected by failure (0 to signal all). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_START_RX_PAIR = 500,  // Starts receiver pairing. | 0:Spektrum. | RC type. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_GET_MESSAGE_INTERVAL = 510,  // Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. | The MAVLink message ID | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,  // Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. | The MAVLink message ID | The interval between two messages. Set to -1 to disable and 0 to request default rate. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    MAV_CMD_REQUEST_MESSAGE = 512,  // Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). | The MAVLink message ID of the requested message. | Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0). | The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0). | The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0). | The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0). | The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0). | Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.
    MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,  // Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message | 1: Request supported protocol versions by all nodes on the network | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,  // Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message | 1: Request autopilot version | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,  // Request camera information (CAMERA_INFORMATION). | 0: No action 1: Request camera capabilities | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,  // Request camera settings (CAMERA_SETTINGS). | 0: No Action 1: Request camera settings | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,  // Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. | Storage ID (0 for all, 1 for first, 2 for second, etc.) | 0: No Action 1: Request storage information | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_STORAGE_FORMAT = 526,  // Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. | Storage ID (1 for first, 2 for second, etc.) | Format storage (and reset image log). 0: No action 1: Format storage | Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,  // Request camera capture status (CAMERA_CAPTURE_STATUS) | 0: No Action 1: Request camera capture status | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,  // Request flight information (FLIGHT_INFORMATION) | 1: Request flight information | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_RESET_CAMERA_SETTINGS = 529,  // Reset all camera settings to Factory Default | 0: No Action 1: Reset all settings | Reserved (all remaining params) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_CAMERA_MODE = 530,  // Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. | Reserved (Set to 0) | Camera mode | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_SET_CAMERA_ZOOM = 531,  // Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). | Zoom type | Zoom value. The range of valid values depend on the zoom type. | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_SET_CAMERA_FOCUS = 532,  // Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). | Focus type | Focus value | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_JUMP_TAG = 600,  // Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. | Tag. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DO_JUMP_TAG = 601,  // Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. | Target tag to jump to. | Repeat count. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_PARAM_TRANSACTION = 900,  // Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. | Action to be performed (start, commit, cancel, etc.) | Possible transport layers to set and get parameters via mavlink during a parameter transaction. | Identifier for a specific transaction. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,  // High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. | Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). | Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). | Pitch rate (positive to pitch up). | Yaw rate (positive to yaw to the right). | Gimbal manager flags to use. | Reserved (default:0) | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,  // Gimbal configuration to set which sysid/compid is in primary and secondary control. | Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control). | Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control). | Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control). | Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control). | Reserved (default:0) | Reserved (default:0) | Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
    MAV_CMD_IMAGE_START_CAPTURE = 2000,  // Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. | Reserved (Set to 0) | Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds). | Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE. | Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted. | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN)
    MAV_CMD_IMAGE_STOP_CAPTURE = 2001,  // Stop image capture sequence Use NaN for reserved values. | Reserved (Set to 0) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,  // Re-request a CAMERA_IMAGE_CAPTURED message. | Sequence number for missing CAMERA_IMAGE_CAPTURED message | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:0) | Reserved (default:0) | Reserved (default:NaN)
    MAV_CMD_DO_TRIGGER_CONTROL = 2003,  // Enable or disable on-board camera triggering system. | Trigger enable/disable (0 for disable, 1 for start), -1 to ignore | 1 to reset the trigger sequence, -1 or 0 to ignore | 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_CAMERA_TRACK_POINT = 2004,  // If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. | Point to track x value (normalized 0..1, 0 is left, 1 is right). | Point to track y value (normalized 0..1, 0 is top, 1 is bottom). | Point radius (normalized 0..1, 0 is image left, 1 is image right). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_CAMERA_TRACK_RECTANGLE = 2005,  // If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. | Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right). | Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom). | Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right). | Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_CAMERA_STOP_TRACKING = 2010,  // Stops ongoing tracking. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_VIDEO_START_CAPTURE = 2500,  // Starts video capture (recording). | Video Stream ID (0 for all streams) | Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN)
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,  // Stop the current video capture (recording). | Video Stream ID (0 for all streams) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN)
    MAV_CMD_VIDEO_START_STREAMING = 2502,  // Start video streaming | Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_VIDEO_STOP_STREAMING = 2503,  // Stop the given video stream | Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,  // Request video stream information (VIDEO_STREAM_INFORMATION) | Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_REQUEST_VIDEO_STREAM_STATUS = 2505,  // Request video stream status (VIDEO_STREAM_STATUS) | Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_LOGGING_START = 2510,  // Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) | Format: 0: ULog | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0)
    MAV_CMD_LOGGING_STOP = 2511,  // Request to stop streaming log data over MAVLink | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0)
    MAV_CMD_AIRFRAME_CONFIGURATION = 2520,  //  | Landing gear ID (default: 0, -1 for all) | Landing gear position (Down: 0, Up: 1, NaN for no change) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN) | Reserved (default:NaN)
    MAV_CMD_CONTROL_HIGH_LATENCY = 2600,  // Request to start/stop transmitting over the high latency telemetry | Control transmission over high latency telemetry (0: stop, 1: start) | Empty | Empty | Empty | Empty | Empty | Empty
    MAV_CMD_PANORAMA_CREATE = 2800,  // Create a panorama at the current position | Viewing angle horizontal of the panorama (+- 0.5 the total angle) | Viewing angle vertical of panorama. | Speed of the horizontal rotation. | Speed of the vertical rotation. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DO_VTOL_TRANSITION = 3000,  // Request VTOL transition | The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,  // Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.         | Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,  // This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.                   | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,  // This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.                   | Radius of desired circle in CIRCLE_MODE | User defined | User defined | User defined | Target latitude of center of circle in CIRCLE_MODE | Target longitude of center of circle in CIRCLE_MODE | Reserved (default:0)
    MAV_CMD_CONDITION_GATE = 4501,  // Delay mission state machine until gate has been reached. | Geometry: 0: orthogonal to path between previous and next waypoint. | Altitude: 0: ignore altitude | Empty | Empty | Latitude | Longitude | Altitude
    MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,  // Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead. | Reserved | Reserved | Reserved | Reserved | Latitude | Longitude | Altitude
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,  // Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.         | Polygon vertex count | Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon | Reserved | Reserved | Latitude | Longitude | Reserved
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,  // Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.         | Polygon vertex count | Reserved | Reserved | Reserved | Latitude | Longitude | Reserved
    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,  // Circular fence area. The vehicle must stay inside this area.         | Radius. | Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group | Reserved | Reserved | Latitude | Longitude | Reserved
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,  // Circular fence area. The vehicle must stay outside this area.         | Radius. | Reserved | Reserved | Reserved | Latitude | Longitude | Reserved
    MAV_CMD_NAV_RALLY_POINT = 5100,  // Rally point. You can have multiple rally points defined.         | Reserved | Reserved | Reserved | Reserved | Latitude | Longitude | Altitude
    MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,  // Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0) | Reserved (set to 0)
    MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,  // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. | Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list. | Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will. | Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will. | Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will. | Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled) | Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled) | Altitude (MSL)
    MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,  // Control the payload deployment. | Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests. | Reserved | Reserved | Reserved | Reserved | Reserved | Reserved
    MAV_CMD_WAYPOINT_USER_1 = 31000,  // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_WAYPOINT_USER_2 = 31001,  // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_WAYPOINT_USER_3 = 31002,  // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_WAYPOINT_USER_4 = 31003,  // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_WAYPOINT_USER_5 = 31004,  // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_SPATIAL_USER_1 = 31005,  // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_SPATIAL_USER_2 = 31006,  // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_SPATIAL_USER_3 = 31007,  // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_SPATIAL_USER_4 = 31008,  // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_SPATIAL_USER_5 = 31009,  // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. | User defined | User defined | User defined | User defined | Latitude unscaled | Longitude unscaled | Altitude (MSL)
    MAV_CMD_USER_1 = 31010,  // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. | User defined | User defined | User defined | User defined | User defined | User defined | User defined
    MAV_CMD_USER_2 = 31011,  // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. | User defined | User defined | User defined | User defined | User defined | User defined | User defined
    MAV_CMD_USER_3 = 31012,  // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. | User defined | User defined | User defined | User defined | User defined | User defined | User defined
    MAV_CMD_USER_4 = 31013,  // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. | User defined | User defined | User defined | User defined | User defined | User defined | User defined
    MAV_CMD_USER_5 = 31014,  // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. | User defined | User defined | User defined | User defined | User defined | User defined | User defined
    MAV_CMD_POWER_OFF_INITIATED = 42000,  // A system wide power-off event has been initiated. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_SOLO_BTN_FLY_CLICK = 42001,  // FLY button has been clicked. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_SOLO_BTN_FLY_HOLD = 42002,  // FLY button has been held for 1.5 seconds. | Takeoff altitude. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_SOLO_BTN_PAUSE_CLICK = 42003,  // PAUSE button has been clicked. | 1 if Solo is in a shot mode, 0 otherwise. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_FIXED_MAG_CAL = 42004,  // Magnetometer calibration based on fixed position        in earth field given by inclination, declination and intensity. | Magnetic declination. | Magnetic inclination. | Magnetic intensity. | Yaw. | Empty. | Empty. | Empty.
    MAV_CMD_FIXED_MAG_CAL_FIELD = 42005,  // Magnetometer calibration based on fixed expected field values. | Field strength X. | Field strength Y. | Field strength Z. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_FIXED_MAG_CAL_YAW = 42006,  // Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. | Yaw of vehicle in earth frame. | CompassMask, 0 for all. | Latitude. | Longitude. | Empty. | Empty. | Empty.
    MAV_CMD_DO_START_MAG_CAL = 42424,  // Initiate a magnetometer calibration. | Bitmask of magnetometers to calibrate. Use 0 to calibrate all sensors that can be started (sensors may not start if disabled, unhealthy, etc.). The command will NACK if calibration does not start for a sensor explicitly specified by the bitmask. | Automatically retry on failure (0=no retry, 1=retry). | Save without user input (0=require input, 1=autosave). | Delay. | Autoreboot (0=user reboot, 1=autoreboot). | Empty. | Empty.
    MAV_CMD_DO_ACCEPT_MAG_CAL = 42425,  // Accept a magnetometer calibration. | Bitmask of magnetometers that calibration is accepted (0 means all). | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_DO_CANCEL_MAG_CAL = 42426,  // Cancel a running magnetometer calibration. | Bitmask of magnetometers to cancel a running calibration (0 means all). | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_SET_FACTORY_TEST_MODE = 42427,  // Command autopilot to get into factory test/diagnostic mode. | 0: activate test mode, 1: exit test mode. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_DO_SEND_BANNER = 42428,  // Reply with the version banner. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_ACCELCAL_VEHICLE_POS = 42429,  // Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. | Position. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_GIMBAL_RESET = 42501,  // Causes the gimbal to reset and boot as if it was just powered on. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS = 42502,  // Reports progress and success or failure of gimbal axis calibration procedure. | Gimbal axis we're reporting calibration progress for. | Current calibration progress for this axis. | Status of the calibration. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503,  // Starts commutation calibration on the gimbal. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_GIMBAL_FULL_RESET = 42505,  // Erases gimbal application and parameters. | Magic number. | Magic number. | Magic number. | Magic number. | Magic number. | Magic number. | Magic number.
    MAV_CMD_DO_WINCH = 42600,  // Command to operate winch. | Winch instance number. | Action to perform. | Length of cable to release (negative to wind). | Release rate (negative to wind). | Empty. | Empty. | Empty.
    MAV_CMD_FLASH_BOOTLOADER = 42650,  // Update the bootloader | Empty | Empty | Empty | Empty | Magic number - set to 290876 to actually flash | Empty | Empty
    MAV_CMD_BATTERY_RESET = 42651,  // Reset battery capacity for batteries that accumulate consumed battery via integration. | Bitmask of batteries to reset. Least significant bit is for the first battery. | Battery percentage remaining to set. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_DEBUG_TRAP = 42700,  // Issue a trap signal to the autopilot process, presumably to enter the debugger. | Magic number - set to 32451 to actually trap. | Empty. | Empty. | Empty. | Empty. | Empty. | Empty.
    MAV_CMD_SCRIPTING = 42701,  // Control onboard scripting. | Scripting command to execute | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_GUIDED_CHANGE_SPEED = 43000,  // Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) | Airspeed or groundspeed. | Target Speed | Acceleration rate, 0 to take effect instantly | Empty | Empty | Empty | Empty
    MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001,  // Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) | Empty | Empty | Rate of change, toward new altitude. 0 for maximum rate change. Positive numbers only, as negative numbers will not converge on the new target alt. | Empty | Empty | Empty | Target Altitude
    MAV_CMD_GUIDED_CHANGE_HEADING = 43002,  // Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) | course-over-ground or raw vehicle heading. | Target heading. | Maximum centripetal accelearation, ie rate of change,  toward new heading. | Empty | Empty | Empty | Empty
    MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW = 60002,  // Command to a gimbal manager to control the gimbal tilt and pan angles. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. A gimbal device is never to react to this command. | Pitch/tilt angle (positive: tilt up, NaN to be ignored). | Yaw/pan angle (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored). | Pitch/tilt rate (positive: tilt up, NaN to be ignored). | Yaw/pan rate (positive: pan to the right, the frame is determined by the STORM32_GIMBAL_DEVICE_FLAGS_YAW_ABSOLUTE flag, NaN to be ignored). | Gimbal device flags. | Gimbal manager flags. | Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals). The client is copied into bits 8-15.
    MAV_CMD_STORM32_DO_GIMBAL_MANAGER_SETUP = 60010,  // Command to configure a gimbal manager. A gimbal device is never to react to this command. The selected profile is reported in the STORM32_GIMBAL_MANAGER_STATUS message. | Gimbal manager profile (0 = default). | Gimbal manager setup flags (0 = none). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals). Send command multiple times for more than one but not all gimbals.
    MAV_CMD_STORM32_DO_GIMBAL_ACTION = 60011,  // Command to initiate gimbal actions. Usually performed by the gimbal device, but some can also be done by the gimbal manager. It is hence best to broadcast this command. | Gimbal action to initiate (0 = none). | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Gimbal ID of the gimbal to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals). Send command multiple times for more than one but not all gimbals.
    MAV_CMD_QSHOT_DO_CONFIGURE = 60020,  // Command to set the shot manager mode. | Set shot mode. | Set shot state or command. The allowed values are specific to the selected shot mode. | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0) | Reserved (default:0)
    MAV_CMD_ENUM_END = 60021,  // end marker
} MAV_CMD;
#endif


//------------------------------
//-- Message definitions
//------------------------------

#include "./mavlink_msg_storm32_gimbal_device_status.h"
#include "./mavlink_msg_storm32_gimbal_device_control.h"
#include "./mavlink_msg_storm32_gimbal_manager_information.h"
#include "./mavlink_msg_storm32_gimbal_manager_status.h"
#include "./mavlink_msg_storm32_gimbal_manager_control.h"
#include "./mavlink_msg_storm32_gimbal_manager_control_pitchyaw.h"
#include "./mavlink_msg_storm32_gimbal_manager_correct_roll.h"
#include "./mavlink_msg_storm32_gimbal_manager_profile.h"
#include "./mavlink_msg_qshot_status.h"


//------------------------------
//-- Dialect includes
//------------------------------

#include "../ardupilotmega/ardupilotmega.h"


#ifdef __cplusplus
}
#endif

#endif // FASTMAVLINK_STORM32_H

//------------------------------
// The fastMavlink library
// (c) OlliW, OlliW42, www.olliw.eu
//------------------------------

//------------------------------
// test_messages
//------------------------------

#pragma once
#ifndef FASTMAVLINK_TEST_CONFIG_H
#define FASTMAVLINK_TEST_CONFIG_H


#ifndef FASTMAVLINK_TEST_GLOBAL_S_LEN
#define FASTMAVLINK_TEST_GLOBAL_S_LEN  16000
#endif

#ifndef FASTMAVLINK_TEST_HELPER_S_LEN
#define FASTMAVLINK_TEST_HELPER_S_LEN  1024
#endif

#ifndef STRCAT
#define STRCAT(s,l,a) strcat_s((s),(l),(a))
#endif

#ifndef PRINTF
#define PRINTF(s) printf(s)
#endif

#ifndef SPRINTF
#define SPRINTF(s,l,...) sprintf_s(s,l,__VA_ARGS__)
#endif


#endif // FASTMAVLINK_TEST_CONFIG_H
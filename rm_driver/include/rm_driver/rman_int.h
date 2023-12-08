#ifndef _RMAN_INT_H
#define _RMAN_INT_H

#include <stdint.h>

#if !defined __stdint_h && !defined  _STDINT && !defined _GCC_STDINT_H
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
// typedef long long          int64_t;

typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
// typedef unsigned long long uint64_t;
#endif

#endif
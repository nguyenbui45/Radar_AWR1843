
/**
 * @file swpform.h
 *
 * @desc This file defines the types to be used when programming proton
 *
 * Copyright (C) 2005-2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef _SWPFORM_H_
#define _SWPFORM_H_

#ifdef INLINE
#undef INLINE
#endif

#ifdef RESTRICT
#undef RESTRICT
#endif

#ifdef IN
#undef IN
#endif

#ifdef OUT
#undef OUT
#endif

#ifdef INOUT
#undef INOUT
#endif

#if defined (__TI_COMPILER_VERSION__)
/******************/
/* C6000 compiler */
/******************/
#if defined (_TMS320C6X)

#if defined (__cplusplus)
/* c++ specific */

#include <inttypes.h>           /* c99 types - cinttypes not available on c6000 compiler */
#define INLINE static inline
#define RESTRICT restrict
#include <cstddef>
#include <climits>
#include <cfloat.h>
#include <ciso646>

#else
/* c specific */

#include <inttypes.h>           /* c99 types */
#include <stdbool.h>           /* _Bool type */
//typedef unsigned int _Bool;     /* c99 types */
//typedef _Bool bool;             /* c99 types */
#define true 1                  /* c99 types */
#define false 0                 /* c99 types */
#define __bool_true_false_are_defined   1   /* c99 types */
#define INLINE static inline
#define RESTRICT restrict
#include <stddef.h>
#include <limits.h>
#include <float.h>
#include <iso646.h>

#endif

#include <c6x.h>                /* get access to global registers */

/* various macros */
#define IN
#define OUT
#define INOUT

#else

//#error Target shall be C6x
#define _LITTLE_ENDIAN

#endif

#elif defined (__GNUC__)
/****************/
/* GNU compiler */
/****************/

#if defined (__cplusplus)
/* c++ specific */

#if defined (__GXX_EXPERIMENTAL_CXX0X__)

#include <cinttypes>            /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef long long int40_t;      /* c99 types */
typedef unsigned long long uint40_t;    /* c99 types */
typedef int40_t int_least40_t;  /* c99 types */
typedef uint40_t uint_least40_t;    /* c99 types */
typedef int40_t int_fast40_t;   /* c99 types */
typedef uint40_t uint_fast40_t; /* c99 types */
#define  INT40_MAX  0x7fffffffff    /* c99 types */
#define  INT40_MIN  (-INT40_MAX-1)  /* c99 types */
#define UINT40_MAX  0xffffffffff    /* c99 types */
#define  INT_LEAST40_MAX  INT40_MAX /* c99 types */
#define  INT_LEAST40_MIN  INT40_MIN /* c99 types */
#define UINT_LEAST40_MAX  UINT40_MAX    /* c99 types */
#define  INT_FAST40_MAX  INT40_MAX  /* c99 types */
#define  INT_FAST40_MIN  INT40_MIN  /* c99 types */
#define UINT_FAST40_MAX  UINT40_MAX /* c99 types */
#define  INT40_C(value) ((int_least40_t)(value))    /* c99 types */
#define UINT40_C(value) ((uint_least40_t)(value))   /* c99 types */
#endif

#else

#error Please enable C++0X compilation

#endif

#define INLINE static __inline__
#define RESTRICT __restrict__
#include <cstddef>
#include <climits>
#include <cfloat>
#include <ciso646>

#else
/* c specific */

#if (__STDC_VERSION__ >= 199901L)

#include <inttypes.h>           /* c99 types */
#include <stdbool.h>            /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef long long int40_t;      /* c99 types */
typedef unsigned long long uint40_t;    /* c99 types */
typedef int40_t int_least40_t;  /* c99 types */
typedef uint40_t uint_least40_t;    /* c99 types */
typedef int40_t int_fast40_t;   /* c99 types */
typedef uint40_t uint_fast40_t; /* c99 types */
#define  INT40_MAX  0x7fffffffff    /* c99 types */
#define  INT40_MIN  (-INT40_MAX-1)  /* c99 types */
#define UINT40_MAX  0xffffffffff    /* c99 types */
#define  INT_LEAST40_MAX  INT40_MAX /* c99 types */
#define  INT_LEAST40_MIN  INT40_MIN /* c99 types */
#define UINT_LEAST40_MAX  UINT40_MAX    /* c99 types */
#define  INT_FAST40_MAX  INT40_MAX  /* c99 types */
#define  INT_FAST40_MIN  INT40_MIN  /* c99 types */
#define UINT_FAST40_MAX  UINT40_MAX /* c99 types */
#define  INT40_C(value) ((int_least40_t)(value))    /* c99 types */
#define UINT40_C(value) ((uint_least40_t)(value))   /* c99 types */
#endif

#else

#error Please enable C99 compilation (ISO/IEC 9899:1999 standard)

#endif

#define INLINE static inline
#define RESTRICT restrict
#include <stddef.h>
#include <limits.h>
#include <float.h>
#include <iso646.h>

#endif

/* intrinsics */
#include <C6xSimulator.h>

/* various macros */
#ifdef BIG_ENDIAN_HOST
#define _BIG_ENDIAN
#endif
#ifdef LITTLE_ENDIAN_HOST
#define _LITTLE_ENDIAN
#endif
#define IN
#define OUT
#define INOUT
#define _nassert(expr) 

#elif defined (_MSC_VER)
/**********************/
/* Microsoft compiler */ 
/**********************/

#if defined (__cplusplus)
/* c++ specific */

typedef unsigned int uintptr_t; /* c99 types - windows in mostly 32-bit */
typedef int intptr_t;           /* c99 types - windows is mostly 32-bit */
typedef unsigned char uint8_t;  /* c99 types */
typedef unsigned short uint16_t;    /* c99 types */
typedef unsigned int uint32_t;  /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef unsigned __int64 uint40_t;  /* c99 types */
#endif
typedef unsigned __int64 uint64_t;  /* c99 types */
typedef char int8_t;            /* c99 types */
typedef short int16_t;          /* c99 types */
typedef int int32_t;            /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef __int64 int40_t;        /* c99 types */
#endif
typedef __int64 int64_t;        /* c99 types */
#define INLINE static __inline
#define RESTRICT
#include <cstddef>
#include <climits>
#include <ciso646.h>
#include <cfloat.h>

#else
/* c specific */

//typedef unsigned int uintptr_t; /* c99 types - windows in mostly 32-bit */
//typedef int intptr_t;           /* c99 types - windows is mostly 32-bit */
typedef unsigned char uint8_t;  /* c99 types */
typedef unsigned short uint16_t;    /* c99 types */
typedef unsigned int uint32_t;  /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef unsigned __int64 uint40_t;  /* c99 types */
#endif
typedef unsigned __int64 uint64_t;  /* c99 types */
typedef char int8_t;            /* c99 types */
typedef short int16_t;          /* c99 types */
typedef int int32_t;            /* c99 types */
#ifdef __TI_40BIT_LONG__
typedef __int64 int40_t;        /* c99 types */
#endif
typedef __int64 int64_t;        /* c99 types */
typedef unsigned int _Bool;     /* c99 types */
//typedef _Bool bool;             /* c99 types */
//#define true 1
//#define false 0
#define __bool_true_false_are_defined   1
#define INLINE static __inline
#define RESTRICT
#include <stddef.h>
#include <limits.h>
#include <float.h>
#include <iso646.h>

#endif

/* intrinsics */
#include <C6xSimulator.h>

/* various macros */
#ifdef BIG_ENDIAN_HOST
#define _BIG_ENDIAN
#endif
#ifdef LITTLE_ENDIAN_HOST
#define _LITTLE_ENDIAN
#endif
#define IN
#define OUT
#define INOUT
#define _nassert(expr) 

#else

#error Unkwown compiler

#endif

#include <cplx_types.h>

#endif /* _SWPFORM_H_ */

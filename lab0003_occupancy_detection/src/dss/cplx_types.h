/******************************************************************************
 * typedef declarations
 ******************************************************************************/

#ifndef _CPLX_TYPES

#define  _CPLX_TYPES

#ifdef _LITTLE_ENDIAN
typedef struct _CPLX8
{
    int8_t imag;
    int8_t real;
} cplx8_t;

typedef struct _CPLX16
{
    int16_t imag;
    int16_t real;
} cplx16_t;

typedef struct _CPLX32
{
    int32_t imag;
    int32_t real;
} cplx32_t;

typedef struct _CPLXF
{
    float imag;
    float real;
} cplxf_t;
#endif /* _LITTLE_ENDIAN */

#ifdef _BIG_ENDIAN
typedef struct _CPLX8
{
    int8_t real;
    int8_t imag;
} cplx8_t;

typedef struct _CPLX16
{
    int16_t real;
    int16_t imag;
} cplx16_t;

typedef struct _CPLX32
{
    int32_t real;
    int32_t imag;
} cplx32_t;

typedef struct _CPLXF
{
    float real;
    float imag;
} cplxf_t;
#endif /* _BIG_ENDIAN */

typedef union _CPLX32U
{
    cplx32_t cplx32;
    uint64_t realimag;
} cplx32u_t;

typedef union _CPLX16U
{
    cplx16_t cplx16;
    uint32_t realimag;
} cplx16u_t;

#endif /* _CPLX_TYPES */

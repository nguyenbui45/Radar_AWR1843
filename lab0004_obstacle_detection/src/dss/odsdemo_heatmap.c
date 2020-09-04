
#include <math.h>

/* C674x mathlib */
#include <ti/mathlib/mathlib.h>

#include "radar_c674x.h"

#include "../common/odsdemo_common.h"
#include <float.h>
#include <odsdemo_heatmap.h>


/**
 *  \fn     void MATRIX_4x4_BWInversionfp(
 *                            IN      cplxf_t  * RESTRICT Input,
 *                            OUT     cplxf_t  * RESTRICT output);
 *
 *  \brief   4x4 matrix inversion using block wise method.
 *
 *  \param[in]    Input
 *              Input 4x4 matrix that needs to be inversed, stored sequentially
 *              in format (1,1), (1,2).... (1,4), (2,1), (2,2)...(4,4).
 *
 *  \param[out]   output
 *              Output 4x4 matrix. Stored sequentially as the input.
 *
 *  \pre
 *
 *  \post
 *
 *  \sa
 *
 */
void MATRIX_4x4_BWInversionfp(cplxf_t *Input,
                              cplxf_t *output)
{

    float      A0, A3, D0, D3, F0, F3;
    float      detA, oneOverdetA;
    float      ftemp1;
    __float2_t A1, D1, F1, C0, C1, C2, C3;
    __float2_t B0, B1, B2, B3;
    __float2_t dtemp, dtemp1, dtemp2;

    /* load A */
    dtemp           =   _amem8_f2(&Input[0]);
    A0              =   _hif2(dtemp);
    dtemp           =   _amem8_f2(&Input[5]);
    A3              =   _hif2(dtemp);
    A1              =   _amem8_f2(&Input[1]);

    /* Calculate D = inv(D) */
    dtemp           =    _amem8_f2(&Input[10]);
    D0              =   _hif2(dtemp);
    dtemp           =    _amem8_f2(&Input[15]);
    D3              =   _hif2(dtemp);
    D1              =    _amem8_f2(&Input[11]);
    dtemp           =   _dmpysp(D1, D1);
    detA            =   D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
    oneOverdetA     =   _rcpsp( detA );
    oneOverdetA     =   oneOverdetA * (2.f - detA * oneOverdetA);
    oneOverdetA     =   oneOverdetA * (2.f - detA * oneOverdetA);

    ftemp1          =   D0 * oneOverdetA;
    D0              =   D3 * oneOverdetA;
    D3              =   ftemp1;
    D1              =   _dmpysp(D1, _ftof2(-oneOverdetA, -oneOverdetA));

    /* load B */
    B0              =   _amem8_f2(&Input[2]);
    B1              =   _amem8_f2(&Input[3]);
    B2              =   _amem8_f2(&Input[6]);
    B3              =   _amem8_f2(&Input[7]);

    /* calculate C = B*inv(D) */
    //results           =   _cmpysp(D1, B1);
    //C0                =   _dsubsp(_hif2_128(results), _lof2_128(results));
    C0              =   _complex_conjugate_mpysp(D1, B1);
    C0              =   _daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
    //results           =   _cmpysp(D1, B0);
    //C1                =   _daddsp(_hif2_128(results), _lof2_128(results));
    C1              =   _complex_mpysp(D1, B0);
    dtemp1          =   C1;
    C1              =   _daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
    //results           =   _cmpysp(D1, B3);
    //C2                =   _dsubsp(_hif2_128(results), _lof2_128(results));
    C2              =   _complex_conjugate_mpysp(D1, B3);
    C2              =   _daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
    //results           =   _cmpysp(D1, B2);
    //C3                =   _daddsp(_hif2_128(results), _lof2_128(results));
    C3              =   _complex_mpysp(D1, B2);
    dtemp2          =   C3;
    C3              =   _daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));

    /* calculate F = A - B *inv(D) * conj(B) -- Hermitian */
    dtemp           =   _dmpysp(B0, B0);
    F0              =   A0 - D0 * (_hif2(dtemp) + _lof2(dtemp));
    dtemp           =   _dmpysp(B1, B1);
    F0              -=  D3 * (_hif2(dtemp) + _lof2(dtemp));
    //results           =   _cmpysp(B1, dtemp1);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(B1, dtemp1);
    F0              -=  2.f * _hif2(dtemp);

    dtemp           =   _dmpysp(B2, B2);
    F3              =   A3 - D0 * (_hif2(dtemp) + _lof2(dtemp));
    dtemp           =   _dmpysp(B3, B3);
    F3              -=  D3 * (_hif2(dtemp) + _lof2(dtemp));
    //results           =   _cmpysp(B3, dtemp2);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(B3, dtemp2);
    F3              -=  2.f * _hif2(dtemp);

    //results           =   _cmpysp(B2, C0);
    //F1                =   _dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    F1              =   _dsubsp(A1, _complex_conjugate_mpysp(B2, C0));
    //results           =   _cmpysp(B3, C1);
    //F1                =   _dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    F1              =   _dsubsp(F1, _complex_conjugate_mpysp(B3, C1));

    /* Calculate F = inv(F) */
    dtemp           =   _dmpysp(F1, F1);
    detA            =   F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
    oneOverdetA     =   _rcpsp( detA );
    oneOverdetA     =   oneOverdetA * (2.f - detA * oneOverdetA);
    oneOverdetA     =   oneOverdetA * (2.f - detA * oneOverdetA);

    ftemp1          =   F0 * oneOverdetA;
    F0              =   F3 * oneOverdetA;
    F3              =   ftemp1;
    F1              =   _dmpysp(F1, _ftof2(-oneOverdetA, -oneOverdetA));

    /* NW output */
    _amem8_f2(&output[0])   =   _ftof2(F0, 0.f);
    _amem8_f2(&output[1])   =   F1;
    _amem8_f2(&output[5])   =   _ftof2(F3, 0.f);
    //_amem8_f2(&output[4]) =   _lltof2(_f2toll(F1) ^ 0x0000000080000000);
    _amem8_f2(&output[4])   =   _ftof2(_hif2(F1), -_lof2(F1));

    /* NE output = - F * C, SW = conj(NW)*/
    //results           =   _cmpysp(F1, C2);
    //dtemp         =   _daddsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_mpysp(F1, C2);
    dtemp1          =   dtemp;
    dtemp           =   _daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
    _amem8_f2(&output[2])       =   _ftof2(-_hif2(dtemp), -_lof2(dtemp));
    _amem8_f2(&output[8])       =   _ftof2(-_hif2(dtemp), _lof2(dtemp));
    //_amem8_f2(&output[2])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
    //_amem8_f2(&output[8])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

    //results           =   _cmpysp(F1, C3);
    //dtemp         =   _daddsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_mpysp(F1, C3);
    dtemp2          =   dtemp;
    dtemp           =   _daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
    B1              =   dtemp;
    _amem8_f2(&output[3])       =   _ftof2(-_hif2(dtemp), -_lof2(dtemp));
    _amem8_f2(&output[12])      =   _ftof2(-_hif2(dtemp), _lof2(dtemp));
    //_amem8_f2(&output[3])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
    //_amem8_f2(&output[12])    =   _lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

    //results           =   _cmpysp(F1, C0);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(F1, C0);
    dtemp           =   _daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
    _amem8_f2(&output[6])       =   _ftof2(-_hif2(dtemp), -_lof2(dtemp));
    _amem8_f2(&output[9])       =   _ftof2(-_hif2(dtemp), _lof2(dtemp));
    //_amem8_f2(&output[6])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
    //_amem8_f2(&output[9])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

    //results           =   _cmpysp(F1, C1);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(F1, C1);
    dtemp           =   _daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
    B3              =   dtemp;
    _amem8_f2(&output[7])       =   _ftof2(-_hif2(dtemp), -_lof2(dtemp));
    _amem8_f2(&output[13])      =   _ftof2(-_hif2(dtemp), _lof2(dtemp));
    //_amem8_f2(&output[7])     =   _lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
    //_amem8_f2(&output[13])    =   _lltof2(_f2toll(dtemp) ^ 0x8000000000000000);


    /* SE output */
    /* inv(D) - conj(C) * inv(F) * C, whrer C = B * inv(D) */
    dtemp           =   _dmpysp(C0, C0);
    A0              =   D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
    dtemp           =   _dmpysp(C2, C2);
    A0              +=  F3 * (_hif2(dtemp) + _lof2(dtemp));
    //results           =   _cmpysp(C0, dtemp1);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(C0, dtemp1);
    A0              +=  2.f * _hif2(dtemp);

    dtemp           =   _dmpysp(C1, C1);
    A3              =   D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
    dtemp           =   _dmpysp(C3, C3);
    A3              +=  F3 * (_hif2(dtemp) + _lof2(dtemp));
    //results           =   _cmpysp(C1, dtemp2);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(C1, dtemp2);
    A3              +=  2.f * _hif2(dtemp);
    _amem8_f2(&output[10])= _ftof2(A0, 0.f);
    _amem8_f2(&output[15])= _ftof2(A3, 0.f);

    //results           =   _cmpysp(C0, B1);
    //dtemp         =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp           =   _complex_conjugate_mpysp(C0, B1);
    A1              =   _daddsp(D1, dtemp);
    //results           =   _cmpysp(C2, B3);
    //A1                =   _daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    A1              =   _daddsp(A1, _complex_conjugate_mpysp(C2, B3));

    _amem8_f2(&output[11])= A1;
    _amem8_f2(&output[14])= _ftof2(_hif2(A1), -_lof2(A1));
    //_amem8_f2(&output[14])=   _lltof2(_f2toll(A1) ^ 0x0000000080000000);
}


void MATRIX_Mult4x4fp(cplxf_t *A,
                      cplxf_t *B,
                      cplxf_t *C)

{

#if GENERICCODE
    int32_t jj, kk, mm;
    float   tempre, tempim;

    for ( jj = 0; jj < 4; jj++ )
    {
        for ( kk = 0; kk < 4; kk++ )
        {
            tempre      =   0.f;
            tempim      =   0.f;
            for (mm = 0; mm < 4; mm++)
            {
                tempre      +=  A[4 * jj + mm].real * B[4 * mm + kk].real - A[4 * jj + mm].imag * B[4 * mm + kk].imag;
                tempim      +=  A[4 * jj + mm].imag * B[4 * mm + kk].real + A[4 * jj + mm].real * B[4 * mm + kk].imag;
            }
            C[4 * jj + kk].real     =   tempre;
            C[4 * jj + kk].imag     =   tempim;
        }
    }
#else
    int32_t kk, mm;
    __float2_t  * A1, * A2, * A3, * A4;
    __float2_t  * C1, * C2, * C3, * C4;
    __float2_t  dtemp, dtemp1, dtemp2, dtemp3;

    A1              =   (__float2_t *) &A[0];
    A2              =   (__float2_t *) &A[4];
    A3              =   (__float2_t *) &A[8];
    A4              =   (__float2_t *) &A[12];
    C1              =   (__float2_t *) &C[0];
    C2              =   (__float2_t *) &C[4];
    C3              =   (__float2_t *) &C[8];
    C4              =   (__float2_t *) &C[12];
    for ( kk = 0; kk < 4; kk++ )
    {
        dtemp       =   _ftof2(0.f, 0.f);
        dtemp1      =   dtemp;
        dtemp2      =   dtemp;
        dtemp3      =   dtemp;
        #ifdef _TMS320C6x
        #pragma UNROLL(4);
        #endif
        for (mm = 0; mm < 4; mm++)
        {
            dtemp       =   _daddsp(dtemp, _complex_mpysp(_amem8_f2(&A1[mm]), _amem8_f2(&B[4 * mm + kk])));
            dtemp1      =   _daddsp(dtemp1, _complex_mpysp(_amem8_f2(&A2[mm]), _amem8_f2(&B[4 * mm + kk])));
            dtemp2      =   _daddsp(dtemp2, _complex_mpysp(_amem8_f2(&A3[mm]), _amem8_f2(&B[4 * mm + kk])));
            dtemp3      =   _daddsp(dtemp3, _complex_mpysp(_amem8_f2(&A4[mm]), _amem8_f2(&B[4 * mm + kk])));
        }
        _amem8_f2(&C1[kk])  =   dtemp;
        _amem8_f2(&C2[kk])  =   dtemp1;
        _amem8_f2(&C3[kk])  =   dtemp2;
        _amem8_f2(&C4[kk])  =   dtemp3;
    }
#endif
}


/**
 *  \fn     void MATRIX_single8x8MatInv(
 *                            IN      Cplx32  * restrict Input,
 *                            IN      int32_t * RESTRICT scratch,
 *                            OUT     cplx32_t  * Output);
 *
 *  \brief   Single 8x8 matrix (complex positive semi-definitive Hermitian) inversion using block wise method.
 *
 *  \param[in]    Input
 *              Input 8x8 matrix that needs to be inversed, stored sequentially
 *              in format (1,1), (1,2).... (1,8), (2,1), (2,2)...(8,8).
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 7 * 2 * 16 = 288 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   Output
 *              Output 8x8 matrix. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post
 *
 *  \sa
 *
 */
void MATRIX_single8x8MatInv(cplxf_t *Input,
                            int32_t *scratch,
                            cplxf_t *output)
{
    cplxf_t    *A;
    cplxf_t    *B;
    cplxf_t    *C;
    cplxf_t    *D;
    cplxf_t    *T;
    cplxf_t    *invT;
    cplxf_t    *invA;
    int32_t     jj, offset;
    int32_t     scratchIndx;
    __float2_t  dtemp1;


    scratchIndx     =   0;
    A               =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    B               =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    C               =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    D               =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    T               =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    invA            =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;
    invT            =   (cplxf_t *) &scratch[scratchIndx];
    scratchIndx     +=  16 * 2;


    /*Copy A */
    _amem8_f2(&A[0])    =   _amem8_f2(&Input[0]);
    dtemp1              =   _amem8_f2(&Input[1]);
    _amem8_f2(&A[1])    =   dtemp1;
    _amem8_f2(&A[4])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[2]);
    _amem8_f2(&A[2])    =   dtemp1;
    _amem8_f2(&A[8])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[3]);
    _amem8_f2(&A[3])    =   dtemp1;
    _amem8_f2(&A[12])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&A[5])    =   _amem8_f2(&Input[8]);
    dtemp1              =   _amem8_f2(&Input[9]);
    _amem8_f2(&A[6])    =   dtemp1;
    _amem8_f2(&A[9])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[10]);
    _amem8_f2(&A[7])    =   dtemp1;
    _amem8_f2(&A[13])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&A[10])   =   _amem8_f2(&Input[15]);
    dtemp1              =   _amem8_f2(&Input[16]);
    _amem8_f2(&A[11])   =   dtemp1;
    _amem8_f2(&A[14])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&A[15])   =   _amem8_f2(&Input[21]);

    /*copy C */
    offset = 4;
    for ( jj = 0; jj < 4; jj++ )
    {
        dtemp1              =   _amem8_f2(&Input[jj + offset]);
        _amem8_f2(&C[4 * jj])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    }
    offset = 11;
    for ( jj = 0; jj < 4; jj++ )
    {
        dtemp1              =   _amem8_f2(&Input[jj + offset]);
        _amem8_f2(&C[4 * jj + 1])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    }
    offset = 17;
    for ( jj = 0; jj < 4; jj++ )
    {
        dtemp1              =   _amem8_f2(&Input[jj + offset]);
        _amem8_f2(&C[4 * jj + 2])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    }
    offset = 22;
    for ( jj = 0; jj < 4; jj++ )
    {
        dtemp1              =   _amem8_f2(&Input[jj + offset]);
        _amem8_f2(&C[4 * jj + 3])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    }

    /*Copy D */
    _amem8_f2(&D[0])    =   _amem8_f2(&Input[26]);
    dtemp1              =   _amem8_f2(&Input[27]);
    _amem8_f2(&D[1])    =   dtemp1;
    _amem8_f2(&D[4])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[28]);
    _amem8_f2(&D[2])    =   dtemp1;
    _amem8_f2(&D[8])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[29]);
    _amem8_f2(&D[3])    =   dtemp1;
    _amem8_f2(&D[12])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&D[5])    =   _amem8_f2(&Input[30]);
    dtemp1              =   _amem8_f2(&Input[31]);
    _amem8_f2(&D[6])    =   dtemp1;
    _amem8_f2(&D[9])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    dtemp1              =   _amem8_f2(&Input[32]);
    _amem8_f2(&D[7])    =   dtemp1;
    _amem8_f2(&D[13])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&D[10])   =   _amem8_f2(&Input[33]);
    dtemp1              =   _amem8_f2(&Input[34]);
    _amem8_f2(&D[11])   =   dtemp1;
    _amem8_f2(&D[14])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    _amem8_f2(&D[15])   =   _amem8_f2(&Input[35]);



    /* calculate inv(A)*/
    MATRIX_4x4_BWInversionfp (A, invA);


    /* calculate tempC = C*inv(A) */
    //tsc_in            =   clock();
    MATRIX_Mult4x4fp(C, invA, B);
    //tsc_out           =   clock();
    //CycleCounts       =   tsc_out - tsc_in;


    /* calculate T = D - C*inv(A)*B */
    //results           =   _cmpysp(_amem8_f2(&C[0]), _amem8_f2(&B[0]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[0]), _amem8_f2(&B[0]));

    //results           =   _cmpysp(_amem8_f2(&C[1]), _amem8_f2(&B[1]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[1]), _amem8_f2(&B[1])));

    //results           =   _cmpysp(_amem8_f2(&C[2]), _amem8_f2(&B[2]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[2]), _amem8_f2(&B[2])));

    //results           =   _cmpysp(_amem8_f2(&C[3]), _amem8_f2(&B[3]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[3]), _amem8_f2(&B[3])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[0]), dtemp1);
    _amem8_f2(&T[0])    =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&C[4]), _amem8_f2(&B[0]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[4]), _amem8_f2(&B[0]));

    //results           =   _cmpysp(_amem8_f2(&C[5]), _amem8_f2(&B[1]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[5]), _amem8_f2(&B[1])));

    //results           =   _cmpysp(_amem8_f2(&C[6]), _amem8_f2(&B[2]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[6]), _amem8_f2(&B[2])));

    //results           =   _cmpysp(_amem8_f2(&C[7]), _amem8_f2(&B[3]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[7]), _amem8_f2(&B[3])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[1]), dtemp1);
    _amem8_f2(&T[1])    =   dtemp1;
    _amem8_f2(&T[4])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[4])  =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[4]), _amem8_f2(&B[4]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[4]), _amem8_f2(&B[4]));

    //results           =   _cmpysp(_amem8_f2(&C[5]), _amem8_f2(&B[5]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[5]), _amem8_f2(&B[5])));

    //results           =   _cmpysp(_amem8_f2(&C[6]), _amem8_f2(&B[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[6]), _amem8_f2(&B[6])));

    //results           =   _cmpysp(_amem8_f2(&C[7]), _amem8_f2(&B[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[7]), _amem8_f2(&B[7])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[5]), dtemp1);
    _amem8_f2(&T[5])    =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[0]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[0]));

    //results           =   _cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[1]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[1])));

    //results           =   _cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[2]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[2])));

    //results           =   _cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[3]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[3])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[2]), dtemp1);
    _amem8_f2(&T[2])    =   dtemp1;
    _amem8_f2(&T[8])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[8])  =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[4]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[4]));

    //results           =   _cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[5]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[5])));

    //results           =   _cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[6])));

    //results           =   _cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[7])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[6]), dtemp1);
    _amem8_f2(&T[6])    =   dtemp1;
    _amem8_f2(&T[9])    =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[9])  =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[8]), _amem8_f2(&B[8]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[8]), _amem8_f2(&B[8]));

    //results           =   _cmpysp(_amem8_f2(&C[9]), _amem8_f2(&B[9]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[9]), _amem8_f2(&B[9])));

    //results           =   _cmpysp(_amem8_f2(&C[10]), _amem8_f2(&B[10]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[10]), _amem8_f2(&B[10])));

    //results           =   _cmpysp(_amem8_f2(&C[11]), _amem8_f2(&B[11]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[11]), _amem8_f2(&B[11])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[10]), dtemp1);
    _amem8_f2(&T[10])   =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[0]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[0]));

    //results           =   _cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[1]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[1])));

    //results           =   _cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[2]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[2])));

    //results           =   _cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[3]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[3])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[3]), dtemp1);
    _amem8_f2(&T[3])    =   dtemp1;
    _amem8_f2(&T[12])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[12]) =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[4]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[4]));

    //results           =   _cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[5]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[5])));

    //results           =   _cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[6])));

    //results           =   _cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[7])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[7]), dtemp1);
    _amem8_f2(&T[7])    =   dtemp1;
    _amem8_f2(&T[13])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[13]) =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[8]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[8]));

    //results           =   _cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[9]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[9])));

    //results           =   _cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[10]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[10])));

    //results           =   _cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[11]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[11])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[11]), dtemp1);
    _amem8_f2(&T[11])   =   dtemp1;
    _amem8_f2(&T[14])   =   _ftof2(_hif2(dtemp1), -_lof2(dtemp1));
    //_amem8_f2(&T[14]) =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&C[12]), _amem8_f2(&B[12]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&C[12]), _amem8_f2(&B[12]));

    //results           =   _cmpysp(_amem8_f2(&C[13]), _amem8_f2(&B[13]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[13]), _amem8_f2(&B[13])));

    //results           =   _cmpysp(_amem8_f2(&C[14]), _amem8_f2(&B[14]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[14]), _amem8_f2(&B[14])));

    //results           =   _cmpysp(_amem8_f2(&C[15]), _amem8_f2(&B[15]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&C[15]), _amem8_f2(&B[15])));

    dtemp1          =   _dsubsp(_amem8_f2(&D[15]), dtemp1);
    _amem8_f2(&T[15])   =   dtemp1;


    /* Calculate T = inv(T) */
    MATRIX_4x4_BWInversionfp (T, invT);


    /* output SE 4x4 matrix */
    _amem8_f2(&output[26])  =   _amem8_f2(&invT[0]);
    _amem8_f2(&output[27])  =   _amem8_f2(&invT[1]);
    _amem8_f2(&output[28])  =   _amem8_f2(&invT[2]);
    _amem8_f2(&output[29])  =   _amem8_f2(&invT[3]);
    _amem8_f2(&output[30])  =   _amem8_f2(&invT[5]);
    _amem8_f2(&output[31])  =   _amem8_f2(&invT[6]);
    _amem8_f2(&output[32])  =   _amem8_f2(&invT[7]);
    _amem8_f2(&output[33])  =   _amem8_f2(&invT[10]);
    _amem8_f2(&output[34])  =   _amem8_f2(&invT[11]);
    _amem8_f2(&output[35])  =   _amem8_f2(&invT[15]);

    /* output SW and NE 4x4 matrix = - invA*B*invT */
    MATRIX_Mult4x4fp(invT, B, C);
    jj                      =   0;
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 0]);
    _amem8_f2(&output[4])   =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 1]);
    _amem8_f2(&output[11])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 2]);
    _amem8_f2(&output[17])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 3]);
    _amem8_f2(&output[22])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    jj                      =   1;
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 0]);
    _amem8_f2(&output[5])   =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 1]);
    _amem8_f2(&output[12])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 2]);
    _amem8_f2(&output[18])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 3]);
    _amem8_f2(&output[23])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    jj                      =   2;
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 0]);
    _amem8_f2(&output[6])   =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 1]);
    _amem8_f2(&output[13])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 2]);
    _amem8_f2(&output[19])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 3]);
    _amem8_f2(&output[24])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    jj                      =   3;
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 0]);
    _amem8_f2(&output[7])   =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 1]);
    _amem8_f2(&output[14])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 2]);
    _amem8_f2(&output[20])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));
    dtemp1                  =   _amem8_f2(&C[jj * 4 + 3]);
    _amem8_f2(&output[25])  =   _ftof2(-_hif2(dtemp1), _lof2(dtemp1));


#if 0
    zeros       =   _ftof2(0.f, 0.f);
    for ( jj = 0; jj < 4; jj++ )
    {
        dtemp1                              =   _dsubsp(zeros, _amem8_f2(&C[jj * 4 + 0]));
        _amem8_f2(&output[(jj + 4) * 8 + 0])    =   dtemp1;
        _amem8_f2(&output[jj + 4])      =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

        dtemp1                              =   _dsubsp(zeros, _amem8_f2(&C[jj * 4 + 1]));
        _amem8_f2(&output[(jj + 4) * 8 + 1])    =   dtemp1;
        _amem8_f2(&output[jj + 12])     =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

        dtemp1                              =   _dsubsp(zeros, _amem8_f2(&C[jj * 4 + 2]));
        _amem8_f2(&output[(jj + 4) * 8 + 2])    =   dtemp1;
        _amem8_f2(&output[jj + 20])     =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

        dtemp1                              =   _dsubsp(zeros, _amem8_f2(&C[jj * 4 + 3]));
        _amem8_f2(&output[(jj + 4) * 8 + 3])    =   dtemp1;
        _amem8_f2(&output[jj + 28])     =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
    }
#endif

    /* output NW 4x4 matrix = invA + invA*B*invT*C*invA */
    /* output NW 4x4 matrix = A + conj(B)*C */
    //results           =   _cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[0]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[0]));

    //results           =   _cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[4]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[4])));

    //results           =   _cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[8]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[8])));

    //results           =   _cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[12]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[12])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[0]), dtemp1);
    _amem8_f2(&output[0])   =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[1]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[1]));

    //results           =   _cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[5]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[5])));

    //results           =   _cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[9]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[9])));

    //results           =   _cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[13]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[13])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[1]), dtemp1);
    _amem8_f2(&output[1])   =   dtemp1;
    //_amem8_f2(&output[8]) =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[1]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[1]));

    //results           =   _cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[5]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[5])));

    //results           =   _cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[9]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[9])));

    //results           =   _cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[13]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[13])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[5]), dtemp1);
    //_amem8_f2(&output[9]) =   dtemp1;
    _amem8_f2(&output[8])   =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[2]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[2]));

    //results           =   _cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[6])));

    //results           =   _cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[10]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[10])));

    //results           =   _cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[14]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[14])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[2]), dtemp1);
    _amem8_f2(&output[2])   =   dtemp1;
    //_amem8_f2(&output[16])    =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[2]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[2]));

    //results           =   _cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[6])));

    //results           =   _cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[10]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[10])));

    //results           =   _cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[14]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[14])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[6]), dtemp1);
    _amem8_f2(&output[9])   =   dtemp1;
    //_amem8_f2(&output[10])    =   dtemp1;
    //_amem8_f2(&output[17])    =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[2]), _amem8_f2(&C[2]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[2]), _amem8_f2(&C[2]));

    //results           =   _cmpysp(_amem8_f2(&B[6]), _amem8_f2(&C[6]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[6]), _amem8_f2(&C[6])));

    //results           =   _cmpysp(_amem8_f2(&B[10]), _amem8_f2(&C[10]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[10]), _amem8_f2(&C[10])));

    //results           =   _cmpysp(_amem8_f2(&B[14]), _amem8_f2(&C[14]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[14]), _amem8_f2(&C[14])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[10]), dtemp1);
    _amem8_f2(&output[15])  =   dtemp1;
    //_amem8_f2(&output[18])    =   dtemp1;

    //results           =   _cmpysp(_amem8_f2(&B[0]), _amem8_f2(&C[3]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[0]), _amem8_f2(&C[3]));

    //results           =   _cmpysp(_amem8_f2(&B[4]), _amem8_f2(&C[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[4]), _amem8_f2(&C[7])));

    //results           =   _cmpysp(_amem8_f2(&B[8]), _amem8_f2(&C[11]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[8]), _amem8_f2(&C[11])));

    //results           =   _cmpysp(_amem8_f2(&B[12]), _amem8_f2(&C[15]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[12]), _amem8_f2(&C[15])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[3]), dtemp1);
    _amem8_f2(&output[3])   =   dtemp1;
    //_amem8_f2(&output[24])    =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[1]), _amem8_f2(&C[3]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[1]), _amem8_f2(&C[3]));

    //results           =   _cmpysp(_amem8_f2(&B[5]), _amem8_f2(&C[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[5]), _amem8_f2(&C[7])));

    //results           =   _cmpysp(_amem8_f2(&B[9]), _amem8_f2(&C[11]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[9]), _amem8_f2(&C[11])));

    //results           =   _cmpysp(_amem8_f2(&B[13]), _amem8_f2(&C[15]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[13]), _amem8_f2(&C[15])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[7]), dtemp1);
    _amem8_f2(&output[10])  =   dtemp1;
    //_amem8_f2(&output[11])    =   dtemp1;
    //_amem8_f2(&output[25])    =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[2]), _amem8_f2(&C[3]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[2]), _amem8_f2(&C[3]));

    //results           =   _cmpysp(_amem8_f2(&B[6]), _amem8_f2(&C[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[6]), _amem8_f2(&C[7])));

//  results         =   _cmpysp(_amem8_f2(&B[10]), _amem8_f2(&C[11]));
//  dtemp1          =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[10]), _amem8_f2(&C[11])));

//  results         =   _cmpysp(_amem8_f2(&B[14]), _amem8_f2(&C[15]));
//  dtemp1          =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[14]), _amem8_f2(&C[15])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[11]), dtemp1);
    _amem8_f2(&output[16])  =   dtemp1;
    //_amem8_f2(&output[19])    =   dtemp1;
    //_amem8_f2(&output[26])    =   _lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);

    //results           =   _cmpysp(_amem8_f2(&B[3]), _amem8_f2(&C[3]));
    //dtemp1            =   _dsubsp(_hif2_128(results), _lof2_128(results));
    dtemp1          =   _complex_conjugate_mpysp(_amem8_f2(&B[3]), _amem8_f2(&C[3]));

    //results           =   _cmpysp(_amem8_f2(&B[7]), _amem8_f2(&C[7]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[7]), _amem8_f2(&C[7])));

    //results           =   _cmpysp(_amem8_f2(&B[11]), _amem8_f2(&C[11]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[11]), _amem8_f2(&C[11])));

    //results           =   _cmpysp(_amem8_f2(&B[15]), _amem8_f2(&C[15]));
    //dtemp1            =   _daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
    dtemp1          =   _daddsp(dtemp1, _complex_conjugate_mpysp(_amem8_f2(&B[15]), _amem8_f2(&C[15])));

    dtemp1          =   _daddsp(_amem8_f2(&invA[15]), dtemp1);
    _amem8_f2(&output[21])  =   dtemp1;
    //_amem8_f2(&output[27])    =   dtemp1;
}


void ODSDemo_Heatmap_steeringVecGen(ODSDemo_DataPathObj *obj)
{

    uint32_t    i, j;
    double      ftemp1, freal1, fimag1, frealJ, fimagJ;

    // Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    for (i = 0; i < obj->steeringVecSize; i++)
    {
        ftemp1          =   (double) sin((-obj->estAngleRange + (double) i * obj->estAngleResolution) * (double)RADARDEMO_AOAESTBF_PIOVER180);
        freal1          =   (double) cos(-RADARDEMO_AOAESTBF_PI*ftemp1);
        fimag1          =   (double) sin(-RADARDEMO_AOAESTBF_PI*ftemp1);
        frealJ          =   freal1;
        fimagJ          =   fimag1;
        obj->steeringVec[(obj->nAntForHeatmap - 1) * i + 0].real = (float)frealJ;
        obj->steeringVec[(obj->nAntForHeatmap - 1) * i + 0].imag = (float)fimagJ;
        for (j = 2; j < obj->nAntForHeatmap; j++)
        {
            ftemp1      =   frealJ;
            frealJ      =   frealJ * freal1 - fimagJ * fimag1;
            fimagJ      =   ftemp1 * fimag1 + fimagJ * freal1;
            obj->steeringVec[(obj->nAntForHeatmap - 1) * i + j - 1].real = (float)frealJ;
            obj->steeringVec[(obj->nAntForHeatmap - 1) * i + j - 1].imag = (float)fimagJ;
        }
    }
}


void ODSDemo_Heatmap_aoaEstCaponBF(ODSDemo_DataPathObj *obj, float *heatmap)
{
    /*Calculate covariance matrix and invert */
    ODSDemo_Heatmap_aoaEstCaponBF_covInv(
            (uint8_t) (obj->fallBackToConvBFFlag ^ 1),
            (uint8_t) obj->clutterRemovalFlag,
            obj->gamma,
            (int32_t) obj->nAntForHeatmap,
            (int32_t) obj->nChirps,
            (int32_t *) &obj->scratchPad[0],
            (cplx16_t *) obj->xdoaBuf,
            (cplxf_t  *) &obj->invRnMatrix[0]);

    /* Capon beamforming */
    ODSDemo_Heatmap_aoaEstCaponBF_heatmap(
            (uint8_t) (obj->fallBackToConvBFFlag ^ 1),
            (int32_t) obj->nAntForHeatmap,
            (int32_t)  obj->steeringVecSize,
            (cplxf_t *) obj->steeringVec,
            (cplxf_t  *) &obj->invRnMatrix[0],
            (float *) heatmap);
}


/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_covInv
 *
 *   \brief   Per range bin, estimate the covariance matrices from input 1D FFT results, and calculate the inverse of these matrices.
 *
 *   \param[in]    invFlag
 *               Flag to indicate matrix inversion will be performed.
 *               If set to 1, output invRnMatrices will contain inversion of covariance matrices.
 *               If set to 0, output invRnMatrices will contain covariance matrices without inversion.
 *
 *   \param[in]    clutterRmFlag
 *               Flag to indicate clutter removal will be performed if set to 1. Otherwise, disabled.
 *
 *   \param[in]    gamma
 *               Scaling factor for diagnal loading.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size TBD!!!
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void ODSDemo_Heatmap_aoaEstCaponBF_covInv(
         uint8_t   invFlag,
         uint8_t   clutterRmFlag,
         float     gamma,
         int32_t   nRxAnt,
         int32_t   nChirps,
         int32_t  *scratch,
         cplx16_t *inputAntSamples,
         cplxf_t  *invRnMatrices)
{
    int32_t      antIdx, chirpIdx, i, j, scratchOffset, rnIdx;
    cplx16_t    *input1;
    cplx16_t    *input2;
    __float2_t  *Rn;
    int64_t      lltemp, llinput1, llinput2;
    __float2_t   acc, acc1, acc2, acc3, scale2;
    int32_t      itemp1;
    cplxf_t     *invRn;
    float        ftemp;

    scratchOffset = 0;
    Rn            = (__float2_t *) &scratch[scratchOffset];
    scratchOffset = scratchOffset + 2 * nRxAnt * (1 + (nRxAnt>>1));  /*72 32-bit word for 8 antennas*/

    ftemp         = _rcpsp((float)nChirps);
    scale2        = _ftof2(ftemp, ftemp);

    /*Rn estimation */
    if (clutterRmFlag)
    {
        int64_t     mean2;
        int64_t     intAcc;

        for (antIdx = 0; antIdx < nRxAnt; antIdx++)
        {
            input1      =   (cplx16_t *) &inputAntSamples[antIdx * nChirps];
            acc         =   _ftof2(0.f, 0.f);
            acc1        =   _ftof2(0.f, 0.f);
            for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 8)
            {
                intAcc  =   _dsadd2(_amem8(&input1[chirpIdx]), _amem8(&input1[chirpIdx+2]));
                itemp1  =   _sadd2(_hill(intAcc), _loll(intAcc));
                acc     =   _daddsp(acc, _dinthsp(itemp1));
                intAcc  =   _dsadd2(_amem8(&input1[chirpIdx+4]), _amem8(&input1[chirpIdx+6]));
                itemp1  =   _sadd2(_hill(intAcc), _loll(intAcc));
                acc1    =   _daddsp(acc1, _dinthsp(itemp1));
            }
            acc         =   _daddsp(acc, acc1);
            acc         =   _dmpysp(acc, scale2);
            itemp1      =   _dspinth(acc);
            mean2       =   _itoll(itemp1,itemp1);
            for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 4)
            {
                llinput1    =   _amem8(&input1[chirpIdx]);
                _amem8(&input1[chirpIdx])   =   _dssub2(llinput1, mean2);
                llinput1    =   _amem8(&input1[chirpIdx + 2]);
                _amem8(&input1[chirpIdx + 2])   =   _dssub2(llinput1, mean2);
            }
        }
    }

    rnIdx   =   0;
    for (antIdx = 0; antIdx < nRxAnt; antIdx++)
    {
        input1      =   (cplx16_t *) &inputAntSamples[antIdx * nChirps];

        //i = antIdx case
        acc         =   _ftof2(0.f, 0.f);
        acc1        =   _ftof2(0.f, 0.f);
        acc2        =   _ftof2(0.f, 0.f);
        acc3        =   _ftof2(0.f, 0.f);
        for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 8)
        {
            llinput1    =   _amem8(&input1[chirpIdx]);
            itemp1      =   _hill(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _cmpy(_hill(llinput1), itemp1);
            itemp1      =   _loll(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
            acc         =   _daddsp(acc, _dintsp(lltemp));
            llinput1    =   _amem8(&input1[chirpIdx + 2]);
            itemp1      =   _hill(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _cmpy(_hill(llinput1), itemp1);
            itemp1      =   _loll(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
            acc2        =   _daddsp(acc2, _dintsp(lltemp));

            llinput1    =   _amem8(&input1[chirpIdx + 4]);
            itemp1      =   _hill(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _cmpy(_hill(llinput1), itemp1);
            itemp1      =   _loll(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
            acc1        =   _daddsp(acc1, _dintsp(lltemp));
            llinput1    =   _amem8(&input1[chirpIdx + 6]);
            itemp1      =   _hill(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _cmpy(_hill(llinput1), itemp1);
            itemp1      =   _loll(llinput1);
            itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
            lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
            acc3        =   _daddsp(acc3, _dintsp(lltemp));

        }
        acc                         =   _daddsp(acc, acc1);
        acc                         =   _daddsp(acc, acc2);
        acc                         =   _daddsp(acc, acc3);
        acc                         =   _dmpysp(acc, scale2);
        _amem8_f2(&Rn[rnIdx++])     =   _ftof2(_hif2(acc), 0.f);

        for (i = antIdx + 1; i < nRxAnt; i++)
        {
            input2      =   (cplx16_t *) &inputAntSamples[i * nChirps];

            acc         =   _ftof2(0.f, 0.f);
            acc1        =   _ftof2(0.f, 0.f);
            for (chirpIdx = 0; chirpIdx < nChirps; chirpIdx += 4)
            {
                llinput1    =   _amem8(&input1[chirpIdx]);
                llinput2    =   _amem8(&input2[chirpIdx]);
                itemp1      =   _hill(llinput2);
                itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
                lltemp      =   _cmpy(_hill(llinput1), itemp1);
                itemp1      =   _loll(llinput2);
                itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
                lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
                acc         =   _daddsp(acc, _dintsp(lltemp));
                llinput1    =   _amem8(&input1[chirpIdx + 2]);
                llinput2    =   _amem8(&input2[chirpIdx + 2]);
                itemp1      =   _hill(llinput2);
                itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
                lltemp      =   _cmpy(_hill(llinput1), itemp1);
                itemp1      =   _loll(llinput2);
                itemp1      =   _packhl2(itemp1, _ssub2(0, itemp1));
                lltemp      =   _dadd(lltemp, _cmpy(_loll(llinput1), itemp1));
                acc1        =   _daddsp(acc1, _dintsp(lltemp));
            }
            acc                         =   _daddsp(acc, acc1);
            acc                         =   _dmpysp(acc, scale2);
            _amem8_f2(&Rn[rnIdx++])     =   acc;
        }
    }

    if (invFlag)
    {
        /*add diagnal loading */
        j = 0;
        ftemp = 0;
        itemp1 = nRxAnt;
        for (i = 0; i < nRxAnt; i++)
        {
            ftemp   +=  _hif2(_amem8_f2(&Rn[j]));
            j       +=  itemp1;
            itemp1--;
        }
        if (nRxAnt == 8)
            ftemp   *=  0.125f;
        else if (nRxAnt == 4)
            ftemp   *=  0.25f;
        j = 0;
        ftemp *= gamma;
        acc = _ftof2(ftemp, 0.f);
        itemp1 = nRxAnt;
        for (i = 0; i < nRxAnt; i++)
        {
            _amem8_f2(&Rn[j])   =   _daddsp(_amem8_f2(&Rn[j]), acc);
            j       +=  itemp1;
            itemp1--;
        }


        /* matrix inversion */
        if (nRxAnt == 8)
        {
            MATRIX_single8x8MatInv (
                                    (cplxf_t    *) Rn,
                                    &scratch[scratchOffset],
                                    invRnMatrices);
        }
        else if (nRxAnt == 4)
        {
            cplxf_t     * inputRn;
            __float2_t f2temp1;

            inputRn         =   (cplxf_t *)&scratch[scratchOffset]; // size 2 * 16
            invRn           =   (cplxf_t *)&scratch[scratchOffset + 2 * 16]; // size 2 * 16

            rnIdx       =   0;
            for (i = 0; i < nRxAnt; i++)
            {
                for (j = i; j < nRxAnt; j++)
                {
                    _amem8_f2(&inputRn[i * nRxAnt + j]) = _amem8_f2(&Rn[rnIdx++]);
                }
            }
            for (i = 1; i < nRxAnt; i++)
            {
                for (j = 0; j < i; j++)
                {
                    f2temp1     =   _amem8_f2(&inputRn[j * nRxAnt + i]);

                    _amem8_f2(&inputRn[i * nRxAnt + j]) = _ftof2(_hif2(f2temp1), -_lof2(f2temp1));
                }
            }

            MATRIX_4x4_BWInversionfp (
                                inputRn,
                                invRn);
            rnIdx   =   0;
            for (i = 0; i < nRxAnt; i++)
            {
                for (j = i; j < nRxAnt; j++)
                {
                    _amem8_f2(&invRnMatrices[rnIdx++]) = _amem8_f2(&invRn[i * nRxAnt + j]);
                }
            }
        }
    }
    else
    {

        if (nRxAnt == 8)
            rnIdx   =   36;
        else if (nRxAnt == 4)
            rnIdx   =   10;

        for (i = 0; i < rnIdx; i++)
        {
            _amem8_f2(&invRnMatrices[i]) = _amem8_f2(&Rn[i]);
        }
    }
}


/*!
 *   \fn     RADARDEMO_aoaEstCaponBF_heatmap
 *
 *   \brief   Use Capon beamforming to generate range azimuth heatmap per range bin.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed.
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    numAzimuthBins
 *               number of Azimuth bins
 *
 *   \param[in]    steeringVec
 *              steering vector for beamforming.
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size of 360 32-bit words for 8 antennas, and 82 32-bit words for 4 antennas.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    rangeAzimuthHeatMap
 *               Output range azimuth heatmap, in the format of numInputRangeBins by numAzimuthBins
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
void ODSDemo_Heatmap_aoaEstCaponBF_heatmap(uint8_t  bfFlag,
                                           int32_t  nRxAnt,
                                           int32_t  numAzimuthBins,
                                           cplxf_t *steeringVec,
                                           cplxf_t *invRnMatrices,
                                           float   *rangeAzimuthHeatMap)
{
    int32_t     azimIdx;
    __float2_t *steeringVecPtr;
    __float2_t *invRnMatPtr;
    float      *heatMapPtr;
    __float2_t  f2temp, steerVecIn;
    float       output, result;

    if (nRxAnt == 8)
    {
        steeringVecPtr  =   (__float2_t *) &steeringVec[0];
        invRnMatPtr     =   (__float2_t *)&invRnMatrices[0];

        //this may need to change depending on which dimension to search first.
        heatMapPtr      =   (float *) &rangeAzimuthHeatMap[0];
        for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
        {
            output      =   _hif2(_amem8_f2(&invRnMatPtr[0]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[8]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[15]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[21]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[26]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[30]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[33]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[35]));

            f2temp      =   _amem8_f2(&invRnMatPtr[1]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[9]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[16]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[22]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[27]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[31]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[34]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[2]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[10]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[17]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[23]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[28]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[32]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[3]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[11]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[18]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[24]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[29]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[4]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[12]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[19]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[25]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[5]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[13]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[20]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[6]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[14]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[7]);
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            result      =   _rcpsp(output);
            result      =   result * (2.f - output * result);
            result      =   result * (2.f - output * result);
            //result        =   result * result;

            if (!bfFlag) result = output;
            heatMapPtr[azimIdx]     =   result;
        }
    }
    else if (nRxAnt == 4)
    {
        steeringVecPtr  =   (__float2_t *) &steeringVec[0];
        invRnMatPtr     =   (__float2_t *)&invRnMatrices[0];

        //this may need to change depending on which dimension to search first.
        heatMapPtr      =   (float *) &rangeAzimuthHeatMap[0];
        for (azimIdx = 0; azimIdx < numAzimuthBins; azimIdx++ )
        {
            output      =   _hif2(_amem8_f2(&invRnMatPtr[0]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[4]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[7]));
            output      +=  _hif2(_amem8_f2(&invRnMatPtr[9]));

            f2temp      =   _amem8_f2(&invRnMatPtr[1]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[5]));
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[8]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[2]);
            f2temp      =   _daddsp(f2temp, _amem8_f2(&invRnMatPtr[6]));
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            f2temp      =   _amem8_f2(&invRnMatPtr[3]);
            steerVecIn  =   _amem8_f2(steeringVecPtr++);
            f2temp      =   _dmpysp(f2temp, steerVecIn);
            output      +=  2.f * (_hif2(f2temp)  + _lof2(f2temp));

            result      =   _rcpsp(output);
            result      =   result * (2.f - output * result);
            result      =   result * (2.f - output * result);
            //result        =   result * result;

            if (!bfFlag) result = output;
            heatMapPtr[azimIdx]     =   result;
        }
    }
}

/*!
 * \file   odsdemo_clusteringDBscan3D_priv.c
 *
 * \brief   Private functions for DBscan3D clustering module.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "ODSDemo_clusteringDBscan3D_priv.h"

//#define OPTIMIZED_CODE


//! \copydoc ODSDemo_clusteringDBscan3D_findNeighbors2Fixed
uint16_t ODSDemo_clusteringDBscan3D_findNeighbors2Fixed(
                            IN ODSDemo_clusteringDBscan3DPoint3dfxdp *RESTRICT pointArray,
							IN int16_t *RESTRICT speedArray,
                            IN uint16_t point,
                            IN uint16_t *RESTRICT neigh,
                            IN uint16_t numPoints,
                            IN int32_t epsilon2,
                            IN float weight,
                            IN int32_t vFactor,
                            IN char *RESTRICT visited,
                            OUT uint16_t *newCount)
{
#ifndef OPTIMIZED_CODE
    int32_t a,b,c,d;
    uint16_t i;
    int32_t sum, epsilon2_, itemp;

    uint16_t newcount = 0;
    int16_t x = pointArray[point].x;
    int16_t y = pointArray[point].y;
    int16_t z = pointArray[point].z;
    int16_t speed = speedArray[point];

	itemp		=	vFactor;
	if (_abs(speed) < vFactor)
		itemp	=	speed;
    epsilon2_ = (int32_t)(itemp*itemp*weight + epsilon2);

    for(i = 0; i < numPoints; i++)
    {
        if (visited[i] == POINT_UNKNOWN)
        {
            a = pointArray[i].x - x;
            b = pointArray[i].y - y;
            c = pointArray[i].z - z;
            d = speedArray[i] - speed;
            sum = a*a + b*b + c*c + (int32_t)(weight*(float)(d*d));

            if (sum < epsilon2_)
            {
                *neigh++ = i;
                newcount++;
            }
        }
    }
#else

    int32_t		i, pointCntBy8, visited2, index;
	int32_t 	itemp, epsilon2_;
    int32_t		sign, weightFixed;
	int32_t		testInput, itemp0, itemp1;

    uint16_t newcount = 0;
    int16_t speed = speedArray[point];
	
    int64_t refPoint1, lltestPoint1, lltestPoint2, epsilon2Pair, speedQuard, speedRef;
	int64_t 	lltemp1, lltemp2, lltemp3, lltemp4, lldist1, lldist2;
	int64_t * RESTRICT speedPtr;
    int64_t * RESTRICT testPoints;
    int32_t * RESTRICT visitedPointer, visitedUnknown, visitedCheck;
#ifdef _TMS320C6600
	__x128_t  tempRes;
	int64_t  weightPair;
#else
	int64_t  tempResHi, tempResLo;
#endif
	
	
	weightFixed		=	(int32_t) (weight * (float)(1 << (31 - DBSCAN_FIXEDWEIGHTSHIFT)));
#ifdef _TMS320C6600
	weightPair		=	_itoll(weightFixed, 0);
#endif

	itemp			=	_amem4(&pointArray[point]);
    refPoint1   	=   _itoll(itemp, itemp);

	itemp			=	_pack2(speed, speed);
	speedRef		=	_itoll(itemp, itemp);

	itemp		=	vFactor;
	if (_abs(speed) < vFactor)
		itemp	=	speed;

	itemp 			= 	(int32_t)(itemp*itemp*weight + epsilon2);
	epsilon2_		=	itemp;
	epsilon2Pair	=	_itoll(epsilon2_, epsilon2_);

    testPoints  	=   (int64_t *) &pointArray[0];
	speedPtr		=	(int64_t *) &speedArray[0];
    visitedPointer  =   (int32_t *) visited;
    itemp			=   POINT_UNKNOWN | (POINT_UNKNOWN << 8);
    visitedUnknown  =   _pack2(itemp, itemp);
    pointCntBy8 = (int32_t) numPoints >> 3;

#ifdef _TMS320C6X
	#pragma UNROLL(2);
#endif
    for(i = 0; i < 2 * pointCntBy8; i++)
    {
		speedQuard		=	_mem8(speedPtr++);
		lltemp3			=	_dssub2(speedQuard, speedRef);
#ifdef _TMS320C6600
		tempRes			=	_dmpy2(lltemp3, lltemp3);
		lltemp3			=	_dshl(_cmpy32r1(_hi128(tempRes), weightPair), DBSCAN_FIXEDWEIGHTSHIFT);
		lltemp4			=	_dshl(_cmpy32r1(_lo128(tempRes), weightPair), DBSCAN_FIXEDWEIGHTSHIFT);
#else
		tempResHi		=	_mpy2ll(_hill(lltemp3), _hill(lltemp3));
		tempResLo		=	_mpy2ll(_loll(lltemp3), _loll(lltemp3));

		lltemp3 		=	_dshl(_itoll(_mpyhir(weightFixed, _hill(tempResHi)), _mpyhir(weightFixed, _loll(tempResHi))), DBSCAN_FIXEDWEIGHTSHIFT);
		lltemp4 		=	_dshl(_itoll(_mpyhir(weightFixed, _hill(tempResLo)), _mpyhir(weightFixed, _loll(tempResLo))), DBSCAN_FIXEDWEIGHTSHIFT);
		//lltemp3			=	_dshl(_cmpy32r1(tempResHi, weightPair), DBSCAN_FIXEDWEIGHTSHIFT);
		//lltemp4			=	_dshl(_cmpy32r1(tempResLo, weightPair), DBSCAN_FIXEDWEIGHTSHIFT);
#endif
        lltestPoint1    =   _mem8(testPoints++);
        lltestPoint2    =   _mem8(testPoints++);
        lltemp1     	=   _dssub2(refPoint1, lltestPoint1);
        lltemp2     	=   _dssub2(refPoint1, lltestPoint2);
        lldist1     	=   _itoll(_dotp2(_hill(lltemp1), _hill(lltemp1)), _dotp2(_loll(lltemp1), _loll(lltemp1)));
        lldist2     	=   _itoll(_dotp2(_hill(lltemp2), _hill(lltemp2)), _dotp2(_loll(lltemp2), _loll(lltemp2)));

		lldist1			=	_dsadd(lldist1, lltemp4);
		lldist2			=	_dsadd(lldist2, lltemp3);
#if 0
		lldist1     	=   _dssub(lldist1, epsilon2Pair);
        lldist2     	=   _dssub(lldist2, epsilon2Pair);
        lltemp1			=   _itoll(_hill(_dpack2(_hill(lldist2), _loll(lldist2))), _hill(_dpack2(_hill(lldist1), _loll(lldist1))));
        sign			=   _dcmpgt2(_itoll(0, 0), lltemp1);
#else
		lldist1     	=   _dssub(lldist1, epsilon2Pair);
        lldist2     	=   _dssub(lldist2, epsilon2Pair);
		sign			=   _deal(_dcmpgt2(0, lldist2));
		sign			=	(_rotl(sign, 2) | _deal(_dcmpgt2(0, lldist1))) >> 16;
#endif
        visited2		=   _mem4(visitedPointer++);
        visitedCheck	=   _cmpeq4(visited2, visitedUnknown);

        visitedCheck	=  visitedCheck & sign;
        newcount		+=  _bitc4(visitedCheck);

        itemp			=   i * 4;
        if(visitedCheck & 1) *neigh++ = itemp;
        if(visitedCheck & 2) *neigh++ = itemp + 1;
        if(visitedCheck & 4) *neigh++ = itemp + 2;
        if(visitedCheck & 8) *neigh++ = itemp + 3;
    }

    index       =   4 * i ;
    for(i = index; i < (int32_t) numPoints; i++)
    {
		itemp1		=	speedArray[i] - speed;
		itemp1		=	itemp1 * itemp1;
		testInput	=	_amem4(&pointArray[i]);
		itemp0		=	_ssub2(testInput, _loll(refPoint1));
		itemp0		=	_dotp2(itemp0, itemp0);
        itemp0		=	itemp0 + (int32_t)(weight*(float)itemp1);

        if ((visited[i] == POINT_UNKNOWN) && (itemp0 < epsilon2_))
        {
            *neigh++ = i;
            newcount++;
        }
    }

#endif
    *newCount = newcount;

	if (newcount > 0)
		newcount = newcount;
    return newcount;
}


//! \copydoc ODSDemo_clusteringDBscan3D_calcInfoFixed
void ODSDemo_clusteringDBscan3D_calcInfoFixed(
                            IN ODSDemo_clusteringDBscan3DPoint3dfxdp * RESTRICT pointArray,
							IN int16_t * RESTRICT speedArray,
							//IN float  *SNRArray,
							//IN float  *aoaVar,
                            IN uint16_t * RESTRICT neighStart,
                            IN uint16_t * RESTRICT neighLast,
                            OUT ODSDemo_clusteringDBscanReport * RESTRICT report)
{
    int16_t ind, length, member;
    float sumx, sumy, sumz, sumVel, xCenter, yCenter, zCenter, xSize, ySize, zSize, avgVel, temp;
    float lengthInv; //, rangeVar, angleVar, velVar;
    //float maxVel, minVel;

    length = (neighLast - neighStart);

    sumx = 0;
    sumy = 0;
	sumz = 0;
    sumVel = 0;
    for (ind = 0; ind < length; ind++)
    {
        member = neighStart[ind];
        sumx += (float)(pointArray[member].x);
        sumy += (float)(pointArray[member].y);
        sumz += (float)(pointArray[member].z);
        sumVel += (float)(speedArray[member]);
    }
    lengthInv = 1.0/(float)(length);
    xCenter = sumx * lengthInv;
    yCenter = sumy * lengthInv;
    zCenter = sumz * lengthInv;
    avgVel  = sumVel * lengthInv;
    xSize = 0;
    ySize = 0;
	zSize = 0;
    //velVar = 0;
    //rangeVar = 0;
    //angleVar = 0; // in degree
    for (ind = 0; ind < length; ind++)
    {
        member = neighStart[ind];
        temp = (pointArray[member].x - xCenter);
        temp = ((temp > 0)? (temp): (-temp)); //abs
        xSize =(xSize > temp)? (xSize):(temp); //max
        temp = (pointArray[member].y - yCenter);
        temp = ((temp > 0)? (temp): (-temp)); //abs
        ySize = ((ySize > temp)? (ySize):(temp));//max
        temp = (pointArray[member].z - zCenter);
        temp = ((temp > 0)? (temp): (-temp)); //abs
        zSize = ((zSize > temp)? (zSize):(temp));//max

        //temp = (speedArray[member] - avgVel);
        //velVar += temp * temp;

        //range2 = (float)(pointArray[member].x) * (float)(pointArray[member].x) + (float)(pointArray[member].y) * (float)(pointArray[member].y)
        // 		+ (float)(pointArray[member].z) * (float)(pointArray[member].z);
        //temp = range2 * SNRArray[member];
        //rangeVar += temp;

        //angleVar += aoaVar[member] * aoaVar[member];

    }
    report->numPoints = length;
	report->xCenter = (int16_t)(xCenter ); //convert to centermeter in integer
	report->yCenter = (int16_t)(yCenter );  //convert to centermeter in integer
	report->zCenter = (int16_t)(zCenter );  //convert to centermeter in integer
	report->xSize  = (int16_t)(xSize );     //convert to centermeter in integer
	report->ySize  = (int16_t)(ySize );    //convert to centermeter in integer
	report->zSize  = (int16_t)(zSize );    //convert to centermeter in integer
	report->avgVel = (int16_t)(avgVel);
	//report->centerRangeVar = rangeVar * lengthInv; //rangeResolution * rangeResolution * 0.25;
	//report->centerAngleVar = angleVar * DBSCAN_PIOVER180 * DBSCAN_PIOVER180 * lengthInv; // (2 * DBSCAN_PIOVER180)^2
	//report->centerDopplerVar = velVar * lengthInv; //dopplerResolution * dopplerResolution * 0.25;

}


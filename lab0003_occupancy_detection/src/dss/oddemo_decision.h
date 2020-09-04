/**
 *  \file   oddemo_decision.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODDEMO_DECISION
#define ODDEMO_DECISION

extern void ODDemo_Decision_init(void);

extern void ODDemo_Decision_process(uint16_t         pair,
                                    float           *coeffMatrix,
                                    ODDEMO_Feature  *featureVect,
                                    ODDEMO_Decision *decision);

#endif //ODDEMO_DECISION

/**
 *  \file   oddemo_feature.h
 *
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *
 */
#ifndef ODDEMO_FEATURE
#define ODDEMO_FEATURE


void ODDemo_Feature_init(void);

void ODDemo_Feature_extract(uint16_t        zone_pair,
                            float          *azimuthHeatMap,
                            ODDEMO_Feature *feature);

#endif //ODDEMO_FEATURE

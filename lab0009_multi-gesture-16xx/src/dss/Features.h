#ifndef FEATURES_H
#define FEATURES_H

 
typedef struct Features_t_ {
    /* Note: These will point to L3 heap */
    float pWtrange;
    float pWtdoppler[10];
    float pInstenergy;
    int32_t pMaxazfreq[10];
    int32_t pMaxelfreq;
    float pAzdoppcorr;
    uint32_t maxindx_range;
    uint32_t maxindx_doppler;
    uint16_t currindx; // this will hold the cyclic index where the current sample is written for the features
    
    /* Note: parameters below will come from cli in demo application */
    uint16_t rangebin_start;
    uint16_t rangebin_stop;
    uint16_t posdopplerbin_start;
    uint16_t posdopplerbin_stop;
    int16_t  negdopplerbin_start;
    int16_t  negdopplerbin_stop;
    float    detthresh;

} Features_t;

float Computefeatures_RDIBased(Features_t *pfeatures, uint32_t *preDetMatrix, uint32_t indx, uint16_t num_doppler_bins);
void Computefeatures_DOABased(Features_t *pfeatures, cmplx32ReIm_t *DOAIn, uint32_t indx, cmplx32ReIm_t *azimuthTwiddle32x32, uint16_t *log2Abs, cmplx32ReIm_t *DOAOut);
void Computefeatures_Hybrid(Features_t *pfeatures, uint32_t indx);
void mmwavelib_Abs32(const int32_t inp[restrict],
                          uint32_t out[restrict], uint32_t len);

#endif // FEATURES_H_INCLUDED

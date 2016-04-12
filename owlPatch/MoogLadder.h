#ifndef __MoogLadder_h__
#define __MoogLadder_h__

#include "StompBox.h"

enum fType
{
    LPF=0,
    HPF
};

class MoogLadder {
public:
    MoogLadder();
    ~MoogLadder();
    fType type ;
    void setType(fType t) ;
    void setCoeffs(float w0) ; // for LPF, HPF
    void process(int numSamples, float* buffer, float w0, float res, float drive, float masterGain) ;
    float processLadder(float input,float x1, float y1);
    void setMutiplexer();
    float nonLinear(float x); // tanh approx

private:
    float A, B, C, D, E ; // multiplexer coeffs
    float a1, b0, b1 ; // one pole section coeffs
    float pw0; // previous w0
    float pres; // previous res
    float pmasterGain; // previous master Gain
    float in1, out1, out2, out3, out4;
    float comp;
};

#endif // __MoogLadder_h__

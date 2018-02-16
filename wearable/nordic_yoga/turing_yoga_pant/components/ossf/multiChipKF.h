/*
 * multichipKF.h
 *
 *  Created on: jan 10, 2017
 *      Author: Gabriele
 *
 */

#ifndef MULTICHIPKF_H_
#define MULTICHIPKF_H_

#include "KF.h"



#if 1
typedef struct KalmanFilterStructStatic {
    
    // Kalman matrices
    float xproj[XDIM][1];
    float Xproj[XDIM][XDIM];
    float xupdt[XDIM][1];
    float Xupdt[XDIM][XDIM];

    float H[YDIM][XDIM];
    float R[YDIM][YDIM];
    float K[XDIM][YDIM];

    float GAMMA[XDIM][XDIM];

    float yproj[YDIM][1];
    float inov[YDIM][1];

    // constant quantities
    float normF;
    
    KF_params * KFpar;
    
    float gyrP[3][1];
    
    bool allocated;

} KF_static;
#endif

typedef struct referenceFrameStatic {

    // vertical and horizontal in the navigation frame (global)
    float gN[3][1];
    float hN[3][1];

    // vertical and horizontal in the navigation frame (global)
    float gb[3][1];
    float hb[3][1];

    float Mref[3][3];

    bool initialized;
    bool allocated;

    } refFrame_static;
#if 0
		
void setKFStaticToZero(KF_static * KFs);

void setRefFrameStaticToZero(refFrame_static * UVRHs);

void loadKFandRefFrame(KF_static * KFs, refFrame_static * UVRHs);

void saveKFandRefFrame(KF_static * KFs, refFrame_static * UVRHs);
#endif
#endif //MULTICHIPKF_H_

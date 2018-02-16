/*
 * KF.h
 *
 *  Created on: 27/gen/2015
 *      Author: Gabriele
 */

#ifndef KF_H_
#define KF_H_

#include "build.h"
#include "matrixComputations.h"
#include "stdbool.h"

#define XDIM 6
#define YDIM 3

// Sampling time
#define TCs (float) 1/60 // TC computed by the KF for computation
#define TCs2 (float) 1/60 // REAL freq we want to get

#define SET_MATRIX_VALUE(M,r,c,v) M[r][c] = v;

//TODO: allocate memory???

typedef struct KalmanFilterParams {
    
    // gyro and acc/mag st devs
    float stdg;
    float stds;
    
    // filter tuning
    float ca;
    float cb;
    
    //initial covariance
    float po;
    
    // state and meas dimension
    int Xdim;
    int Ydim;
    
} KF_params;

typedef struct KalmanFilterStruct {
    
    // Kalman matrices
    float ** xproj;
    float ** Xproj;
    float ** xupdt;
    float ** Xupdt;

    float ** H;
    float ** R;
    float ** K;

    float **GAMMA;

    float ** yproj;
    float ** inov;

    // constant quantities
    float normF;
    
    KF_params * KFpar;
    
    float **gyrP;
    
    bool allocated;

    } KF;

// Alloc KF
void KF_alloc(KF * thisKF);

// Initialize KF
void KF_init(KF * thisKF, KF_params *thisKFpar, float ** g0,float ** gyr);

// DeInit the global variables
void KF_deInit(KF * thisKF);

// Kalman Propagation
// Skew antisymmetric Matrix, see Shuster pag 445, eqs. 44 & 46
void skewAntiSymm(float **w,float **W);

// Projection Matrix: Exact exponential matrix
void projectionMatrix(float** w, float** W);

// Whole Prediction matrix
void KF_predictionMatrix(KF * thisKF, float **PM);

// Uncertainty prediction matrix
void KF_uncertaintyPredictionMatrix(KF * thisKF, float **PP);

// KF prediction
void KF_predict(KF * thisKF);

// Kalman Measurement update
void KF_update(KF * thisKF, float ** meas);

// Kalman gain computation
void KF_computeK(KF * thisKF);

// Uncertainty update
void KF_computeXupdt(KF * thisKF);

#endif /* KF_H_ */

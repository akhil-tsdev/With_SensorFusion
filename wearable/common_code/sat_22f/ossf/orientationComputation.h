/*
 * KF.h
 *
 *  Created on: 27/gen/2015
 *      Author: Gabriele
 */

#ifndef ORIENTATIONCOMPUTATION_H_
#define ORIENTATIONCOMPUTATION_H_

#include "stdbool.h"
#include "build.h"

typedef struct referenceFrame {
    
    // vertical and horizontal in the navigation frame (global)
    float ** gN;
    float ** hN;
    
    // vertical and horizontal in the navigation frame (global)    
    float ** gb;
    float ** hb;

    float **Mref;
    
    bool initialized;
    bool allocated;
    
} refFrame;


int sgn(float val);



void tiltAngles(float **v, float * pitch, float *roll);



void eaZYXToQuat(float **q, float yaw, float pitch, float roll);



void eaZYXToRotMatrix(float **R, float yaw, float pitch, float roll);



void quatToRotMatrix(float **q, float **R);



void rotateVectorRotMatrix(float **R, float ** vin, float ** vout);


// rotate a vector - quaternion
void rotateVectorQuat(float ** q, float ** vin, float ** vout);


// Project ahead a 3x1 vector given the angular rate data
void projVector(float** w, float** v, float** vproj);


void rotMatrixToQuaternion(float ** Rbn, float *qbn);

void allocRefFrame(refFrame * thisRefFrame);

void initRefFrame(refFrame * thisRefFrame);

void deInitRefFrame(refFrame * thisRefFrame);

void Triad(refFrame * thisRefFrame, float * qbn);

void fRunAccGyrSF(KF * thisKF, KF_params * thisKFpar, int16_t *accel, int16_t *gyro, refFrame* thisRefFrame, float qbn[4]);

#endif /* ORIENTATIONCOMPUTATION_H */

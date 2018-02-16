/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <math.h>
#include <stdlib.h>

#include "build.h"
#include "matrixComputations.h"
#include "KF.h"
#include "orientationComputation.h"

void initRefFrame(refFrame * thisRefFrame)
{
    // allocations
	if (thisRefFrame->allocated == false)
	{

	    thisRefFrame->hN = matrixAllocFloat(3,1);
	    thisRefFrame->gN = matrixAllocFloat(3,1);

	    // Mref
	    thisRefFrame->Mref = matrixAllocFloat(3,3);

	    // Alloc hb and gb
	    thisRefFrame->hb = matrixAllocFloat(3,1);
	    thisRefFrame->gb = matrixAllocFloat(3,1);

		thisRefFrame->allocated = true;

	}

	// Initialization
    float ** n2 = matrixAllocFloat(3,1);
    float ** n3 = matrixAllocFloat(3,1);
    float ** SA = matrixAllocFloat(3,3);
    
    // Init (define) global reference frame
    SET_MATRIX_VALUE(thisRefFrame->hN,0,0,1);
    SET_MATRIX_VALUE(thisRefFrame->hN,1,0,0);
    SET_MATRIX_VALUE(thisRefFrame->hN,2,0,0);

    SET_MATRIX_VALUE(thisRefFrame->gN,0,0,0);  
    SET_MATRIX_VALUE(thisRefFrame->gN,1,0,0);
    SET_MATRIX_VALUE(thisRefFrame->gN,2,0,1);

    skewAntiSymm(thisRefFrame->gN,SA);
    matMult(SA,thisRefFrame->hN,n2,3,3,3,1);
    normalizeVectorColumn(n2,n2,3);
    
    matMult(SA,n2,n3,3,3,3,1);
    
    // Mref
    matrixSubBlock(thisRefFrame->Mref ,thisRefFrame->gN,0,0,3,1);
    matrixSubBlock(thisRefFrame->Mref ,n2,0,1,3,1);
    matrixSubBlock(thisRefFrame->Mref ,n3,0,2,3,1);
    
    thisRefFrame->initialized = true;

    matrixFreeFloat(n2,3);
    matrixFreeFloat(n3,3);
    matrixFreeFloat(SA,3);
}

void deInitRefFrame(refFrame * thisRefFrame)
{
    // Init (define) global reference frame
    matrixFreeFloat(thisRefFrame->gN,3);
    matrixFreeFloat(thisRefFrame->hN,3);
    
    matrixFreeFloat(thisRefFrame->Mref,3);
    
    matrixFreeFloat(thisRefFrame->gb,3);
    matrixFreeFloat(thisRefFrame->hb,3);
    
    
    thisRefFrame->allocated = false;
    thisRefFrame->initialized = false;

}


void fRunAccGyrSF(KF * thisKF, KF_params * thisKFpar, int16_t *accel, int16_t *gyro, refFrame* thisRefFrame, float qbn[4])
{
	uint8_t i;

	float ** gyr = matrixAllocFloat(3,1);
	float ** acc = matrixAllocFloat(3,1);
	//float ** qbnTemp = matrixAllocFloat(4,1);

	// process the HAL-aligned sensor measurements prior to calling the sensor fusion
	for (i = X; i <= Z; i++)
	{
		// calculate the fast accelerometer reading for the Kalman filters (to reduce phase errors)
		acc[i][0] = (float) ((int16_t) accel[i]) * 0.00048828125;

		// calculate the fast magnetometer reading for the Kalman filters (to reduce phase errors)
		//thisMag.fBpFast[i] = (float) mag[i] * thisMag.fuTPerCount;MPU9250_UTPERCOUNT0.15F

		// Fast gyro readings, RAD/s, (there is no fYpFast field in GyroSensor struct)
		gyr[i][0] = (float) ((int16_t) gyro[i])*0.001064225153655;
                                                        
	}

   if(thisRefFrame->initialized == false)
    {
        float pitch, roll;
        float **RTilt = matrixAllocFloat(3,3);
        
        // init global refFrame
        initRefFrame(thisRefFrame);
        
        //KF init
        normalizeVectorColumn(acc,acc,3);

        KF_init(thisKF,thisKFpar,acc,gyr);
        matrixCopy(acc,thisRefFrame->gb,3,1);
        
        // init horizontal reference
        tiltAngles(acc,&pitch,&roll);
        eaZYXToRotMatrix(RTilt,0.0,pitch,roll);
        rotateVectorRotMatrix(RTilt,thisRefFrame->hN,thisRefFrame->hb);
        
        matrixFreeFloat(RTilt,3);
     
    } 
    else
    {
        float ** hproj = matrixAllocFloat(3,1);

        // gravity reference acc + gyr
        KF_predict(thisKF); 
        KF_update(thisKF,acc);
        matrixCopy(thisKF->xupdt,thisRefFrame->gb,3,1);
        

        // horizontal reference (gyr only)
        projVector(thisKF->gyrP,thisRefFrame->hb,hproj);
        matrixCopy(hproj,thisRefFrame->hb,3,1);
        matrixCopy(gyr, thisKF->gyrP,3,1);
        
        //printf("%f %f %f %f %f %f\n",gyr[0][0],gyr[1][0],gyr[2][0],thisRefFrame->hb[0][0],thisRefFrame->hb[1][0],thisRefFrame->hb[2][0]);
        matrixFreeFloat(hproj,3);
    }
    
    // Compute orientation
    Triad(thisRefFrame,qbn);

    matrixFreeFloat(gyr,3);
    matrixFreeFloat(acc,3);

}


int sgn(float val)
{
    if (val > 0) return 1;
    if (val < 0) return -1;
    return 0;
}



 void tiltAngles(float **v, float * pitch, float *roll)
 {
    float ** vn = matrixAllocFloat(3,1);
     
    normalizeVectorColumn(v,vn,3);
    
    *pitch = atan2f(-vn[0][0],sqrt(pow(vn[1][0],2)+pow(vn[2][0],2)));
    
    *roll = atan2f(vn[1][0],sgn(vn[2][0])*sqrt(pow(vn[2][0],2)+0.001*pow(vn[0][0],2)));
    
    matrixFreeFloat(vn,3);
 }

 void eaZYXToQuat(float **q, float yaw, float pitch, float roll)
 {
    float sy = sin(yaw*0.5);
    float cy = cos(yaw*0.5);
    float sp = sin(pitch*0.5);
    float cp = cos(pitch*0.5);
    float sr = sin(roll*0.5);
    float cr = cos(roll*0.5);
    
    q[0][0] = cr*cp*cy + sr*sp*sy;
    q[1][0] = sr*cp*cy - cr*sp*sy;
    q[2][0] = cr*sp*cy + sr*cp*sy;
    q[3][0] = cr*cp*sy + sr*sp*cy;
     
 }
 void eaZYXToRotMatrix(float **R, float yaw, float pitch, float roll)
 {
    float sy = sin(yaw);
    float cy = cos(yaw);
    float sp = sin(pitch);
    float cp = cos(pitch);
    float sr = sin(roll);
    float cr = cos(roll);

    SET_MATRIX_VALUE(R,0,0,cp*cy);
    SET_MATRIX_VALUE(R,0,1,cp*sy);
    SET_MATRIX_VALUE(R,0,2,-sp);
    SET_MATRIX_VALUE(R,1,0,sr*sp*cy - cr*sy);
    SET_MATRIX_VALUE(R,1,1,sr*sp*sy + cr*cy);
    SET_MATRIX_VALUE(R,1,2,sr*cp);
    SET_MATRIX_VALUE(R,2,0,cr*sp*cy + sr*sy);
    SET_MATRIX_VALUE(R,2,1,cr*sp*sy - sr*cy);
    SET_MATRIX_VALUE(R,2,2,cr*cp);
   
 }

void quatToRotMatrix(float **q, float **R)
{
    float Q00 = q[0][0]*q[0][0];
    float Q11 = q[1][0]*q[1][0];
    float Q22 = q[2][0]*q[2][0];
    float Q33 = q[3][0]*q[3][0];
    
    R[0][0] = Q00 + Q11 - Q22 - Q33;
    R[0][1] = 2*(q[1][0]*q[2][0] + q[0][0]*q[3][0]);
    R[0][2] = 2*(q[1][0]*q[3][0] - q[0][0]*q[2][0]);
    R[1][0] = 2*(q[1][0]*q[2][0] - q[0][0]*q[3][0]);
    R[1][1] = Q00 - Q11 + Q22 - Q33;
    R[1][2] = 2*(q[2][0]*q[3][0] + q[0][0]*q[1][0]);
    R[2][0] = 2*(q[1][0]*q[3][0] + q[0][0]*q[2][0]);
    R[2][1] = 2*(q[2][0]*q[3][0] - q[0][0]*q[1][0]);
    R[2][2] = Q00 - Q11 - Q22 + Q33;
    

}


// rotate a vector - rot matrix
void rotateVectorRotMatrix(float **R, float ** vin, float ** vout)
{
    int i,j;
    
    for (i = 0; i < 3; i++)
        vout[i][0] = 0;
    
    for (i = 0; i < 3; i++)
        for(j = 0; j < 3; j++)
            vout[i][0] += R[i][j]*vin[j][0];
}

// rotate a vector - quaternion
void rotateVectorQuat(float ** q, float ** vin, float ** vout)
{
    float ** R = matrixAllocFloat(3,3);
    
    quatToRotMatrix(q,R);
    
    rotateVectorRotMatrix(R,vin,vout);
    
    matrixFreeFloat(R,3);
}


// Project ahead a 3x1 vector given the angular rate data
void projVector(float** w, float** v, float** vproj)
{
  float ** PROJM = matrixAllocFloat(3,3);
  setMatrixZeros(PROJM,3,3);

  projectionMatrix(w,PROJM);

  matMult(PROJM,v,vproj,3,3,3,1);
  normalizeVectorColumn(vproj,vproj,3);
  
  matrixFreeFloat(PROJM,3);

}

// Triad algoritm: given g and h defined in {b} and {n} returns qbn (from n to b)
// the input vector must have unitary norm
void Triad(refFrame * thisRefFrame, float * qbn)
{
    float ** b2 = matrixAllocFloat(3,1);
    float ** b3 = matrixAllocFloat(3,1);

    float ** Mobs = matrixAllocFloat(3,3);
    float ** Rbn = matrixAllocFloat(3,3);

    crossProduct3D(thisRefFrame->gb,thisRefFrame->hb,b2);
    normalizeVectorColumn(b2,b2,3);
    crossProduct3D(thisRefFrame->gb,b2,b3);

    
    // Mobs
    matrixSubBlock(Mobs,thisRefFrame->gb,0,0,3,1);
    matrixSubBlock(Mobs,b2,0,1,3,1);
    matrixSubBlock(Mobs,b3,0,2,3,1);

    
    // Compute orientation matrix
    matMultTransp(thisRefFrame->Mref,Mobs,Rbn,3,3,3,3);
    
    // convert to quaternion
    rotMatrixToQuaternion(Rbn,qbn);
    
    // Free 
    matrixFreeFloat(b2,3);
    matrixFreeFloat(b3,3);
    

    matrixFreeFloat(Mobs,3);
    matrixFreeFloat(Rbn,3);

}



void rotMatrixToQuaternion(float ** Rbn, float *qbn)
{

    float m11 = Rbn[0][0];
    float m21 = Rbn[1][0];
    float m31 = Rbn[2][0];
    
    float m12 = Rbn[0][1];
    float m22 = Rbn[1][1];
    float m32 = Rbn[2][1];
    
    float m13 = Rbn[0][2];
    float m23 = Rbn[1][2];
    float m33 = Rbn[2][2];
    
    float x,y,w,z;


    // Determine which of w,x,y, or z has the largest absolute value
    float fourWSquaredMinus1 = m11 + m22 + m33;
    float fourXSquaredMinus1 = m11 - m22 - m33;
    float fourYSquaredMinus1 = m22 - m11 - m33;
    float fourZSquaredMinus1 = m33 - m11 - m22;

            int biggestIndex = 0;
            float fourBiggestSquaredMinus1 = fourWSquaredMinus1;

            if(fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourXSquaredMinus1;
                biggestIndex = 1;
            }
            if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourYSquaredMinus1;
                biggestIndex = 2;
            }
            if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
                fourBiggestSquaredMinus1 = fourZSquaredMinus1;
                biggestIndex = 3;
            }
            // Per form square root and division
            float biggestVal = sqrt (fourBiggestSquaredMinus1 + 1.0f ) * 0.5f;
            float mult = 0.25f / biggestVal;

            // Apply table to compute quaternion values
            switch (biggestIndex) {
                case 0:
                    w = biggestVal;
                    x = (m23 - m32) * mult;
                    y = (m31 - m13) * mult;
                    z = (m12 - m21) * mult;
                    break;
                case 1:
                    x = biggestVal;
                    w = (m23 - m32) * mult;
                    y = (m12 + m21) * mult;
                    z = (m31 + m13) * mult;
                    break;
                case 2:
                    y = biggestVal;
                    w = (m31 - m13) * mult;
                    x = (m12 + m21) * mult;
                    z = (m23 + m32) * mult;
                    break;
                case 3:
                    z = biggestVal;
                    w = (m12 - m21) * mult;
                    x = (m31 + m13) * mult;
                    y = (m23 + m32) * mult;
                    break;



                }
                                
            qbn[0] = -w;
            qbn[1] = x;
            qbn[2] = y;
            qbn[3] = z;

} 



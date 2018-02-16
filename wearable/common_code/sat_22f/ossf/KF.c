/*
 * KF.c
 *
 *  Created on: 27/gen/2015
 *      Author: Gabriele
 */

#include "KF.h"
#include <math.h>


void KF_alloc(KF * thisKF)
{

	  // Allocations
	  if (thisKF->allocated == false)
	  {
		// predicted state
	    thisKF->xproj = matrixAllocFloat(thisKF->KFpar->Xdim,1);

	    // predicted state covariance
	    thisKF->Xproj = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

	    // Updated state
	    thisKF->xupdt = matrixAllocFloat(thisKF->KFpar->Xdim,1);

	    // Updated State covariance
	    thisKF->Xupdt = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

	    // Measurement model
	    thisKF->H = matrixAllocFloat(thisKF->KFpar->Ydim,thisKF->KFpar->Xdim);

	    // Gamma Noise matrix
	    thisKF->GAMMA = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

	    // Kalman gain
	    thisKF->K = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Ydim);

	    // Measurement Noise
	    thisKF->R = matrixAllocFloat(thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);

	    // predicted measuyrement
	    thisKF->yproj = matrixAllocFloat(thisKF->KFpar->Ydim,1);

	    // Innovation
	    thisKF->inov = matrixAllocFloat(thisKF->KFpar->Ydim,1);

	    // previous gyr meas
	    thisKF->gyrP = matrixAllocFloat(3,1);

	    thisKF->allocated = true;
	  }

}


void KF_init(KF * thisKF, KF_params *thisKFpar, float ** g0,float ** gyr)
{
  // init KF params
  thisKF->KFpar = thisKFpar; 
    
  // Identity matrix
  float **IDM = matrixAllocFloat(3,3);
  setMatrixIdentity(IDM,3);

  //It will alloc memory only the first time
  KF_alloc(thisKF);

  // Initializations

  // predicted state and covariance
  setMatrixZeros(thisKF->xproj,thisKF->KFpar->Xdim,1);
  setMatrixZeros(thisKF->Xproj,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
  // Updated state and covariance
  setMatrixZeros(thisKF->xupdt,thisKF->KFpar->Xdim,1);
  matrixSubBlock(thisKF->xupdt,g0,0,0,3,1);
  setMatrixIdentity(thisKF->Xupdt,thisKF->KFpar->Xdim);
  matMultConstant(thisKF->Xupdt,thisKF->KFpar->po,thisKF->Xupdt,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

  // Jacobian
  setMatrixZeros(thisKF->H,thisKF->KFpar->Ydim,thisKF->KFpar->Xdim);
  matrixSubBlock(thisKF->H,IDM,0,0,3,3);
  matrixSubBlock(thisKF->H,IDM,0,3,3,3);
  
  // norm of the reference vector
  thisKF->normF = normVectorColumn(g0,3);
  
  // Gamma Noise matrix
  setMatrixZeros(thisKF->GAMMA,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  matrixSubBlock(thisKF->GAMMA,IDM,3,3,3,3);
  matMultConstant(IDM,pow(thisKF->KFpar->stdg*TCs,2),IDM,3,3);
  matrixSubBlock(thisKF->GAMMA,IDM,0,0,3,3);

  // Kalman gain
  setMatrixZeros(thisKF->K,thisKF->KFpar->Xdim,thisKF->KFpar->Ydim);

  // Measurement noise matrix
  setMatrixIdentity(thisKF->R,thisKF->KFpar->Ydim);
  matMultConstant(thisKF->R,pow(thisKF->KFpar->stds,2),thisKF->R,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);
    
  // Predicted measurements
  setMatrixZeros(thisKF->yproj,thisKF->KFpar->Ydim,1);
  
  // Innovations
  setMatrixZeros(thisKF->inov,thisKF->KFpar->Ydim,1);

  // Previous angular velocity
  matrixCopy(gyr,thisKF->gyrP,3,1);
  
  matrixFreeFloat(IDM,3);


}

void KF_deInit(KF * thisKF)
{
  matrixFreeFloat(thisKF->xproj,thisKF->KFpar->Xdim);
  matrixFreeFloat(thisKF->Xproj,thisKF->KFpar->Xdim);
  
  matrixFreeFloat(thisKF->xupdt,thisKF->KFpar->Xdim);
  matrixFreeFloat(thisKF->Xupdt,thisKF->KFpar->Xdim);
    
  matrixFreeFloat(thisKF->H,thisKF->KFpar->Ydim);
  
  matrixFreeFloat(thisKF->GAMMA,thisKF->KFpar->Xdim);
  
  matrixFreeFloat(thisKF->K,thisKF->KFpar->Xdim);
  matrixFreeFloat(thisKF->R,thisKF->KFpar->Ydim);
  
  matrixFreeFloat(thisKF->yproj,thisKF->KFpar->Ydim);
  matrixFreeFloat(thisKF->inov,thisKF->KFpar->Ydim);

  matrixFreeFloat(thisKF->gyrP,3);  
}

// Skew antisymmetric Matrix, see Shuster pag 445, eqs. 44 & 46
void skewAntiSymm(float **w,float **W)
{

  setMatrixZeros(W,3,3);  
    
  // First row
  SET_MATRIX_VALUE(W,0,1,-w[2][0]);
  SET_MATRIX_VALUE(W,0,2,w[1][0]);

  // Second row
  SET_MATRIX_VALUE(W,1,0,w[2][0]);
  SET_MATRIX_VALUE(W,1,2,-w[0][0]);

  // Third row
  SET_MATRIX_VALUE(W,2,0,-w[1][0]);
  SET_MATRIX_VALUE(W,2,1,w[0][0]);

}

// Projection Matrix: 3x3 exponential matrix
void projectionMatrix(float ** w, float ** EXPW)
{
  float sinc_coeff, cos_coeff;
  uint8_t i;

  float ** SA2 = matrixAllocFloat(3,3);
  float ** SA = matrixAllocFloat(3,3);
  
  float wnormT2 = normVectorColumn(w,3)*TCs;

  skewAntiSymm(w,SA);
   // Note the - sign
  matMultConstant(SA,-TCs,SA,3,3);

  matMult(SA,SA,SA2,3,3,3,3);


  if (wnormT2 < pow(10,-20))
  {
      sinc_coeff = 1;
      cos_coeff = 0;
  }
  else
  {
    sinc_coeff = sin(wnormT2)/(wnormT2);
    cos_coeff = (1-cos(wnormT2))/(pow(wnormT2,2));
  }
  
  matMultConstant(SA,sinc_coeff,SA,3,3);
  matMultConstant(SA2,cos_coeff,SA2,3,3);

  matrixCopy(SA,EXPW,3,3);
  for (i = 0; i < 3; i++)
    SET_MATRIX_VALUE(EXPW,i,i,1.0);

  matAdd(EXPW,SA2,EXPW,3,3);

  matrixFreeFloat(SA2,3);
  matrixFreeFloat(SA,3);

  
}


void KF_predictionMatrix(KF * thisKF, float **PM)
{
  uint8_t i,j;
  setMatrixZeros(PM,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
  projectionMatrix(thisKF->gyrP,PM);  

  
  SET_MATRIX_VALUE(PM,3,3,thisKF->KFpar->ca);
  SET_MATRIX_VALUE(PM,4,4,thisKF->KFpar->ca);
  SET_MATRIX_VALUE(PM,5,5,thisKF->KFpar->ca);
  
}


void KF_uncertaintyPredictionMatrix(KF * thisKF, float **PP)
{
  
  setMatrixZeros(PP,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
  skewAntiSymm(thisKF->xupdt,PP);


  SET_MATRIX_VALUE(PP,3,3,thisKF->KFpar->cb);
  SET_MATRIX_VALUE(PP,4,4,thisKF->KFpar->cb);
  SET_MATRIX_VALUE(PP,5,5,thisKF->KFpar->cb);

  
}


// Kalman Propagation
void KF_predict(KF * thisKF)
{

  // State prediction matrix
  float ** TMP = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  KF_predictionMatrix(thisKF, TMP);
  
  // KF prediction computations
  // Predicted state
  matMult(TMP,thisKF->xupdt,thisKF->xproj,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,1);
  
  // Predicted Covariance
  float ** temp2b = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

  // temp1b = PM*thisKF->Xupdt*PM'
   matMultTranspABAt(TMP,thisKF->Xupdt,thisKF->Xproj,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
   // State noise Matrices
   KF_uncertaintyPredictionMatrix(thisKF,TMP);

  // temp2b = PP*thisKF->GAMMA*PP'
   matMultTranspABAt(TMP,thisKF->GAMMA,temp2b,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
  //thisKF->Xproj = PM*thisKF->Xupdt*PM + PP*thisKF->GAMMA*PP'
  matAdd(thisKF->Xproj,temp2b,thisKF->Xproj,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

  // Predicted measurements- It would be thisKF->yproj = thisKF->H*thisKF->xproj but writing it explictly I avoid a lot of multiplications
  SET_MATRIX_VALUE(thisKF->yproj,0,0,thisKF->xproj[0][0] + thisKF->xproj[3][0]);
  SET_MATRIX_VALUE(thisKF->yproj,1,0,thisKF->xproj[1][0] + thisKF->xproj[4][0]);
  SET_MATRIX_VALUE(thisKF->yproj,2,0,thisKF->xproj[2][0] + thisKF->xproj[5][0]);


  matrixFreeFloat(temp2b,thisKF->KFpar->Xdim);
  matrixFreeFloat(TMP,thisKF->KFpar->Xdim);


}

  

void KF_update(KF * thisKF, float ** meas)
{
  // Innovations
  matSub(meas,thisKF->yproj,thisKF->inov,thisKF->KFpar->Ydim,1);

  // Compute Kalman gain
  KF_computeK(thisKF);

  // Measurement update
  matMult(thisKF->K,thisKF->inov,thisKF->xupdt,thisKF->KFpar->Xdim,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim,1);
  matAdd(thisKF->xproj,thisKF->xupdt,thisKF->xupdt,thisKF->KFpar->Xdim,1);

  float normxupdt = normVectorColumn(thisKF->xupdt, 3);
  SET_MATRIX_VALUE(thisKF->xupdt,0,0,thisKF->xupdt[0][0]*thisKF->normF/normxupdt);
  SET_MATRIX_VALUE(thisKF->xupdt,1,0,thisKF->xupdt[1][0]*thisKF->normF/normxupdt);
  SET_MATRIX_VALUE(thisKF->xupdt,2,0,thisKF->xupdt[2][0]*thisKF->normF/normxupdt);
  
  // Update the covariance matrix
  KF_computeXupdt(thisKF);
}


void KF_computeK(KF * thisKF)
{
  float ** PVV = matrixAllocFloat(thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);
  float ** PVVinv = matrixAllocFloat(thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);


  // PVV_tmp_ = thisKF->H*thisKF->Xproj*thisKF->H';
  matMultTranspABAt(thisKF->H,thisKF->Xproj,PVV,thisKF->KFpar->Ydim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  
  // PVV = thisKF->H*thisKF->Xproj*thisKF->H' + thisKF->R;
  matAdd(PVV,thisKF->R,PVV,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);
  
  //printMatrix(PVV,3,3);
  //PVV^(-1)
  matInv(PVV,PVVinv,thisKF->KFpar->Ydim);
  matrixFreeFloat(PVV,thisKF->KFpar->Ydim);
  
  //TMP1 = thisKF->Xproj*thisKF->H';
  float ** TMP1 = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Ydim);
  matMultTransp(thisKF->Xproj,thisKF->H,TMP1,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Ydim,thisKF->KFpar->Xdim);
  
  // thisKF->K = Xproj*H'/(H*Xproj*H' + thisKF->R);
  matMult(TMP1,PVVinv,thisKF->K,thisKF->KFpar->Xdim,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim);

  //matrixFreeFloat(HT,thisKF->KFpar->Xdim);
  matrixFreeFloat(TMP1,thisKF->KFpar->Xdim);
  matrixFreeFloat(PVVinv,thisKF->KFpar->Ydim);


}


void KF_computeXupdt(KF * thisKF)
{
  uint8_t i;
    
  float ** TMP = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  //float ** II = matrixAllocFloat(thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  //setMatrixIdentity(II,thisKF->KFpar->Xdim);
  
  // TMP = thisKF->K*thisKF->H
  matMult(thisKF->K,thisKF->H,TMP,thisKF->KFpar->Xdim,thisKF->KFpar->Ydim,thisKF->KFpar->Ydim,thisKF->KFpar->Xdim);

  matMultConstant(TMP,-1.0,TMP,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);
  for (i = 0; i < thisKF->KFpar->Xdim; i++)
      TMP[i][i] = 1.0+TMP[i][i];
      
  // TMP1 = I-KH
  //matSub(II,TMP,TMP,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

  //thisKF->Xupdt = (I-KH)*thisKF->Xproj
  matMult(TMP,thisKF->Xproj,thisKF->Xupdt,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim,thisKF->KFpar->Xdim);

  
  matrixFreeFloat(TMP,thisKF->KFpar->Xdim);
  //matrixFreeFloat(II,thisKF->KFpar->Xdim);

}

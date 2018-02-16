/*
 * multiChipKF.c
 *
 *  Created on: jan 10, 2018
 *      Author: Gabriele
 */


#include "multiChipKF.h"
#include "tasks.h"

#if 1


void setKFStaticToZero(KF_static * KFs)
{
	//Vectors XDIM
	for (int i = 0; i < XDIM; i++)
	{
		KFs->xproj[i][0] = 0.0;
		KFs->xupdt[i][0] = 0.0;
	}

	//Vectors YDIM
	for (int i = 0; i < YDIM; i++)
	{
		KFs->inov[i][0] = 0.0;
		KFs->yproj[i][0] = 0.0;
		KFs->gyrP[i][0] = 0.0;
	}


	// Square matrices XDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < XDIM; j++)
		{

			KFs->Xproj[i][j] = 0.0;
			KFs->Xupdt[i][j] = 0.0;
			KFs->GAMMA[i][j] = 0.0;
		}

	// Square matrices YDIM x YDIM
	for (int i = 0; i < YDIM; i++)
		for (int j = 0; j < YDIM; j++)
			KFs->R[i][j] = 0.0;

	// Square matrices XIM x YDIM and YDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < YDIM; j++)
		{
			KFs->K[i][j] = 0.0;
			KFs->H[j][i] = 0.0;
		}

	KFs->KFpar = &accGyrKFpar;

	KFs->normF = 0.0;

	KFs->allocated = true;

}

void setRefFrameStaticToZero(refFrame_static * UVRHs)
{

	//Vectors 3D
	for (int i = 0; i < 3; i++)
	{
		UVRHs->gN[i][0] = 0.0;
		UVRHs->hN[i][0] = 0.0;
		UVRHs->gb[i][0] = 0.0;
		UVRHs->hb[i][0] = 0.0;
	}

	// Matrix 3x3
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			UVRHs->Mref[i][j] = 0.0;

	UVRHs->allocated = true;
	UVRHs->initialized = false;

}

// copy from static to dynamic to get ready for computation
void loadKFandRefFrame(KF_static * KFs, refFrame_static * UVRHs)
{

	// Kalman Filter loading

	//Vectors XDIM
	for (int i = 0; i < XDIM; i++)
	{
		accGyrKF.xproj[i][0] = KFs->xproj[i][0];
		accGyrKF.xupdt[i][0] = KFs->xupdt[i][0];
	}

	//Vectors YDIM
	for (int i = 0; i < YDIM; i++)
	{
		accGyrKF.inov[i][0] = KFs->inov[i][0];
		accGyrKF.yproj[i][0] = KFs->yproj[i][0];
		accGyrKF.gyrP[i][0] = KFs->gyrP[i][0];
	}


	// Square matrices XDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < XDIM; j++)
		{
			accGyrKF.Xproj[i][j] = KFs->Xproj[i][j];
			accGyrKF.Xupdt[i][j] = KFs->Xupdt[i][j];
			accGyrKF.GAMMA[i][j] = KFs->GAMMA[i][j];
		}

	// Square matrices YDIM x YDIM
	for (int i = 0; i < YDIM; i++)
		for (int j = 0; j < YDIM; j++)
			accGyrKF.R[i][j] = KFs->R[i][j];

	// Square matrices XIM x YDIM and YDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < YDIM; j++)
		{
			accGyrKF.K[i][j] = KFs->K[i][j];
			accGyrKF.H[j][i] = KFs->H[j][i];
		}

	accGyrKF.normF = KFs->normF;

	accGyrKF.allocated = KFs->allocated;

	// Reference Frame loading

	//Vectors 3D
	for (int i = 0; i < 3; i++)
	{
		UVRH.gN[i][0] = UVRHs->gN[i][0];
		UVRH.hN[i][0] = UVRHs->hN[i][0];
		UVRH.gb[i][0] = UVRHs->gb[i][0];
		UVRH.hb[i][0] = UVRHs->hb[i][0];
	}

	// Matrix 3x3
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			UVRH.Mref[i][j] = UVRHs->Mref[i][j];

	UVRH.allocated = UVRHs->allocated;
	UVRH.initialized = UVRHs->initialized;


}

void saveKFandRefFrame(KF_static * KFs, refFrame_static * UVRHs)
{

	// Kalman Filter loading

	//Vectors XDIM
	for (int i = 0; i < XDIM; i++)
	{
		KFs->xproj[i][0] = accGyrKF.xproj[i][0];
		KFs->xupdt[i][0] = accGyrKF.xupdt[i][0];
	}

	//Vectors YDIM
	for (int i = 0; i < YDIM; i++)
	{
		KFs->inov[i][0] = accGyrKF.inov[i][0];
		KFs->yproj[i][0] = accGyrKF.yproj[i][0];
		KFs->gyrP[i][0] = accGyrKF.gyrP[i][0];
	}


	// Square matrices XDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < XDIM; j++)
		{
			KFs->Xproj[i][j] = accGyrKF.Xproj[i][j];
			KFs->Xupdt[i][j] = accGyrKF.Xupdt[i][j];
			KFs->GAMMA[i][j] = accGyrKF.GAMMA[i][j];
		}

	// Square matrices YDIM x YDIM
	for (int i = 0; i < YDIM; i++)
		for (int j = 0; j < YDIM; j++)
			KFs->R[i][j] = accGyrKF.R[i][j];

	// Square matrices XIM x YDIM and YDIM x XDIM
	for (int i = 0; i < XDIM; i++)
		for (int j = 0; j < YDIM; j++)
		{
			KFs->K[i][j] = accGyrKF.K[i][j];
			KFs->H[j][i] = accGyrKF.H[j][i];
		}

	KFs->normF = accGyrKF.normF;

	KFs->allocated = accGyrKF.allocated;

	// Reference Frame saving
	//Vectors 3D
	for (int i = 0; i < 3; i++)
	{
		UVRHs->gN[i][0] = UVRH.gN[i][0];
		UVRHs->hN[i][0] = UVRH.hN[i][0];
		UVRHs->gb[i][0] = UVRH.gb[i][0];
		UVRHs->hb[i][0] = UVRH.hb[i][0];
	}

	// Matrix 3x3
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			 UVRHs->Mref[i][j] = UVRH.Mref[i][j];

	UVRHs->allocated = UVRH.allocated;
	UVRHs->initialized = UVRH.initialized;

}
#endif
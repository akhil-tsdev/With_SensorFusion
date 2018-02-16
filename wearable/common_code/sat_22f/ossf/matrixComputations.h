/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   matrixComputations.h
 * Author: User
 *
 * Created on 4 maggio 2017, 12.42
 */

#ifndef MATRIXCOMPUTATIONS_H
#define MATRIXCOMPUTATIONS_H
#include "stdint.h"



// Allocate a matrix of int
int** matrixAllocInt(int nr, int nc);

// Allocate a matrix of float
float** matrixAllocFloat(int nr, int nc);

// Deallocate an int matrix with nr rows
void matrixFreeInt(int **arr, int nr);

// Deallocate a float matrix with nr rows
void matrixFreeFloat(float **arr, int nr);

// Deallocate a matrix with nr rows
void matrixFree(void **arr, int nr);



// Set identity matrix (square matrix only)
void setMatrixIdentity(float **A, int N);

// Zero matrix
void setMatrixZeros(float **AA, int nr, int nc);

// Ones matrix
void setMatrixOnes(float **AA, int nr, int nc);

// Copy matrix
void matrixCopy(float **AIN, float **AOUT, int nr, int nc);

// Copy B as a sub-matrix of A starting at (rStart,cStart) 
void matrixSubBlock(float **A, float **B, int rStart, int cStart, int rB, int cB);

//matrix addition C = A + B
void matAdd(float **A, float **B, float **C, int nr, int nc);

//matrix subtraction C = A - B
void matSub(float **A, float **B, float **C, int nr, int nc);

//matrix multiplication AB = A*B
void matMult(float ** A, float ** B, float ** AB, int Ar, int Ac, int Br, int Bc);

// matrixMuliplication ABt = A*B'
void matMultTransp(float ** A, float ** B, float ** ABt, int Ar, int Ac, int Br, int Bc);

// matrix multiplication with transp matrix ABAt = A*B*A';
void matMultTranspABAt(float ** A, float ** B, float ** ABAt, int Ar, int Ac, int Br, int Bc);

//matrix transposition
void matTransp(float ** AIN, float ** AOUT, int Nr, int Nc);

//matrix inversion
void matInv(float ** AIN, float ** AOUT, int N);

// LU decomposition
void luDcmp(float **a, int n, int **indx, float *d);

// LU backsubstitution
void luBksb(float **a, int n, int **indx, float **col);

// matrix multiplication for a constant
void matMultConstant(float ** A, float c, float **cA, int Ar, int Ac);

// Compute the norm of a vector (column)
float normVectorColumn(float **v, int dim);

// Compute the norm of a vector (column)
float normVectorRow(float **v, int dim);

// Normalize a column vector
void normalizeVectorColumn(float **v, float **vn, int dim);

void crossProduct3D(float **v1, float **v2, float** vout);







#endif /* MATRIXCOMPUTATIONS_H */


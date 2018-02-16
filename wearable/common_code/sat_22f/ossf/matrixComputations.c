/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <math.h>
#include <stdlib.h>

#include "matrixComputations.h"

#define TINY 1.0e-20


// Allocate a matrix of int
int** matrixAllocInt(int nr, int nc)
{
    int i;
    
    int **arr = (int **)malloc(nr * sizeof(int *));
    
    for (i=0; i<nr; i++)
         arr[i] = (int *)malloc(nc * sizeof(int));

    return arr;
}

// Allocate a matrix of float
float** matrixAllocFloat(int nr, int nc)
{
    int i;
    
    float **arr = (float **)malloc(nr * sizeof(float *));
    
    for (i=0; i<nr; i++)
         arr[i] = (float *)malloc(nc * sizeof(float));

    return arr;
}


// Deallocate an int matrix with nr rows
void matrixFreeInt(int **arr, int nr)
{
    matrixFree((void**)arr, nr);
}

// Deallocate a float matrix with nr rows
void matrixFreeFloat(float **arr, int nr)
{
    matrixFree((void**)arr, nr);
}

// Deallocate a matrix with nr rows
void matrixFree(void **arr, int nr)
{
    int i;
    
    for (i= 0; i < nr; i++)
        free(arr[i]);
    free(arr);
}


// Set identity matrix (square matrix only)
void setMatrixIdentity(float **A, int n)
{   
    int i,j;
    for (i = 0; i < n; i++)
	for(j = 0; j < n; j++)
            if (i != j)
		A[i][j] = 0;
            else
                A[i][j] = 1.0;
}

// Zero matrix
void setMatrixZeros(float **AA, int nr, int nc)
{
    int i,j;
    for (i = 0; i < nr; i++)
	for(j = 0; j < nc; j++)
            AA[i][j] = 0.0;

}

// Ones matrix
void setMatrixOnes(float **AA, int nr, int nc)
{
    int i,j;
    for (i = 0; i < nr; i++)
	for(j = 0; j < nc; j++)
            AA[i][j] = 1.0;

}
// Copy matrix
void matrixCopy(float **AIN, float **AOUT, int nr, int nc)
{
	int i,j;
	for (i=0;i<nc;i++)
		for (j=0;j<nr;j++)
			AOUT[j][i]=AIN[j][i];

}
// Copy B as a sub-matrix of A starting at (rStart,cStart) 
void matrixSubBlock(float **A, float **B, int rStart, int cStart, int rB, int cB)
{
    int i,j;
    for(i=0; i < rB; i++)
        for(j=0; j < cB; j++)
            A[rStart+i][cStart+j] = B[i][j];
}

//matrix addition C = A + B
void matAdd(float **A, float **B, float **C, int nr, int nc)
{
	int i,j;

	for (j=0;j<nc;j++)
		for (i=0;i<nr;i++)
			C[i][j]=A[i][j]+B[i][j];
}


//matrix subtraction C = A - B
void matSub(float **A, float **B, float **C, int nr, int nc)
{
	int i,j;
	for (i=0;i<nr;i++)
		for (j=0;j<nc;j++)
			C[i][j]=A[i][j]-B[i][j];
}

//matrix multiplication AB = A*B
void matMult(float ** A, float ** B, float ** AB, int Ar, int Ac, int Br, int Bc)
{
	//if (Ac != Br) 
		//nrerror("Inner matrix dimension must agree");

        int i,j,k;
	
        for (i= 0; i < Ar; i++)
		for (j = 0; j < Bc; j++) 
		{
			float sum = 0;
			for (k = 0; k < Ac; k++) 
			{
				sum += A[i][k]*B[k][j];
			}
			AB[i][j] = sum;
		}
}

//matrix multiplication ABAt = A*B*(A') (square matrix only)
void matMultTranspABAt(float ** A, float ** B, float ** ABAt, int Ar, int Ac, int Br, int Bc)
{
        int i,j,k;
	float ** temp = matrixAllocFloat(Ar,Bc);
        
        // AB = A*B
        for (i= 0; i < Ar; i++)
		for (j = 0; j < Bc; j++) 
		{
			float sum = 0;
			for (k = 0; k < Ac; k++) 
			{
				sum += A[i][k]*B[k][j];
			}
			temp[i][j] = sum;
		}
        
        // ABAt = AB*At
        for (i= 0; i < Ar; i++)
            for (j = 0; j < Ar; j++) 
            {
		float sum = 0;
		for (k = 0; k < Bc; k++) 
		{
                    sum += temp[i][k]*A[j][k];
		}
		ABAt[i][j] = sum;
            }
        
        matrixFreeFloat(temp,Ar);
        
}

void matMultTransp(float ** A, float ** B, float ** ABt, int Ar, int Ac, int Br, int Bc)
{
    uint8_t i,j,k;
    
            // ABAt = AB*At
        for (i= 0; i < Ar; i++)
            for (j = 0; j < Br; j++) 
            {
		float sum = 0;
		for (k = 0; k < Ac; k++) 
		{
                    sum += A[i][k]*B[j][k];
		}
		ABt[i][j] = sum;
            }
}


//matrix transposition
void matTransp(float ** AIN, float ** AOUT, int Nr, int Nc)
{
    uint8_t i,j;
    
    for (i = 0; i < Nc; i ++)
    	for(j = 0; j < Nr; j++)
            AOUT[i][j] = AIN[j][i];
}


//matrix inversion
void matInv(float ** AIN, float ** AOUT, int N)
{

	float d;
	int i,j;
	int ** indx = matrixAllocInt(N,1);
	float ** col = matrixAllocFloat(N,1);
	float** temp = matrixAllocFloat(N,N);
	matrixCopy(AIN,temp,N,N);

	// LU decomposition, ludcmp() destroys its argument matrix
	luDcmp(temp,N,indx,&d);

	//Find inverse by columns.
	for(j=0;j<N;j++) 
	{ 
		for(i=0;i<N;i++) 
                    col[i][0]=0.0;
                
                col[j][0]=1.0;
		
		luBksb(temp,N,indx,col);
		for(i=0;i<N;i++) 
			AOUT[i][j]=col[i][0];
	}
	
	// Deallocation
	matrixFreeInt(indx,N);
	matrixFreeFloat(col,N);
        matrixFreeFloat(temp,N);
}


// LU decomposition
void luDcmp(float **a, int n, int **indx, float *d)
{
	int i,imax,j,k;
	float big,dum,sum,temp;
	float **vv = matrixAllocFloat(n,1);

	*d=1.0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if ((temp=fabs(a[i][j])) > big) 
				big=temp;
//		if (big == 0.0)
//			printf("Singular matrix in routine ludcmp");
		vv[i][0]=1.0/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a[i][j];
			for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			sum=a[i][j];
			for (k=0;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i][0]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=0;k<n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax][0]=vv[j][0];
		}
		indx[j][0]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<n;i++) a[i][j] *= dum;
		}
	}
	matrixFreeFloat(vv,n);
}



// LU substitution
void luBksb(float **a, int n, int **indx, float **col)
{
	int i,ii=-1,ip,j;
	float sum;

	for (i=0;i<n;i++) {
		ip=indx[i][0];
		sum=col[ip][0];
		col[ip][0]=col[i][0];
		if (ii+1)
			for (j=ii;j<=i-1;j++) 
				sum -= a[i][j]*col[j][0];
		else 
			if (sum) 
				ii=i;
		col[i][0]=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=col[i][0];
		for (j=i+1;j<n;j++) 
			sum -= a[i][j]*col[j][0];
		col[i][0]=sum/a[i][i];
	}
}

// matrix multiplication for a constant
void matMultConstant(float ** A, float c, float **cA, int Ar, int Ac)
{
    int i, j;
    
    for(i = 0; i < Ar; i++)
        for(j = 0; j < Ac; j++)
            cA[i][j] = c*A[i][j];
}

// Compute the norm of a vector (column)
float normVectorColumn(float **v, int dim)
{
    int j;
    float res = 0.0;
    
    for(j = 0; j < dim; j++)
        res += v[j][0]*v[j][0];
    
    return sqrt(res);
}

// Compute the norm of a vector (column)
float normVectorRow(float **v, int dim)
{
    int i;
    float res = 0.0;
    
    for(i = 0; i < dim; i++)
        res += v[0][i]*v[0][i];
    
    return sqrt(res);
}

void normalizeVectorColumn(float **v, float **vn, int dim)
{
    int i;
    float normv = normVectorColumn(v, 3);
    for (i = 0; i < dim; i++)
        vn[i][0] = v[i][0]/normv;

}

// vout = v1 x v2;
void crossProduct3D(float **v1, float **v2, float** vout)
{
    vout[0][0] = -v1[2][0]*v2[1][0] + v1[1][0]*v2[2][0];
    vout[1][0] = v1[2][0]*v2[0][0] - v1[0][0]*v2[2][0];
    vout[2][0] = -v1[1][0]*v2[0][0] + v1[0][0]*v2[1][0];
}

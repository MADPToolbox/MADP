/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Bas Terwijn
 * Xuanjie Liu
 *
 * For contact information please see the included AUTHORS file.
 */


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <cuda_runtime.h>

#include <cublas_v2.h>
#include <cusolverDn.h>
#include <iostream>
#include "SystemOfLinearEquationsSolver.h"

using namespace std;


SystemOfLinearEquationsSolver::SystemOfLinearEquationsSolver(int m, int nrhs, double *A, double *B)
{
	this->m = m;
	this->nrhs = nrhs;
	this->A = A;
	this->B = B;
	this->XC = (double *)malloc(sizeof(double)*m);
}

void SystemOfLinearEquationsSolver::printMatrix(int m, int n, const double*A, int lda, const char* name)
{
    for(int row = 0 ; row < m ; row++){
        for(int col = 0 ; col < n ; col++){
            double Areg = A[row + col*lda];
        }
    }
}



double *SystemOfLinearEquationsSolver::getSolution()
{
	return this->XC;
}

void SystemOfLinearEquationsSolver::solveSystemOfLinearEquations()
{
	cusolverDnHandle_t cudenseH = NULL;
	cublasHandle_t cublasH = NULL;
	cublasStatus_t cublas_status = CUBLAS_STATUS_SUCCESS;
	cusolverStatus_t cusolver_status = CUSOLVER_STATUS_SUCCESS;
	cudaError_t cudaStat1 = cudaSuccess;
	cudaError_t cudaStat2 = cudaSuccess;
	cudaError_t cudaStat3 = cudaSuccess;
	cudaError_t cudaStat4 = cudaSuccess;
	int m = this->m;
	int lda = m;
	int ldb = m;
	int nrhs = this->nrhs;
	double *A = this->A;
	double *B = this->B;
	double XC[ldb*nrhs]; // solution matrix from GPU

	double *d_A = NULL; // linear memory of GPU
	double *d_tau = NULL; // linear memory of GPU
	double *d_B  = NULL;
	int *devInfo = NULL; // info in gpu (device copy)
	double *d_work = NULL;
	int  lwork = 0;

	int info_gpu = 0;

	const double one = 1;


	// step 1: create cudense/cublas handle
	cusolver_status = cusolverDnCreate(&cudenseH);
	assert(CUSOLVER_STATUS_SUCCESS == cusolver_status);

	cublas_status = cublasCreate(&cublasH);
	assert(CUBLAS_STATUS_SUCCESS == cublas_status);

	// step 2: copy A and B to device
	cudaStat1 = cudaMalloc ((void**)&d_A  , sizeof(double) * lda * m);
	cudaStat2 = cudaMalloc ((void**)&d_tau, sizeof(double) * m);
	cudaStat3 = cudaMalloc ((void**)&d_B  , sizeof(double) * ldb * nrhs);
	cudaStat4 = cudaMalloc ((void**)&devInfo, sizeof(int));
	assert(cudaSuccess == cudaStat1);
	assert(cudaSuccess == cudaStat2);
	assert(cudaSuccess == cudaStat3);
	assert(cudaSuccess == cudaStat4);

	cudaStat1 = cudaMemcpy(d_A, A, sizeof(double) * lda * m   , cudaMemcpyHostToDevice);
	cudaStat2 = cudaMemcpy(d_B, B, sizeof(double) * ldb * nrhs, cudaMemcpyHostToDevice);
	assert(cudaSuccess == cudaStat1);
	assert(cudaSuccess == cudaStat2);

	// step 3: query working space of geqrf and ormqr
	cusolver_status = cusolverDnDgeqrf_bufferSize(
	    cudenseH,
	    m,
	    m,
	    d_A,
	    lda,
	    &lwork);
	assert (cusolver_status == CUSOLVER_STATUS_SUCCESS);

	cudaStat1 = cudaMalloc((void**)&d_work, sizeof(double)*lwork);
	assert(cudaSuccess == cudaStat1);

	// step 4: compute QR factorization
	cusolver_status = cusolverDnDgeqrf(
	    cudenseH,
	    m,
	    m,
	    d_A,
	    lda,
	    d_tau,
	    d_work,
	    lwork,
	    devInfo);
	cudaStat1 = cudaDeviceSynchronize();
	assert(CUSOLVER_STATUS_SUCCESS == cusolver_status);
	assert(cudaSuccess == cudaStat1);

	    // check if QR is good or not
	cudaStat1 = cudaMemcpy(&info_gpu, devInfo, sizeof(int), cudaMemcpyDeviceToHost);
	assert(cudaSuccess == cudaStat1);

	//printf("after geqrf: info_gpu = %d\n", info_gpu);
	assert(0 == info_gpu);

	// step 5: compute Q^T*B
	cusolver_status= cusolverDnDormqr(
	    cudenseH,
	    CUBLAS_SIDE_LEFT,
	    CUBLAS_OP_T,
	    m,
	    nrhs,
	    m,
	    d_A,
	    lda,
	    d_tau,
	    d_B,
	    ldb,
	    d_work,
	    lwork,
	    devInfo);
	cudaStat1 = cudaDeviceSynchronize();
	assert(CUSOLVER_STATUS_SUCCESS == cusolver_status);
	assert(cudaSuccess == cudaStat1);
	    // check if QR is good or not
	cudaStat1 = cudaMemcpy(&info_gpu, devInfo, sizeof(int), cudaMemcpyDeviceToHost);
	assert(cudaSuccess == cudaStat1);

	//printf("after ormqr: info_gpu = %d\n", info_gpu);
	assert(0 == info_gpu);

	// step 6: compute x = R \ Q^T*B

	cublas_status = cublasDtrsm(
	     cublasH,
	     CUBLAS_SIDE_LEFT,
	     CUBLAS_FILL_MODE_UPPER,
	     CUBLAS_OP_N,
	     CUBLAS_DIAG_NON_UNIT,
	     m,
	     nrhs,
	     &one,
	     d_A,
	     lda,
	     d_B,
	     ldb);
	cudaStat1 = cudaDeviceSynchronize();
	assert(CUBLAS_STATUS_SUCCESS == cublas_status);
	assert(cudaSuccess == cudaStat1);

	cudaStat1 = cudaMemcpy(XC, d_B, sizeof(double)*ldb*nrhs, cudaMemcpyDeviceToHost);
	assert(cudaSuccess == cudaStat1);

	for(int i = 0; i < m; i++){
			this->XC[i] = XC[i];
	}


	// free resources
	if (d_A    ) cudaFree(d_A);
	if (d_tau  ) cudaFree(d_tau);
	if (d_B    ) cudaFree(d_B);
	if (devInfo) cudaFree(devInfo);
	if (d_work ) cudaFree(d_work);


	if (cublasH ) cublasDestroy(cublasH);
	if (cudenseH) cusolverDnDestroy(cudenseH);

	cudaDeviceReset();


}


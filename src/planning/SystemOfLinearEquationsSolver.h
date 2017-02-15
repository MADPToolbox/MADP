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
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#ifndef _SYSTEMOFLINEAREQUATIONSSOLVER_H_
#define _SYSTEMOFLINEAREQUATIONSSOLVER_H_ 1

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <cuda_runtime.h>

#include <cublas_v2.h>
#include <cusolverDn.h>
#include <iostream>

class SystemOfLinearEquationsSolver
{
private:
	int m;
	int nrhs;
	double *A;
	double *B;
	double *XC;

public:
	SystemOfLinearEquationsSolver(int m, int nrhs, double *A, double *B);
	void printMatrix(int m, int n, const double*A, int lda, const char* name);
	void solveSystemOfLinearEquations();
	double *getSolution();


};

#endif

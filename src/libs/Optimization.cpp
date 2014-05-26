// author: manuel scholz

//*** INCLUDE *************************************************************************************

#include "Optimization.h"
#include "Linearsolver.h"
#include "fastmath.h"
#include "stdlib.h"

#include <string.h>

//*** CLASS ***************************************************************************************

/**
* computes the jacobi matrix of the residual (the derivative of the system) at the given location in parameter space
* @param ParameterVector	parameter vector defining the location at which the jacobi matrix is computed
* @param h			step size for finite differences
* @param JacobiMatrix		output jacobi matrix of the residual. Must be an n x m matrix where n=getParameterCount() and m=getResidualElementCount()
* @param ResidualVector		output residual vector at location defined by ParameterVector
*/
void CLeastSquareProblem::computeResidualJacobiMatrix(float* ParameterVector, float h, float* JacobiMatrix, float* ResidualVector) {
  int n = getParameterCount();			// number of parameter
  int m = getResidualElementCount();	// number of measurements

  // jacobimatrix must have n rows, m cols

  // create parameter vector
  float* r0 = ResidualVector;	// residual vector for given parameter vector
  float* r = new float[m];	// residual vector for modified parameter vector

  // compute residual at initial dParameterVector
  computeResidualVector(ParameterVector, r0);

  // iterate over parameters
  for(int j=0; j<n; j++) {
    // backup parameter value
    float tmp = ParameterVector[j];

    // get residual vector after parameter j was changed
    ParameterVector[j] += h;
    computeResidualVector(ParameterVector, r);
    ParameterVector[j] = tmp;

    // fill matrix with finite differences
    for(int i=0; i<m; i++)
      JacobiMatrix[i*n+j]  = (r[i] - r0[i])/h;
  }

  for(int j=0; j<n; j++) {
    r[j] = 0;
    for(int i=0; i<m; i++) {
      r[j] += JacobiMatrix[i*n+j];
    }
  }

  // clean up
  delete[] r;
}

/**
* Performs a levenber marquard optimization of the givien least squares system. 
* The method alters the given parameters so that the squared residual length is minimized
* @param Parameter		vector of initial parameters of length 'lsp->getParameterCount()'. Will contain the result parameters after optimization.
* @param lsp			a least square problem
* @param MaxIterations		maximum number of optimization iterations
* @param MaxError		maximum allowed error. When error drops below this threshold the optimization is stopped
* @return			error after optimization (squared residual length)
*/
float CLeastSquareProblem::optimizeLevMar(float* Parameter, int MaxIterations, float MaxError) {
  const float lambdaMax = 100000;	// maximum step size
  const float h = 0.001;		// step size for numeric differentiation

  int n = getParameterCount();		// number of parameters
  int m = getResidualElementCount();	// number of measurements

  float lambda = 0.0000000001;		// step size
  float dSqrResidualLength = 1.0e30;	// error

  float* pold	= new float[n];		// old parameter delta	
  float* dp	= new float[n];		// parameter delta	
  float* r	= new float[m];		// residual vector
  float* J	= new float[m*n];	// Jacobi matrix of residual vector
  float* b	= new float[n];		// right side of linear equation system	
  float* A	= new float[m*m];	// matrix of the left side of linear equation system (Ax = b)
  
  for(int a=0; a<MaxIterations; a++) {
    // compute Jacobi matrix of residual and residual for current parameter set
    computeResidualJacobiMatrix(Parameter, h, J, r);

    // compute new residual length
    float l = 0;
    for(int i=0; i<m; i++)
      l += r[i]*r[i];

    // check if we got a better solution
    if(l <= dSqrResidualLength) {
      // update lambda
      lambda /= 10;
      dSqrResidualLength = l;
    } else {
      // adjust lambda
      if(lambda<lambdaMax)
	lambda *= 9;
      // restore old parameter set
      memcpy(Parameter, pold, sizeof(float)*n);
      // recompute Jacobi matrix of residual and residual for current parameter set
      computeResidualJacobiMatrix(Parameter, h, J, r);
    }

    // stop error is small enough
    if(dSqrResidualLength < MaxError)
      break;

    // A = J-Transposed x J 
    for(int i=0; i<n; i++) {
      for(int j=0; j<n; j++) {
	A[n*i+j] = 0;
	for(int k=0; k<m; k++)
	  A[n*i+j] += J[n*k+i] * J[n*k+j];
      }
    }

    // add diag(A)*lambda to A
    for(int i=0; i<n; i++)
      A[n*i+i] += A[n*i+i]*lambda;

    // compute J-Transposed x r
    for(int i=0; i<n; i++) {
      b[i] = 0;
      for(int k=0; k<m; k++)
	b[i] += J[n*k+i] * r[k];
    }

    // solve linear equation system A*dp = b
    linearSolveLU(A, b, dp, n);

    // backup old parameters set and update parameter vector 
    memcpy(pold, Parameter, sizeof(float)*n);
    for(int i=0; i<n; i++)
      Parameter[i] -= dp[i];

    onIterationFinished(a, sqrt(dSqrResidualLength/m));
  }

  // clean up
  delete[] pold;
  delete[] dp;	// parameter delta	
  delete[] r;	// residual vector
  delete[] J;	// Jacobi matrix of residual vector
  delete[] b;	// right side of linear equation system	
  delete[] A;	// matrix of the left side of linear equation system (Ax = b)

  return dSqrResidualLength;
}

//*************************************************************************************************

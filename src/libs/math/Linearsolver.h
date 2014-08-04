// author: manuel scholz

#ifndef LINEAR_SOLVER_H
#define LINEAR_SOLVER_H

//*** FUNCTION ************************************************************************************

// solves linear equation system Ax = b for x using LU-decomposition. Not very effective for large sparse systems
void linearSolveLU(float* A, float* b, float* x, int n);

#endif

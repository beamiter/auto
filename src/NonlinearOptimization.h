#ifndef _NON_LINEAR_OPTIMIZATION_H
#define _NON_LINEAR_OPTIMIZATION_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <math.h>

using namespace std;
using namespace Eigen;

const double DERIV_STEP = 1e-5;
const int MAX_ITER = 100;

/*
 *GaussNewton
 */

double func(const VectorXd &input, const VectorXd &output, const VectorXd &params, double objIndex);

//return vector make up of func() element.
VectorXd objF(const VectorXd &input, const VectorXd &output, const VectorXd &params);

//F = (f ^t * f)/2
double Func(const VectorXd &obj);

double Deriv(const VectorXd &input, const VectorXd &output, int objIndex, const VectorXd &params,
             int paraIndex);

MatrixXd Jacobin(const VectorXd &input, const VectorXd &output, const VectorXd &params);

void gaussNewton(const VectorXd &input, const VectorXd &output, VectorXd &params);

/*
 *LevenbergMarquardt
 */

double maxMatrixDiagonale(const MatrixXd &Hessian);
//L(h) = F(x) + h^t*J^t*f + h^t*J^t*J*h/2
//deltaL = h^t * (u * h - g)/2
double linerDeltaL(const VectorXd &step, const VectorXd &gradient, const double u);

void levenMar(const VectorXd &input, const VectorXd &output, VectorXd &params);

/*
 *dogleg
 */

void dogLeg(const VectorXd &input, const VectorXd &output, VectorXd &params);

#endif
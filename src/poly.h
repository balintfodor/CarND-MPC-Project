#ifndef _POLY_H_
#define _POLY_H_

#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

double polyeval(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif
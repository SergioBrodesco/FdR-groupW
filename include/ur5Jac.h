/**
 * @file ur5Jac.h
 * @brief Header with the declaration of the function we use to compute the jacobian.
 * 
 */
#ifndef UR5JAC_H
#define UR5JAC_H

#include "utils.h"
using namespace Eigen;

/**
 * @brief Compute the jacobian matrix for a specified joint configuration
 * 
 * 19
 * @param Th joint configuration we want to compute the jacobian for
 * @return Resulting Jacobian Matrix
 */
MatrixXd ur5Jac(VectorXd Th);

#endif

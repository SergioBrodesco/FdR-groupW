#include "../include/invDiffKinematiControlRedundancy.h"
#include <iostream>

using namespace Eigen;

// Computation of the controlled values for the translation motion
// A: parameters. We exploit redundancy to attain some secondary goal, which
// is encoded in the solutions q0
// Jac: jacobian (function of q)
// q: position of the joints
// xe: position of the end effector
// xd: desired position of the end effector
// vd: desired velocity fo the end effector
// dotq0: secondary task joint velocities
// K: positive definite matrix to reduce the error
// dotQ: velocity to be applied at the joints
VectorXd invDiffKinematicControlRedundancy(const VectorXd& q, const Vector3d& xe, const Vector3d& xd, const Vector3d& vd, const VectorXd& dotq0, const Matrix3d& K){
  MatrixXd J = ur5Jac(q);
  
  // Extract first 3 rows and 6 columns
  J = J.block(0, 0, 3, 6);

  MatrixXd pseud_inv_J = J.completeOrthogonalDecomposition().pseudoInverse();
  MatrixXd identity = MatrixXd::Identity(6, 6);
  MatrixXd dotQ = (pseud_inv_J * (vd + K * (xd - xe))) +
                  ((identity - (pseud_inv_J * J)) * dotq0);

  return dotQ;
}

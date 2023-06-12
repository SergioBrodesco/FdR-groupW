/**
 * @file invDiffKinematicControlSimCompleteAngleAxis.h
 * @brief Header file containing the declaration of the functions we use to compute the Differential kinematic.
 * 
 */
#ifndef INVDIFFKINEMATICCONTROLSIMCOMPLETEANGLEAXIS_H
#define INVDIFFKINEMATICCONTROLSIMCOMPLETEANGLEAXIS_H

#include "ur5Jac.h"
#include "invDirKinematics.h"

using namespace Eigen;

/**
 * @brief Computes the differential kinematic over a received trajectory
 * 
 * @param xd matrix containing end effector positions at each time step of the trajectory
 * @param phid matrix containing end effector rotations at each time step of the trajectory
 * @param TH0 joint configuration at the start of the trajectory
 * @param THf joint configuration at the end of the trajectory
 * @param minT time of the start of the trajectory
 * @param maxT time of the end of the trajectory
 * @param Dt sampling time
 * @return std::tuple<MatrixXd, MatrixXd, MatrixXd> corrected trajectory (joint positions, end effector positions, end effector rotations)
 */
std::tuple<MatrixXd, MatrixXd, MatrixXd> invDiffKinematicControlSimCompleteAngleAxis(const MatrixXd& xd, const MatrixXd& phid , const VectorXd& TH0,
                                                                                     const VectorXd& THf, const double minT, const double maxT, const double Dt);

/**
 * @brief Computes the joint velocities we to use for the Differential kinematics.\n
 * This is called by invDiffKinematicControlSimCompleteAngleAxis() for every configuration in a trajectory.
 * 
 * @param q 
 * @param xe current end effector position
 * @param xd desired end effector position
 * @param vd desired end effector velocity
 * @param w_R_e current end effector rotation (rotation matrix)
 * @param phid desired end effector rotation (euler angles)
 * @param phiddot desired end effector angular velocity
 * @param Kp scaling factor for the position error
 * @param Kphi scaling factor for the orientation error obtained from get_optimal_Kphi()
 * @return VectorXd joint velocities corrected using the position and orientation error
 */
VectorXd invDiffKinematicControlCompleteAngleAxis(const VectorXd& q, const Vector3d& xe, const Vector3d& xd, const Vector3d& vd, const Matrix3d& w_R_e,
                                                  const Vector3d& phid, const Vector3d& phiddot, const Matrix3d& Kp, const Matrix3d& Kphi);

/**
 * @brief Compute Kphi used in the invDiffKinematicControlCompleteAngleAxis() function
 * 
 * @param start_cfg start joint configuration
 * @param end_cfg finish joint configuration
 * @param base_factor base factor added to the computed Kphi (to make sure its never 0)
 * @return Matrix3d matrix containing the values for Kphi we are going to use on the orientation error
 */
Matrix3d get_optimal_Kphi (const VectorXd& start_cfg, const VectorXd& end_cfg, const float base_factor);

/**
 * @brief Function that computes the orientation error of the end effector, using Angle axis instead of Euler angles
 * 
 * @param w_R_e current end effector rotation (rotation matrix)
 * @param w_R_d desired end effector rotation (rotation matrix)
 * @return Vector3d orientation error between current and desired end effector rotation
 */
Vector3d computeOrientationErrorW(Matrix3d w_R_e, Matrix3d w_R_d);

#endif
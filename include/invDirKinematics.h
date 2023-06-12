/**
 * @file invDirKinematics.h
 * @brief Header file containing the declaration of the functions we use to compute the Direct and Inverse kinematics.
 * 
 */
#ifndef INVDIRKINEMATICS_H
#define INVDIRKINEMATICS_H

#include "utils.h"
using namespace Eigen;
/**
 * @brief Implementation of Inverse kinematic for the ur5
 * 
 * @param p60 end effector position
 * @param R60 end effector rotation
 * @return MatrixXd 
 */
MatrixXd ur5Inverse(Vector3d p60, Matrix3d R60);
/**
 * @brief Implementation of Direct kinematic for the ur5
 * 
 * @param Th vector containing the joint configuration
 * @return std::tuple<Vector3d, Matrix3d> position and rotation of the end effector
 */
std::tuple<Vector3d, Matrix3d> ur5Direct(VectorXd Th);

/**
 * @brief Returns the transformation matrix from base frame to joint 1, given the angle we are using
 * 
 * @param th1 angle between base frame and joint 1
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T10(double th1);
/**
 * @brief Returns the transformation matrix from joint 1 to joint 2, given the angle we are using
 * 
 * @param th2 angle between joint 1 and joint 2
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T21(double th2);
/**
 * @brief Returns the transformation matrix from joint 2 to joint 3, given the angle we are using
 * 
 * @param th3 angle between joint 2 and joint 3
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T32(double th3);
/**
 * @brief Returns the transformation matrix from joint 3 to joint 4, given the angle we are using
 * 
 * @param th4 angle between joint 3 and joint 4
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T43(double th4);
/**
 * @brief Returns the transformation matrix from joint 4 to joint 5, given the angle we are using
 * 
 * @param th5 angle between joint 4 and joint 5
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T54(double th5);
/**
 * @brief Returns the transformation matrix from joint 5 to the last joint, given the angle we are using
 * 
 * @param th6 angle between joint 5 and the last joint
 * @return Matrix4d Computed transformation matrix
 */
Matrix4d T65(double th6);

#endif

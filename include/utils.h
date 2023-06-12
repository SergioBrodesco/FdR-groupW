/**
 * @file utils.h
 * @brief Header containing commonly used functions and libraries.
 * 
 */
#ifndef UTILS_H
#define UTILS_H

#include <array>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <string.h>
#include <cstdlib>
#include <cmath>

using namespace Eigen;


/**
 * @brief Function that transforms a point from World frame to Robot frame
 * 
 * @param pos point we want to transform
 * @return Vector3d containing the point translated in the Robot frame
 */
Vector3d W_t_R_transform(const Vector3d& pos);  // World frame to Robot frame transformation of a point

/**
 * @brief Function that transforms a point from Robot frame to World frame
 * 
 * @param pos point we want to transform
 * @return Vector3d containing the point translated in the World frame 
 */
Vector3d R_t_W_transform(const Vector3d& pos);  // Robot frame to World frame transformation of a point

/**
 * @brief Function that translates from Euler angles to rotation matrix
 * 
 * @param eulXYZ Euler angles we want to transform (XYZ vector)
 * @return Matrix3d containing the rotation matrix (ZYX matrix) obtained from the received Euler angles
 */
Matrix3d eul2rotmFDR(const Vector3d& eulXYZ);  // Transform vector expressed in Euler angles into a rotaiton matrix (XYZ vector into ZYX matrix)

/**
 * @brief Function that translates from rotation matrix to Euler angles
 * 
 * @param R rotation matrix we want to transform (ZYX matrix)
 * @return Vector3d containing the Euler angles (XYZ vector) obtained from the received rotation matrix
 */
Vector3d rotm2eulFDR(const Matrix3d& R);  // Transform rotaiton matrix into a vector expressed in Euler angles (ZYX matrix into XYZ vector)

#endif
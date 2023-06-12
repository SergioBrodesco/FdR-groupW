/**
 * @file utils.cpp
 * @brief Implementation of commonly used functions.
 * 
 * 
 */
#include "../include/utils.h"

Vector3d W_t_R_transform(const Vector3d& pos){
  // Define translation vector from world frame to robot frame
  Vector3d tvec_robot(0.5, 0.35, 1.75);

  // Define rotation matrix from world frame to robot frame
  Matrix3d world_to_robot_rot;
  world_to_robot_rot << 1, 0, 0,
                        0, -1, 0,
                        0, 0, -1;

  // Apply translation and rotation to robot coordinates
  Vector3d final_position_robot = world_to_robot_rot.transpose() * (pos - tvec_robot);

  return final_position_robot;
}

Vector3d R_t_W_transform(const Vector3d& pos){
  // Define translation vector from world frame to robot frame
  Vector3d tvec_world(-0.5, 0.35, 1.75);

  // Define rotation matrix from robot frame to world frame
  Matrix3d robot_to_world_rot;
  robot_to_world_rot << 1, 0, 0,
                        0, -1, 0,
                        0, 0, -1;

  // Apply translation and rotation to world coordinates
  Vector3d final_position_world = robot_to_world_rot.transpose() * (pos - tvec_world);

  return final_position_world;
}


// Generate matrix in ZYX format
Matrix3d eul2rotmFDR(const Vector3d& eulXYZ){
    Matrix3d R;
    R = AngleAxisd(eulXYZ(2), Vector3d::UnitZ()) * AngleAxisd(eulXYZ(1), Vector3d::UnitY()) * AngleAxisd(eulXYZ(0), Vector3d::UnitX());
    
    return R;
}

// Generate rotation vector in XYZ format
Vector3d rotm2eulFDR(const Matrix3d& R)
{   
    Vector3d eulXYZ;

    float x1 = -asin(R(2,0));
    float x2 = M_PI - x1;

    float y1 = atan2(R(2,1) / cos(x1), R(2,2) / cos(x1));
    float y2 = atan2(R(2,1) / cos(x2), R(2,2) / cos(x2));

    float z1 = atan2(R(1,0) / cos(x1), R(0,0) / cos(x1));
    float z2 = atan2(R(1,0) / cos(x2), R(0,0) / cos(x2));

    //choose one solution to return
    //In this case the "shortest" rotation
    if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
        eulXYZ << x1, y1, z1;
    } else {
        eulXYZ << x2, y2, z2;
    }

    return eulXYZ;
}
